#include "rgbd-slam-node.hpp"

#include <cmath>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "MapPoint.h"
#include "Atlas.h"

using std::placeholders::_1;

namespace
{
    bool DepthEncodingLooksSupported(const std::string &encoding)
    {
        return encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
               encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    }

    bool DepthHasNonFiniteValues(const cv::Mat &depth, cv::Point *bad_pos)
    {
        if (depth.empty())
        {
            return true;
        }

        const int depth_type = depth.depth();
        if (depth_type == CV_32F || depth_type == CV_64F)
        {
            return !cv::checkRange(depth, true, bad_pos);
        }

        return false;
    }

    cv::Mat SanitizeDepthImage(const cv::Mat &depth, size_t *invalid_count)
    {
        if (invalid_count)
        {
            *invalid_count = 0;
        }

        if (depth.empty())
        {
            return depth;
        }

        if (depth.depth() == CV_32F)
        {
            cv::Mat sanitized = depth.clone();
            for (int y = 0; y < sanitized.rows; ++y)
            {
                float *row = sanitized.ptr<float>(y);
                for (int x = 0; x < sanitized.cols; ++x)
                {
                    if (!std::isfinite(row[x]))
                    {
                        row[x] = 0.0f;
                        if (invalid_count)
                        {
                            ++(*invalid_count);
                        }
                    }
                }
            }
            return sanitized;
        }

        if (depth.depth() == CV_64F)
        {
            cv::Mat sanitized = depth.clone();
            for (int y = 0; y < sanitized.rows; ++y)
            {
                double *row = sanitized.ptr<double>(y);
                for (int x = 0; x < sanitized.cols; ++x)
                {
                    if (!std::isfinite(row[x]))
                    {
                        row[x] = 0.0;
                        if (invalid_count)
                        {
                            ++(*invalid_count);
                        }
                    }
                }
            }
            return sanitized;
        }

        return depth;
    }
}

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System *pSLAM, bool use_imu)
    : Node("ORB_SLAM3_ROS2"),
      m_SLAM(pSLAM),
      use_imu_(use_imu)
{
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/point_cloud", 10);
    finalized_ = false;

    pointcloud_enable_quality_filter_ = this->declare_parameter<bool>("pointcloud_quality_filter", true);
    pointcloud_min_observations_ = std::max(1, static_cast<int>(this->declare_parameter<int>("pointcloud_min_observations", 3)));
    pointcloud_min_found_ratio_ = this->declare_parameter<double>("pointcloud_min_found_ratio", 0.25);
    pointcloud_min_found_ratio_ = std::max(0.0, std::min(1.0, pointcloud_min_found_ratio_));
    pointcloud_max_distance_ = this->declare_parameter<double>("pointcloud_max_distance", 0.0);
    pointcloud_max_distance_ = std::max(0.0, pointcloud_max_distance_);
    pointcloud_apply_optical_to_ros_transform_ = this->declare_parameter<bool>("pointcloud_apply_optical_to_ros_transform", true);
    export_final_map_pcd_ = this->declare_parameter<bool>("export_final_map_pcd", true);
    final_map_pcd_path_ = this->declare_parameter<std::string>("final_map_pcd_path", "orbslam3_final_map.pcd");

    const bool declared_use_imu = this->declare_parameter<bool>("use_imu", use_imu_);
    if (declared_use_imu != use_imu_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "use_imu parameter (%s) differs from SLAM sensor mode (%s). Using sensor mode value.",
            declared_use_imu ? "true" : "false",
            use_imu_ ? "true" : "false");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Point cloud quality filter: %s (min_observations=%d, min_found_ratio=%.2f)",
        pointcloud_enable_quality_filter_ ? "enabled" : "disabled",
        pointcloud_min_observations_,
        pointcloud_min_found_ratio_);

    const std::string max_distance_text = pointcloud_max_distance_ > 0.0 ? std::to_string(pointcloud_max_distance_) : "disabled";
    RCLCPP_INFO(
        this->get_logger(),
        "Point cloud output transform: %s, max_distance=%s",
        pointcloud_apply_optical_to_ros_transform_ ? "optical_to_ros" : "none",
        max_distance_text.c_str());

    RCLCPP_INFO(
        this->get_logger(),
        "Final map export: %s (path=%s)",
        export_final_map_pcd_ ? "enabled" : "disabled",
        final_map_pcd_path_.c_str());

    RCLCPP_INFO(
        this->get_logger(),
        "RGB-D IMU support: %s (topic=imu)",
        use_imu_ ? "enabled" : "disabled");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    // Let's use SystemDefaultsQoS which is more forgiving, or specifically SensorDataQoS
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/rgb", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/depth", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    if (use_imu_)
    {
        imu_sub_ = this->create_subscription<ImuMsg>("imu", rclcpp::SensorDataQoS(), std::bind(&RgbdSlamNode::GrabImu, this, _1));
    }

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    graph_pub_ = std::make_shared<GraphPublisher>(this);
}

RgbdSlamNode::~RgbdSlamNode()
{
    FinalizeSlamAndOutputs();
}

void RgbdSlamNode::FinalizeAndSaveOutputs()
{
    FinalizeSlamAndOutputs();
}

void RgbdSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buf_.push(msg);
}

bool RgbdSlamNode::FillImuMeasurements(double timestamp, std::vector<ORB_SLAM3::IMU::Point> *measurements)
{
    if (!measurements)
    {
        return false;
    }

    measurements->clear();

    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu_buf_.empty())
    {
        return false;
    }

    if (Utility::StampToSec(imu_buf_.back()->header.stamp) < timestamp)
    {
        return false;
    }

    while (!imu_buf_.empty() && Utility::StampToSec(imu_buf_.front()->header.stamp) <= timestamp)
    {
        const auto &imu_msg = imu_buf_.front();

        if (!Utility::IsFinite(imu_msg->linear_acceleration) || !Utility::IsFinite(imu_msg->angular_velocity))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "IMU measurement has non-finite values. Skipping.");
            imu_buf_.pop();
            continue;
        }

        const double t = Utility::StampToSec(imu_msg->header.stamp);
        cv::Point3f acc(
            imu_msg->linear_acceleration.x,
            imu_msg->linear_acceleration.y,
            imu_msg->linear_acceleration.z);
        cv::Point3f gyr(
            imu_msg->angular_velocity.x,
            imu_msg->angular_velocity.y,
            imu_msg->angular_velocity.z);
        measurements->push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
        imu_buf_.pop();
    }

    return true;
}

bool RgbdSlamNode::IsPointUsable(ORB_SLAM3::MapPoint *pMP) const
{
    if (!pMP || pMP->isBad())
    {
        return false;
    }

    const Eigen::Vector3f pos = TransformPointForOutput(pMP->GetWorldPos());
    if (!std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2)))
    {
        return false;
    }

    if (pointcloud_max_distance_ > 0.0 && pos.norm() > pointcloud_max_distance_)
    {
        return false;
    }

    if (!pointcloud_enable_quality_filter_)
    {
        return true;
    }

    if (pMP->Observations() < pointcloud_min_observations_)
    {
        return false;
    }

    if (static_cast<double>(pMP->GetFoundRatio()) < pointcloud_min_found_ratio_)
    {
        return false;
    }

    return true;
}

Eigen::Vector3f RgbdSlamNode::TransformPointForOutput(const Eigen::Vector3f &point) const
{
    if (!pointcloud_apply_optical_to_ros_transform_)
    {
        return point;
    }

    // Convert camera optical-style axes (x right, y down, z forward)
    // into a ROS base-like convention (x forward, y left, z up).
    return Eigen::Vector3f(point(2), -point(0), -point(1));
}

std::vector<Eigen::Vector3f> RgbdSlamNode::CollectUsablePoints(const std::vector<ORB_SLAM3::MapPoint *> &map_points) const
{
    std::vector<Eigen::Vector3f> points;
    points.reserve(map_points.size());

    for (auto pMP : map_points)
    {
        if (IsPointUsable(pMP))
        {
            points.push_back(TransformPointForOutput(pMP->GetWorldPos()));
        }
    }

    return points;
}

void RgbdSlamNode::WritePcdAscii(const std::string &file_path, const std::vector<Eigen::Vector3f> &points) const
{
    std::ofstream out(file_path);
    if (!out.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open final map PCD file: %s", file_path.c_str());
        return;
    }

    out << "# .PCD v0.7\n";
    out << "VERSION 0.7\n";
    out << "FIELDS x y z\n";
    out << "SIZE 4 4 4\n";
    out << "TYPE F F F\n";
    out << "COUNT 1 1 1\n";
    out << "WIDTH " << points.size() << "\n";
    out << "HEIGHT 1\n";
    out << "VIEWPOINT 0 0 0 1 0 0 0\n";
    out << "POINTS " << points.size() << "\n";
    out << "DATA ascii\n";
    out << std::fixed << std::setprecision(6);

    for (const auto &p : points)
    {
        out << p(0) << " " << p(1) << " " << p(2) << "\n";
    }

    out.close();
}

void RgbdSlamNode::ExportFinalMapPcd()
{
    if (!export_final_map_pcd_)
    {
        return;
    }

    const std::vector<ORB_SLAM3::MapPoint *> all_map_points = m_SLAM->GetAllMapPoints();
    if (all_map_points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Final map export skipped: ORB-SLAM returned no map points.");
        return;
    }

    const std::vector<Eigen::Vector3f> filtered_points = CollectUsablePoints(all_map_points);
    if (filtered_points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Final map export skipped: all map points were filtered out.");
        return;
    }

    WritePcdAscii(final_map_pcd_path_, filtered_points);
    RCLCPP_INFO(
        this->get_logger(),
        "Final ORB map exported to %s (%zu/%zu points kept).",
        final_map_pcd_path_.c_str(),
        filtered_points.size(),
        all_map_points.size());
}

void RgbdSlamNode::FinalizeSlamAndOutputs()
{
    if (finalized_)
    {
        return;
    }
    finalized_ = true;

    m_SLAM->Shutdown();
    ExportFinalMapPcd();

    if (graph_pub_)
    {
        graph_pub_->SaveGraphs(m_SLAM, "covisibility_graph.txt", "essential_graph.txt");
    }

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryTUM("FrameTrajectoryTUM.txt");
    m_SLAM->SaveTrajectoryKITTI("FrameTrajectoryKITTI.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "GrabRGBD called for the first time! Synced an RGB and Depth pair.");

    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptrD->image.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Depth image is empty. Skipping frame.");
        return;
    }

    static bool logged_depth_encoding = false;
    if (!logged_depth_encoding && !DepthEncodingLooksSupported(msgD->encoding))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Unexpected depth encoding '%s'. Expected 16UC1 or 32FC1.",
            msgD->encoding.c_str());
        logged_depth_encoding = true;
    }

    cv::Mat depth_for_slam = cv_ptrD->image;
    cv::Point bad_pos;
    if (DepthHasNonFiniteValues(cv_ptrD->image, &bad_pos))
    {
        size_t invalid_count = 0;
        depth_for_slam = SanitizeDepthImage(cv_ptrD->image, &invalid_count);

        double min_val = 0.0;
        double max_val = 0.0;
        cv::minMaxLoc(depth_for_slam, &min_val, &max_val);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Depth image has non-finite values (encoding=%s, type=%d, bad_pos=%d,%d, invalid=%zu, min=%.6f, max=%.6f). Replacing with 0.",
            msgD->encoding.c_str(),
            cv_ptrD->image.type(),
            bad_pos.x,
            bad_pos.y,
            invalid_count,
            min_val,
            max_val);
    }

    const double timestamp = Utility::StampToSec(msgRGB->header.stamp);
    std::vector<ORB_SLAM3::IMU::Point> imu_meas;
    if (use_imu_)
    {
        if (!FillImuMeasurements(timestamp, &imu_meas))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for IMU data up to t=%.6f.",
                timestamp);
            return;
        }
    }

    m_SLAM->TrackRGBD(cv_ptrRGB->image, depth_for_slam, timestamp, imu_meas);

    // Check tracking state before publishing
    if (m_SLAM->GetTrackingState() == 2)
    {
        std::cout << "DEBUG: TrackRGBD finished. Now calling PublishPointCloud..." << std::endl;
        PublishPointCloud(msgRGB->header.stamp);
        if (graph_pub_)
        {
            graph_pub_->PublishGraphs(m_SLAM, msgRGB->header.stamp);
        }
    }
}

void RgbdSlamNode::PublishPointCloud(const builtin_interfaces::msg::Time &stamp)
{
    // Retrieve tracked map points
    std::vector<ORB_SLAM3::MapPoint *> mp = m_SLAM->GetTrackedMapPoints();

    if (mp.empty())
    {
        std::cout << "DEBUG: GetTrackedMapPoints returned empty. Nothing to publish this frame." << std::endl;
        return;
    }

    const std::vector<Eigen::Vector3f> filtered_points = CollectUsablePoints(mp);
    if (filtered_points.empty())
    {
        std::cout << "DEBUG: All tracked points filtered out. Nothing to publish this frame." << std::endl;
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "root"; // publish directly in root frame

    cloud.height = 1;
    cloud.width = 0;
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    std::cout << "DEBUG: Publishing " << filtered_points.size() << "/" << mp.size() << " filtered points to /orb_slam3/point_cloud" << std::endl;

    modifier.resize(filtered_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto &pos : filtered_points)
    {
        *iter_x = pos(0);
        *iter_y = pos(1);
        *iter_z = pos(2);

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    pcl_pub->publish(cloud);
}
