#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "MapPoint.h"
#include "Atlas.h"

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2"),
      m_SLAM(pSLAM)
{
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/point_cloud", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    // Let's use SystemDefaultsQoS which is more forgiving, or specifically SensorDataQoS
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/rgb", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/depth", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryTUM("FrameTrajectoryTUM.txt");
    m_SLAM->SaveTrajectoryKITTI("FrameTrajectoryKITTI.txt");

    // Optional: Save map
    // m_SLAM->SaveMap("MyMap.osa");
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

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
    std::cout << "DEBUG: TrackRGBD finished. Now calling PublishPointCloud..." << std::endl;
    PublishPointCloud();
}

void RgbdSlamNode::PublishPointCloud()
{
    // Retrieve tracked map points
    std::vector<ORB_SLAM3::MapPoint *> mp = m_SLAM->GetTrackedMapPoints();

    if (mp.empty())
    {
        std::cout << "DEBUG: GetTrackedMapPoints returned empty. Nothing to publish this frame." << std::endl;
        return;
    }

    std::cout << "DEBUG: Publishing " << mp.size() << " points to /orb_slam3/point_cloud" << std::endl;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = this->now();
    cloud.header.frame_id = "root"; // publish directly in root frame

    cloud.height = 1;
    cloud.width = 0;
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // We count valid points first, or we can just resize up to mp.size() and adjust width later
    int valid_points = 0;
    for (auto pMP : mp)
    {
        if (pMP && !pMP->isBad())
        {
            valid_points++;
        }
    }

    modifier.resize(valid_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (auto pMP : mp)
    {
        if (pMP && !pMP->isBad())
        {
            auto pos = pMP->GetWorldPos();
            *iter_x = pos(0);
            *iter_y = pos(1);
            *iter_z = pos(2);

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }

    pcl_pub->publish(cloud);
}
