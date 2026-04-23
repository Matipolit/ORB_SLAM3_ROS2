#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "MapPoint.h"
#include "Atlas.h"

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/point_cloud", 10);

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/depth");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
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
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
    PublishPointCloud();
}

void RgbdSlamNode::PublishPointCloud()
{
    // Retrieve all map points
    // Note: If you only want currently tracked points, you can use:
    // std::vector<ORB_SLAM3::MapPoint*> mp = m_SLAM->GetTrackedMapPoints();
    std::vector<ORB_SLAM3::MapPoint*> mp;
    
    // Attempting to safely get points from the active map or atlas
    if(m_SLAM->GetAtlas()){
        std::vector<ORB_SLAM3::Map*> maps = m_SLAM->GetAtlas()->GetAllMaps();
        for(ORB_SLAM3::Map* map : maps) {
            std::vector<ORB_SLAM3::MapPoint*> map_pts = map->GetAllMapPoints();
            mp.insert(mp.end(), map_pts.begin(), map_pts.end());
        }
    }

    if (mp.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = this->now();
    cloud.header.frame_id = "map"; // the frame in which points are located

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
        if (pMP && !pMP->isBad()) {
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
            cv::Mat pos = pMP->GetWorldPos();
            *iter_x = pos.at<float>(0);
            *iter_y = pos.at<float>(1);
            *iter_z = pos.at<float>(2);
            
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }

    pcl_pub->publish(cloud);
}
