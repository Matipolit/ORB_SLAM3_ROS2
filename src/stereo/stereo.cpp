#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    std::cout << "==========================================================" << std::endl;
    std::cout << "ORB-SLAM3 ROS2 Wrapper Version: " << ORB_SLAM3_VERSION << std::endl;
    std::cout << "Built against ORB-SLAM3 at: " << ORB_SLAM3_LIB_PATH << std::endl;
    std::cout << "Mode: Stereo" << std::endl;
    std::cout << "==========================================================" << std::endl;

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = false;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], argv[3]);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
