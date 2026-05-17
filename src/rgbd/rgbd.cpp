#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

namespace
{
    bool ParseBool(const std::string &value, bool fallback)
    {
        std::string normalized = value;
        std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                       [](unsigned char c)
                       { return static_cast<char>(std::tolower(c)); });

        if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on")
        {
            return true;
        }
        if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off")
        {
            return false;
        }

        return fallback;
    }
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = false;
    {
        auto bootstrap_node = std::make_shared<rclcpp::Node>("orbslam3_rgbd_bootstrap");
        visualization = bootstrap_node->declare_parameter<bool>("enable_viewer", false);

        const char *viewer_env = std::getenv("ORB_SLAM3_ENABLE_VIEWER");
        if (viewer_env != nullptr)
        {
            visualization = ParseBool(viewer_env, visualization);
        }

        RCLCPP_INFO(
            bootstrap_node->get_logger(),
            "ORB-SLAM3 RGB-D viewer: %s (set 'enable_viewer' param or ORB_SLAM3_ENABLE_VIEWER env)",
            visualization ? "enabled" : "disabled");
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    auto node = std::make_shared<RgbdSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    node->FinalizeAndSaveOutputs();
    rclcpp::shutdown();

    return 0;
}
