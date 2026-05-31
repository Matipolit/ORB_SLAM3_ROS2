#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = static_cast<double>(stamp.sec) + (static_cast<double>(stamp.nanosec) * 1e-9);
    return seconds;
  }

  static bool IsFinite(const geometry_msgs::msg::Vector3 &v)
  {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
};

#endif
