#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = static_cast<double>(stamp.sec) + (static_cast<double>(stamp.nanosec) * 1e-9);
    return seconds;
  }
};

#endif
