#pragma once

#include "geometry_msgs/msg/quaternion.hpp"

namespace lynxmotion_al5d
{

struct RPY
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

double radians(double degrees);
double degrees(double radians);

/// Normalised quaternion from fixed-axis roll/pitch/yaw in radians.
geometry_msgs::msg::Quaternion quaternionFromRPY(double roll, double pitch, double yaw);

/// Roll/pitch/yaw in radians from a quaternion (need not be normalised).
RPY rpyFromQuaternion(const geometry_msgs::msg::Quaternion & q);

}  // namespace lynxmotion_al5d
