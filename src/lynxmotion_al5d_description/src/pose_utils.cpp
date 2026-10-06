#include "lynxmotion_al5d_description/pose_utils.hpp"

#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace lynxmotion_al5d
{

double radians(double degrees) {return degrees * M_PI / 180.0;}

double degrees(double radians) {return radians * 180.0 / M_PI;}

geometry_msgs::msg::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  quat.normalize();
  geometry_msgs::msg::Quaternion out;
  out.x = quat.x();
  out.y = quat.y();
  out.z = quat.z();
  out.w = quat.w();
  return out;
}

RPY rpyFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  quat.normalize();
  RPY values;
  tf2::Matrix3x3(quat).getRPY(values.roll, values.pitch, values.yaw);
  return values;
}

}  // namespace lynxmotion_al5d
