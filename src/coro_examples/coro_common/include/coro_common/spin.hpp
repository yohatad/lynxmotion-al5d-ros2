#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace coro_common
{

/// Equivalent of ROS 1 ros::spinOnce(): runs the callbacks of *all* messages that have
/// arrived, so a polling control loop always sees the latest data.
///
/// rclcpp::spin_some() is not equivalent: it services roughly one message per subscription
/// per call, so a loop that runs slower than the publisher (for example a 50 Hz loop on a
/// 62.5 Hz pose topic) falls ever further behind and steers on stale data.
inline void spinOnce(
  const rclcpp::Node::SharedPtr & node,
  std::chrono::nanoseconds max_duration = std::chrono::milliseconds(5))
{
  if (!rclcpp::ok()) {
    return;  // interrupted: nothing left to service
  }
  rclcpp::spin_all(node, max_duration);
}

}  // namespace coro_common
