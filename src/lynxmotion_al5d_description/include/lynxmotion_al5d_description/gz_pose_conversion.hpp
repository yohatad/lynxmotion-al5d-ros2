#pragma once

#include <string>

#include <gz/msgs/pose_v.pb.h>

#include "tf2_msgs/msg/tf_message.hpp"

namespace lynxmotion_al5d
{

/// Convert Gazebo's Pose_V (as published on /world/<w>/dynamic_pose/info) to a TFMessage.
///
/// ros_gz_bridge cannot do this usefully: it takes child frame names from header metadata that
/// this topic does not carry, so every transform arrives with an empty child_frame_id.
/// Here the entity name becomes child_frame_id and the world is the parent frame.
/// Entries without a name are dropped.
tf2_msgs::msg::TFMessage poseVectorToTf(
  const gz::msgs::Pose_V & poses, const std::string & world_frame = "world");

}  // namespace lynxmotion_al5d
