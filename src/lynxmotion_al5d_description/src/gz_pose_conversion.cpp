#include "lynxmotion_al5d_description/gz_pose_conversion.hpp"

namespace lynxmotion_al5d
{

tf2_msgs::msg::TFMessage poseVectorToTf(
  const gz::msgs::Pose_V & poses, const std::string & world_frame)
{
  tf2_msgs::msg::TFMessage out;
  out.transforms.reserve(static_cast<std::size_t>(poses.pose_size()));
  for (const auto & pose : poses.pose()) {
    if (pose.name().empty()) {
      continue;
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp.sec = static_cast<int32_t>(poses.header().stamp().sec());
    t.header.stamp.nanosec = static_cast<uint32_t>(poses.header().stamp().nsec());
    t.header.frame_id = world_frame;
    t.child_frame_id = pose.name();
    t.transform.translation.x = pose.position().x();
    t.transform.translation.y = pose.position().y();
    t.transform.translation.z = pose.position().z();
    t.transform.rotation.x = pose.orientation().x();
    t.transform.rotation.y = pose.orientation().y();
    t.transform.rotation.z = pose.orientation().z();
    t.transform.rotation.w = pose.orientation().w();
    out.transforms.push_back(t);
  }
  return out;
}

}  // namespace lynxmotion_al5d
