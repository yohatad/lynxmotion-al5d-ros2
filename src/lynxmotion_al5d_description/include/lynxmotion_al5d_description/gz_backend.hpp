#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/delete_entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"

#include "lynxmotion_al5d_description/sim_backend.hpp"

namespace lynxmotion_al5d
{

/// SimBackend over the ros_gz_bridge service bridges
///   /world/<world>/create, /world/<world>/remove, /world/<world>/set_pose.
/// Calls block (with a timeout). They must be made from a callback group other than the
/// one servicing the clients, with a multi-threaded executor spinning the node.
class GzBackend : public SimBackend
{
public:
  GzBackend(rclcpp::Node * node, const std::string & world, std::chrono::milliseconds timeout);

  SimResult spawn(
    const std::string & name, const std::string & sdf,
    const geometry_msgs::msg::Pose & pose) override;
  SimResult remove(const std::string & name) override;
  SimResult setPose(const std::string & name, const geometry_msgs::msg::Pose & pose) override;

private:
  template<typename ClientT>
  SimResult call(
    ClientT & client, typename ClientT::SharedRequest request, const std::string & what);

  rclcpp::Node * node_;
  std::chrono::milliseconds timeout_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr remove_client_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr pose_client_;
};

}  // namespace lynxmotion_al5d
