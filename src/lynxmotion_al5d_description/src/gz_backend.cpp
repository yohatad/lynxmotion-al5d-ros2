#include "lynxmotion_al5d_description/gz_backend.hpp"

#include <future>
#include <string>
#include <utility>

#include "ros_gz_interfaces/msg/entity.hpp"

namespace lynxmotion_al5d
{

GzBackend::GzBackend(
  rclcpp::Node * node, const std::string & world, std::chrono::milliseconds timeout)
: node_(node), timeout_(timeout)
{
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const std::string prefix = "/world/" + world;
  spawn_client_ = node_->create_client<ros_gz_interfaces::srv::SpawnEntity>(
    prefix + "/create", rclcpp::ServicesQoS(), group_);
  remove_client_ = node_->create_client<ros_gz_interfaces::srv::DeleteEntity>(
    prefix + "/remove", rclcpp::ServicesQoS(), group_);
  pose_client_ = node_->create_client<ros_gz_interfaces::srv::SetEntityPose>(
    prefix + "/set_pose", rclcpp::ServicesQoS(), group_);
}

template<typename ClientT>
SimResult GzBackend::call(
  ClientT & client, typename ClientT::SharedRequest request, const std::string & what)
{
  if (!client.wait_for_service(timeout_)) {
    return {false, what + ": simulator service " + client.get_service_name() + " unavailable"};
  }
  auto pending = client.async_send_request(std::move(request));
  if (pending.future.wait_for(timeout_) != std::future_status::ready) {
    client.remove_pending_request(pending.request_id);
    return {false, what + ": timed out waiting for the simulator"};
  }
  if (!pending.future.get()->success) {
    return {false, what + ": the simulator rejected the request"};
  }
  return {true, ""};
}

SimResult GzBackend::spawn(
  const std::string & name, const std::string & sdf, const geometry_msgs::msg::Pose & pose)
{
  auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
  request->entity_factory.name = name;
  request->entity_factory.allow_renaming = false;
  request->entity_factory.sdf = sdf;
  request->entity_factory.pose = pose;
  return call(*spawn_client_, request, "spawn '" + name + "'");
}

SimResult GzBackend::remove(const std::string & name)
{
  auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
  request->entity.name = name;
  request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
  return call(*remove_client_, request, "remove '" + name + "'");
}

SimResult GzBackend::setPose(const std::string & name, const geometry_msgs::msg::Pose & pose)
{
  auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
  request->entity.name = name;
  request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
  request->pose = pose;
  return call(*pose_client_, request, "teleport '" + name + "'");
}

}  // namespace lynxmotion_al5d
