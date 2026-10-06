#include "lynxmotion_al5d_description/brick.hpp"

#include <chrono>
#include <utility>

#include "lynxmotion_al5d_description/pose_utils.hpp"

namespace lynxmotion_al5d
{

namespace
{
geometry_msgs::msg::Pose toGeometryPose(const lynxmotion_al5d_description::msg::Pose & p)
{
  geometry_msgs::msg::Pose out;
  out.position = p.position;
  out.orientation = quaternionFromRPY(p.orientation.roll, p.orientation.pitch, p.orientation.yaw);
  return out;
}
}  // namespace

Brick::Brick(
  SimBackend & backend, std::string color, std::string name,
  const geometry_msgs::msg::Pose & initial_pose)
: backend_(backend), color_(std::move(color)), name_(std::move(name)), pose_(initial_pose)
{
}

std::shared_ptr<Brick> Brick::create(
  rclcpp::Node & node, SimBackend & backend, rclcpp::CallbackGroup::SharedPtr group,
  std::string color, std::string name, const geometry_msgs::msg::Pose & initial_pose)
{
  auto brick = std::make_shared<Brick>(backend, std::move(color), std::move(name), initial_pose);
  brick->init(node, std::move(group));
  return brick;
}

void Brick::init(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group)
{
  std::weak_ptr<Brick> weak = weak_from_this();
  const std::string prefix = "/lynxmotion_al5d/" + name_;
  pose_pub_ = node.create_publisher<lynxmotion_al5d_description::msg::Pose>(prefix + "/pose", 10);
  absolute_srv_ = node.create_service<Absolute>(
    prefix + "/teleport_absolute",
    [weak](const std::shared_ptr<Absolute::Request> req, std::shared_ptr<Absolute::Response> res) {
      if (auto self = weak.lock()) {
        self->onTeleportAbsolute(req, res);
      } else {
        res->success = false;
        res->message = "brick no longer exists";
      }
    },
    rclcpp::ServicesQoS(), group);
  relative_srv_ = node.create_service<Relative>(
    prefix + "/teleport_relative",
    [weak](const std::shared_ptr<Relative::Request> req, std::shared_ptr<Relative::Response> res) {
      if (auto self = weak.lock()) {
        self->onTeleportRelative(req, res);
      } else {
        res->success = false;
        res->message = "brick no longer exists";
      }
    },
    rclcpp::ServicesQoS(), group);
  timer_ = node.create_wall_timer(
    std::chrono::duration<double>(kPublishPeriodSeconds), [weak]() {
      if (auto self = weak.lock()) {
        self->publishPose();
      }
    });
}

geometry_msgs::msg::Pose Brick::pose() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pose_;
}

void Brick::updatePose(const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = pose;
}

void Brick::publishPose()
{
  const auto current = pose();
  const RPY rpy = rpyFromQuaternion(current.orientation);
  lynxmotion_al5d_description::msg::Pose message;
  message.position = current.position;
  message.orientation.roll = static_cast<float>(rpy.roll);
  message.orientation.pitch = static_cast<float>(rpy.pitch);
  message.orientation.yaw = static_cast<float>(rpy.yaw);
  pose_pub_->publish(message);
}

SimResult Brick::teleport(const geometry_msgs::msg::Pose & pose)
{
  return backend_.setPose(name_, pose);
}

void Brick::onTeleportAbsolute(
  const std::shared_ptr<Absolute::Request> request, std::shared_ptr<Absolute::Response> response)
{
  const SimResult result = teleport(toGeometryPose(request->pose));
  response->success = result.ok;
  response->message = result.message;
}

void Brick::onTeleportRelative(
  const std::shared_ptr<Relative::Request> request, std::shared_ptr<Relative::Response> response)
{
  geometry_msgs::msg::Pose goal = pose();
  goal.position.x += request->pose.position.x;
  goal.position.y += request->pose.position.y;
  goal.position.z += request->pose.position.z;

  RPY rpy = rpyFromQuaternion(goal.orientation);
  rpy.roll += request->pose.orientation.roll;
  rpy.pitch += request->pose.orientation.pitch;
  rpy.yaw += request->pose.orientation.yaw;
  goal.orientation = quaternionFromRPY(rpy.roll, rpy.pitch, rpy.yaw);

  const SimResult result = teleport(goal);
  response->success = result.ok;
  response->message = result.message;
}

}  // namespace lynxmotion_al5d
