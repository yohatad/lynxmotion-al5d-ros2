#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lynxmotion_al5d_description/msg/pose.hpp"
#include "lynxmotion_al5d_description/sim_backend.hpp"
#include "lynxmotion_al5d_description/srv/teleport_absolute.hpp"
#include "lynxmotion_al5d_description/srv/teleport_relative.hpp"

namespace lynxmotion_al5d
{

/// A Lego brick living in the simulation. Publishes its pose on
/// /lynxmotion_al5d/<name>/pose and offers teleport_absolute / teleport_relative services.
/// The pose is kept up to date by Manager from the simulator's pose stream.
class Brick : public std::enable_shared_from_this<Brick>
{
public:
  static constexpr double kPublishPeriodSeconds = 0.1;

  /// Bricks must be created through create(): ROS callbacks hold weak references so that a
  /// brick removed while one of its services is executing is never used after destruction.
  static std::shared_ptr<Brick> create(
    rclcpp::Node & node, SimBackend & backend, rclcpp::CallbackGroup::SharedPtr group,
    std::string color, std::string name, const geometry_msgs::msg::Pose & initial_pose);

  Brick(
    SimBackend & backend, std::string color, std::string name,
    const geometry_msgs::msg::Pose & initial_pose);
  Brick(const Brick &) = delete;
  Brick & operator=(const Brick &) = delete;

  const std::string & name() const {return name_;}
  const std::string & color() const {return color_;}
  geometry_msgs::msg::Pose pose() const;
  void updatePose(const geometry_msgs::msg::Pose & pose);

  /// Absolute teleport; the result carries the simulator's refusal reason, if any.
  SimResult teleport(const geometry_msgs::msg::Pose & pose);

private:
  using Absolute = lynxmotion_al5d_description::srv::TeleportAbsolute;
  using Relative = lynxmotion_al5d_description::srv::TeleportRelative;

  void init(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group);
  void publishPose();
  void onTeleportAbsolute(
    const std::shared_ptr<Absolute::Request> request, std::shared_ptr<Absolute::Response> response);
  void onTeleportRelative(
    const std::shared_ptr<Relative::Request> request, std::shared_ptr<Relative::Response> response);

  SimBackend & backend_;
  std::string color_;
  std::string name_;
  mutable std::mutex mutex_;
  geometry_msgs::msg::Pose pose_;

  rclcpp::Publisher<lynxmotion_al5d_description::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Service<Absolute>::SharedPtr absolute_srv_;
  rclcpp::Service<Relative>::SharedPtr relative_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lynxmotion_al5d
