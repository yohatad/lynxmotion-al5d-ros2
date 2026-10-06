#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "lynxmotion_al5d_description/brick.hpp"
#include "lynxmotion_al5d_description/sim_backend.hpp"
#include "lynxmotion_al5d_description/srv/kill_brick.hpp"
#include "lynxmotion_al5d_description/srv/spawn_brick.hpp"

namespace lynxmotion_al5d
{

/// Brick management node. Services (all under /lynxmotion_al5d):
///   spawn_brick, kill_brick, clear, reset
/// Parameters:
///   world_name (string, "al5d")          Gazebo world that bricks live in
///   models_dir (string, "")              directory holding <Color>_Lego_Brick/model.sdf
///                                        (default: the package's installed models/)
///   service_timeout (double, 5.0 s)      timeout for simulator requests
///   joint_command_topic (string)         topic used by `reset` to send the arm home
///   home_position (double[6])            joint positions used by `reset`
/// Must be spun by a multi-threaded executor (service handlers block on the simulator).
class Manager : public rclcpp::Node
{
public:
  /// `backend` defaults to a GzBackend; tests inject a fake.
  explicit Manager(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    std::unique_ptr<SimBackend> backend = nullptr);

  std::size_t brickCount() const;

  /// Accepted brick colours.
  static bool isValidColor(const std::string & color);
  /// Names become ROS topic/service tokens, so only [A-Za-z0-9_] (not starting with a digit).
  static bool isValidName(const std::string & name);

private:
  using SpawnBrick = lynxmotion_al5d_description::srv::SpawnBrick;
  using KillBrick = lynxmotion_al5d_description::srv::KillBrick;
  using Empty = std_srvs::srv::Empty;

  void onSpawn(
    const std::shared_ptr<SpawnBrick::Request> request, std::shared_ptr<SpawnBrick::Response> response);
  void onKill(
    const std::shared_ptr<KillBrick::Request> request, std::shared_ptr<KillBrick::Response> response);
  void onClear(const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>);
  void onReset(const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>);
  void onPoses(const tf2_msgs::msg::TFMessage & message);

  /// Core of kill_brick, reused by clear.
  KillBrick::Response kill(const std::string & name);
  bool loadSdf(const std::string & color, std::string & sdf, std::string & error);

  std::unique_ptr<SimBackend> backend_;
  std::string models_dir_;
  double service_timeout_;
  std::string joint_command_topic_;
  std::vector<double> home_position_;

  mutable std::mutex mutex_;
  std::map<std::string, std::shared_ptr<Brick>> bricks_;
  std::set<std::string> pending_;  // names reserved by in-flight spawns
  std::map<std::string, std::string> sdf_cache_;
  int num_spawned_{0};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr data_group_;
  rclcpp::Service<SpawnBrick>::SharedPtr spawn_srv_;
  rclcpp::Service<KillBrick>::SharedPtr kill_srv_;
  rclcpp::Service<Empty>::SharedPtr clear_srv_;
  rclcpp::Service<Empty>::SharedPtr reset_srv_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_pub_;
};

}  // namespace lynxmotion_al5d
