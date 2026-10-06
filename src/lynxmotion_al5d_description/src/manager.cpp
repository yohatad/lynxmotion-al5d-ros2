#include "lynxmotion_al5d_description/manager.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "lynxmotion_al5d_description/gz_backend.hpp"
#include "lynxmotion_al5d_description/pose_utils.hpp"

namespace lynxmotion_al5d
{

namespace
{
std::string capitalise(std::string s)
{
  if (!s.empty()) {
    s[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
  }
  return s;
}
}  // namespace

bool Manager::isValidColor(const std::string & color)
{
  return color == "red" || color == "green" || color == "blue";
}

bool Manager::isValidName(const std::string & name)
{
  if (name.empty() || std::isdigit(static_cast<unsigned char>(name[0]))) {
    return false;
  }
  return std::all_of(
    name.begin(), name.end(), [](unsigned char c) {return std::isalnum(c) || c == '_';});
}

Manager::Manager(const rclcpp::NodeOptions & options, std::unique_ptr<SimBackend> backend)
: rclcpp::Node("sim_manager", "", options), backend_(std::move(backend))
{
  const auto world = declare_parameter<std::string>("world_name", "al5d");
  models_dir_ = declare_parameter<std::string>("models_dir", "");
  service_timeout_ = declare_parameter<double>("service_timeout", 5.0);
  joint_command_topic_ = declare_parameter<std::string>(
    "joint_command_topic", "/lynxmotion_al5d/joints_positions/commands");
  home_position_ = declare_parameter<std::vector<double>>(
    "home_position", std::vector<double>{0.0, 1.57, -1.57, 0.0, 0.0, 0.0});

  if (models_dir_.empty()) {
    models_dir_ =
      ament_index_cpp::get_package_share_directory("lynxmotion_al5d_description") + "/models";
  }
  if (!backend_) {
    backend_ = std::make_unique<GzBackend>(
      this, world, std::chrono::milliseconds(static_cast<int>(service_timeout_ * 1000)));
  }

  service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  data_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto srv_qos = rclcpp::ServicesQoS();
  spawn_srv_ = create_service<SpawnBrick>(
    "/lynxmotion_al5d/spawn_brick",
    [this](const std::shared_ptr<SpawnBrick::Request> q, std::shared_ptr<SpawnBrick::Response> r) {
      onSpawn(q, r);
    },
    srv_qos, service_group_);
  kill_srv_ = create_service<KillBrick>(
    "/lynxmotion_al5d/kill_brick",
    [this](const std::shared_ptr<KillBrick::Request> q, std::shared_ptr<KillBrick::Response> r) {
      onKill(q, r);
    },
    srv_qos, service_group_);
  clear_srv_ = create_service<Empty>(
    "/lynxmotion_al5d/clear",
    [this](const std::shared_ptr<Empty::Request> q, std::shared_ptr<Empty::Response> r) {
      onClear(q, r);
    },
    srv_qos, service_group_);
  reset_srv_ = create_service<Empty>(
    "/lynxmotion_al5d/reset",
    [this](const std::shared_ptr<Empty::Request> q, std::shared_ptr<Empty::Response> r) {
      onReset(q, r);
    },
    srv_qos, service_group_);

  joints_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(joint_command_topic_, 10);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = data_group_;
  pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/world/" + world + "/dynamic_pose/info", rclcpp::SensorDataQoS(),
    [this](const tf2_msgs::msg::TFMessage & m) {onPoses(m);}, sub_options);

  RCLCPP_INFO(get_logger(), "Brick management services ready (world '%s')", world.c_str());
}

std::size_t Manager::brickCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return bricks_.size();
}

bool Manager::loadSdf(const std::string & color, std::string & sdf, std::string & error)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sdf_cache_.find(color);
    if (it != sdf_cache_.end()) {
      sdf = it->second;
      return true;
    }
  }
  const std::string path = models_dir_ + "/" + capitalise(color) + "_Lego_Brick/model.sdf";
  std::ifstream file(path);
  if (!file) {
    error = "cannot read brick model '" + path + "'";
    return false;
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  sdf = buffer.str();
  std::lock_guard<std::mutex> lock(mutex_);
  sdf_cache_[color] = sdf;
  return true;
}

void Manager::onSpawn(
  const std::shared_ptr<SpawnBrick::Request> request, std::shared_ptr<SpawnBrick::Response> response)
{
  response->success = false;

  if (!isValidColor(request->color)) {
    response->message = "unknown colour '" + request->color + "' (use red, green or blue)";
    RCLCPP_ERROR(get_logger(), "spawn_brick: %s", response->message.c_str());
    return;
  }
  if (!request->name.empty() && !isValidName(request->name)) {
    response->message = "invalid brick name '" + request->name + "' (use letters, digits and _)";
    RCLCPP_ERROR(get_logger(), "spawn_brick: %s", response->message.c_str());
    return;
  }

  std::string sdf;
  if (!loadSdf(request->color, sdf, response->message)) {
    RCLCPP_ERROR(get_logger(), "spawn_brick: %s", response->message.c_str());
    return;
  }

  // Reserve the name under the lock so concurrent requests cannot collide.
  std::string name;
  bool auto_named = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto taken = [this](const std::string & n) {
        return bricks_.count(n) > 0 || pending_.count(n) > 0;
      };
    if (request->name.empty()) {
      do {
        name = "brick" + std::to_string(++num_spawned_);
      } while (taken(name));
      auto_named = true;
    } else {
      name = request->name;
      if (taken(name)) {
        response->message = "brick '" + name + "' already exists";
        RCLCPP_ERROR(get_logger(), "spawn_brick: %s", response->message.c_str());
        return;
      }
    }
    pending_.insert(name);
  }

  geometry_msgs::msg::Pose pose;
  pose.position = request->pose.position;
  pose.orientation = quaternionFromRPY(
    request->pose.orientation.roll, request->pose.orientation.pitch,
    request->pose.orientation.yaw);

  const SimResult result = backend_->spawn(name, sdf, pose);

  std::lock_guard<std::mutex> lock(mutex_);
  pending_.erase(name);
  if (!result.ok) {
    if (auto_named) {
      --num_spawned_;
    }
    response->message = result.message;
    RCLCPP_ERROR(get_logger(), "spawn_brick: %s", result.message.c_str());
    return;
  }
  bricks_[name] = Brick::create(*this, *backend_, service_group_, request->color, name, pose);
  response->name = name;
  response->success = true;
}

Manager::KillBrick::Response Manager::kill(const std::string & name)
{
  KillBrick::Response response;
  response.result = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (bricks_.count(name) == 0) {
      response.message = "brick '" + name + "' doesn't exist";
      return response;
    }
  }
  const SimResult result = backend_->remove(name);
  response.message = result.message;
  if (!result.ok) {
    return response;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bricks_.erase(name);
  // Reuse the auto-generated index if this was the most recently generated name.
  if (name == "brick" + std::to_string(num_spawned_)) {
    --num_spawned_;
  }
  response.result = true;
  return response;
}

void Manager::onKill(
  const std::shared_ptr<KillBrick::Request> request, std::shared_ptr<KillBrick::Response> response)
{
  *response = kill(request->name);
  if (!response->result) {
    RCLCPP_ERROR(get_logger(), "kill_brick: %s", response->message.c_str());
  }
}

void Manager::onClear(const std::shared_ptr<Empty::Request>, std::shared_ptr<Empty::Response>)
{
  std::vector<std::string> names;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : bricks_) {
      names.push_back(entry.first);
    }
  }
  for (const auto & name : names) {
    const auto result = kill(name);
    if (!result.result) {
      RCLCPP_WARN(get_logger(), "clear: %s", result.message.c_str());
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  num_spawned_ = 0;
}

void Manager::onReset(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response)
{
  onClear(request, response);

  // Wait (bounded) for the controller to subscribe, then send the arm home.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(service_timeout_);
  while (joints_pub_->get_subscription_count() < 1) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_WARN(
        get_logger(), "reset: nobody subscribes to %s; arm not moved", joint_command_topic_.c_str());
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std_msgs::msg::Float64MultiArray command;
  command.data = home_position_;
  joints_pub_->publish(command);
}

void Manager::onPoses(const tf2_msgs::msg::TFMessage & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & transform : message.transforms) {
    const auto it = bricks_.find(transform.child_frame_id);
    if (it == bricks_.end()) {
      continue;
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    it->second->updatePose(pose);
  }
}

}  // namespace lynxmotion_al5d
