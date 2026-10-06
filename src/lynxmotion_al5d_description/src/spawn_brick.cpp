#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "lynxmotion_al5d_description/cli_args.hpp"
#include "lynxmotion_al5d_description/srv/spawn_brick.hpp"

using lynxmotion_al5d::ParseStatus;

int main(int argc, char ** argv)
{
  const auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  const std::vector<std::string> args(non_ros_args.begin() + 1, non_ros_args.end());
  const auto parsed = lynxmotion_al5d::parseSpawnArgs(args);
  if (parsed.status == ParseStatus::Help) {
    std::cout << lynxmotion_al5d::spawnUsage() << std::endl;
    return 0;
  }
  if (parsed.status == ParseStatus::Error) {
    std::cerr << "spawn_brick: " << parsed.error << "\n" << lynxmotion_al5d::spawnUsage() << std::endl;
    return 2;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spawn_lego_brick");
  auto client = node->create_client<lynxmotion_al5d_description::srv::SpawnBrick>(
    "/lynxmotion_al5d/spawn_brick");

  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "service /lynxmotion_al5d/spawn_brick is not available");
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<lynxmotion_al5d_description::srv::SpawnBrick::Request>();
  const auto & a = parsed.args;
  request->color = a.color;
  request->name = a.name;
  request->pose.position.x = a.x;
  request->pose.position.y = a.y;
  request->pose.position.z = a.z;
  request->pose.orientation.roll = static_cast<float>(a.roll);
  request->pose.orientation.pitch = static_cast<float>(a.pitch);
  request->pose.orientation.yaw = static_cast<float>(a.yaw);

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "failed to call the service");
    rclcpp::shutdown();
    return 1;
  }
  const auto response = future.get();
  int code = 0;
  if (response->success) {
    RCLCPP_INFO(
      node->get_logger(), "Spawned brick [%s] of color [%s] at position (%.2f %.2f %.2f %.2f %.2f %.2f)",
      response->name.c_str(), a.color.c_str(), a.x, a.y, a.z, a.roll, a.pitch, a.yaw);
  } else {
    RCLCPP_ERROR(node->get_logger(), "spawn failed: %s", response->message.c_str());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
