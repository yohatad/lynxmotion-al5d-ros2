#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "lynxmotion_al5d_description/cli_args.hpp"
#include "lynxmotion_al5d_description/srv/kill_brick.hpp"

using lynxmotion_al5d::ParseStatus;

int main(int argc, char ** argv)
{
  const auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  const std::vector<std::string> args(non_ros_args.begin() + 1, non_ros_args.end());
  const auto parsed = lynxmotion_al5d::parseKillArgs(args);
  if (parsed.status == ParseStatus::Help) {
    std::cout << lynxmotion_al5d::killUsage() << std::endl;
    return 0;
  }
  if (parsed.status == ParseStatus::Error) {
    std::cerr << "kill_brick: " << parsed.error << "\n" << lynxmotion_al5d::killUsage() << std::endl;
    return 2;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kill_lego_brick");
  auto client = node->create_client<lynxmotion_al5d_description::srv::KillBrick>(
    "/lynxmotion_al5d/kill_brick");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "service /lynxmotion_al5d/kill_brick is not available");
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<lynxmotion_al5d_description::srv::KillBrick::Request>();
  request->name = parsed.name;
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
  if (response->result) {
    RCLCPP_INFO(node->get_logger(), "Brick deleted successfully");
  } else {
    RCLCPP_WARN(node->get_logger(), "%s", response->message.c_str());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
