/* This is a ROS 2 version of the standard "Hello , World" program */

#include "rclcpp/rclcpp.hpp"                   // This header defines the standard ROS classes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                    // Initialize the ROS system
  auto node = rclcpp::Node::make_shared("hello_world");  // Register this program as a ROS node
  RCLCPP_INFO_STREAM(node->get_logger(), "Hello World!");  // Send some output as a log message
  rclcpp::shutdown();
  return 0;
}
