/* This program publishes randomly-generated velocity messages for turtlesim */

#include <cstdlib>                          // For rand() and RAND_MAX
#include <ctime>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"      // For geometry_msgs::msg::Twist
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                  // Initialize the ROS system
  auto node = rclcpp::Node::make_shared("publish_velocity");  // Become a node
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1000);
  srand(time(0));        // Seed the random number generator
  rclcpp::Rate rate(2);  // Loop at 2Hz until the node is shut down

  while (rclcpp::ok()) {
    geometry_msgs::msg::Twist msg;                                  // Create the message
    msg.linear.x = static_cast<double>(rand()) / RAND_MAX;          // fill in the fields
    msg.angular.z = 2 * static_cast<double>(rand()) / RAND_MAX - 1;  // other fields default to 0
    pub->publish(msg);                                              // Publish the message

    /* Send a message to rosout with the details */
    RCLCPP_INFO_STREAM(
      node->get_logger(), "Sending random velocity command:" <<
        " linear =" << msg.linear.x << " angular =" << msg.angular.z);
    rate.sleep();  // Wait until it is time for another iteration
  }
  rclcpp::shutdown();
  return 0;
}
