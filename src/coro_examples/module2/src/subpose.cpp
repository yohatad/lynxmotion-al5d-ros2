/* This program subscribes to turtle1/pose and shows its messages on the screen */

#include <iomanip>  // for std::setprecision and std::fixed
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

/* A callback function. Executed each time a new pose message arrives */
void poseMessageReceived(const turtlesim::msg::Pose & msg, const rclcpp::Logger & logger)
{
  RCLCPP_INFO_STREAM(
    logger, std::setprecision(2) << std::fixed <<
      "position=(" << msg.x << "," << msg.y << ")" <<
      " direction=" << msg.theta);
}

int main(int argc, char ** argv)
{
  /* Initialize the ROS system and become a node */
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscribe_to_pose");

  /* Create a subscriber object */
  auto sub = node->create_subscription<turtlesim::msg::Pose>(
    "turtle1/pose", 1000,
    [logger = node->get_logger()](const turtlesim::msg::Pose & msg) {
      poseMessageReceived(msg, logger);
    });

  /* Let ROS take over */
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
