/* This client program uses a sample of turtlesim services */
/* /clear, /turtle1/set_pen, and /turtle1/teleport_absolute */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"               // for reset and clear services
#include "turtlesim/srv/set_pen.hpp"            // for turtle1/set_pen service
#include "turtlesim/srv/teleport_absolute.hpp"  // for turtle1/teleport_absolute service

using namespace std::chrono_literals;

/* Wait for a service to appear; false if ROS is shut down first */
template<typename ClientT>
static bool waitFor(const rclcpp::Node::SharedPtr & node, const ClientT & client)
{
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service %s ...", client->get_service_name());
  }
  return true;
}

/* Send a request and block until the response arrives; false on failure or timeout */
template<typename ClientT, typename RequestPtr>
static bool callService(
  const rclcpp::Node::SharedPtr & node, const ClientT & client, const RequestPtr & request)
{
  auto future = client->async_send_request(request);
  return rclcpp::spin_until_future_complete(node, future, 10s) ==
         rclcpp::FutureReturnCode::SUCCESS;
}

int main(int argc, char ** argv)
{
  bool success = true;

  /* Initialize the ROS system and become a node */
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dvernon");

  /* Create client objects for the required services */
  auto teleportClient = node->create_client<turtlesim::srv::TeleportAbsolute>(
    "turtle1/teleport_absolute");
  auto setpenClient = node->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
  auto clearClient = node->create_client<std_srvs::srv::Empty>("clear");
  if (!waitFor(node, teleportClient) || !waitFor(node, setpenClient) ||
    !waitFor(node, clearClient))
  {
    rclcpp::shutdown();
    return 1;
  }

  int status = 0;

  /* clear the simulator background */
  success = callService(node, clearClient, std::make_shared<std_srvs::srv::Empty::Request>());
  if (!success) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Turtle failed to clear");
    status = 1;
  }

  /* turn the pen off so that we do not see a trace when the turtle teleports */
  auto pen_arguments = std::make_shared<turtlesim::srv::SetPen::Request>();
  pen_arguments->off = 1;
  success = callService(node, setpenClient, pen_arguments);
  if (!success) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "TurtlePen failed to switch off");
    status = 1;
  }

  auto teleport_arguments = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
  teleport_arguments->x = 2.5;              // location
  teleport_arguments->y = 3.5;              // coordinates
  teleport_arguments->theta = 3.14159 / 2;  // facing up, i.e. 90 degrees

  success = callService(node, teleportClient, teleport_arguments);
  if (!success) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Turtle failed to teleport");
    status = 1;
  }

  /* turn the pen on again so that we do see a trace when the turtle moves later on */
  pen_arguments = std::make_shared<turtlesim::srv::SetPen::Request>();
  pen_arguments->off = 0;
  pen_arguments->r = 255;    // white
  pen_arguments->g = 255;    // pen
  pen_arguments->b = 255;    // colour
  pen_arguments->width = 1;  // narrow line

  success = callService(node, setpenClient, pen_arguments);
  if (!success) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "TurtlePen failed to switch on");
    status = 1;
  }

  rclcpp::shutdown();
  return status;
}
