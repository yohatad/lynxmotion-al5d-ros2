#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace coro_common
{

/// ROS 2 replacement for ros::service::waitForService(): blocks until the service of
/// `client` exists. Returns false if ROS is shut down first, or `timeout` (when > 0) expires.
template<typename ClientT>
bool waitForService(
  const rclcpp::Node::SharedPtr & node, const ClientT & client,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
{
  const auto start = std::chrono::steady_clock::now();
  const auto step = std::chrono::milliseconds(500);
  while (!client->wait_for_service(step)) {
    if (!rclcpp::ok()) {
      return false;
    }
    if (timeout.count() > 0 && std::chrono::steady_clock::now() - start >= timeout) {
      return false;
    }
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "waiting for service %s ...",
      client->get_service_name());
  }
  return true;
}

/// ROS 2 replacement for ServiceClient::call(): sends `request`, spins `node` until the
/// response arrives and returns it, or returns nullptr on failure / timeout.
template<typename ClientT, typename RequestPtr>
auto callService(
  const rclcpp::Node::SharedPtr & node, const ClientT & client, const RequestPtr & request,
  std::chrono::milliseconds timeout = std::chrono::seconds(10))
-> typename ClientT::element_type::SharedResponse
{
  auto pending = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, pending, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    client->remove_pending_request(pending);
    return nullptr;
  }
  return pending.get();
}

}  // namespace coro_common
