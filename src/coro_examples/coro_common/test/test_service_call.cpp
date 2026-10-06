#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "coro_common/service_call.hpp"

using namespace std::chrono_literals;
using SetBool = std_srvs::srv::SetBool;

class ServiceCall : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_node_ = std::make_shared<rclcpp::Node>("service_call_client");
    server_node_ = std::make_shared<rclcpp::Node>("service_call_server");
    server_exec_.add_node(server_node_);
  }

  void startServer()
  {
    service_ = server_node_->create_service<SetBool>(
      "/service_call_test",
      [](const std::shared_ptr<SetBool::Request> req, std::shared_ptr<SetBool::Response> res) {
        res->success = req->data;
        res->message = req->data ? "yes" : "no";
      });
    server_thread_ = std::thread([this]() {server_exec_.spin();});
  }

  void TearDown() override
  {
    server_exec_.cancel();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Node> server_node_;
  rclcpp::executors::SingleThreadedExecutor server_exec_;
  rclcpp::Service<SetBool>::SharedPtr service_;
  std::thread server_thread_;
};

TEST_F(ServiceCall, WaitTimesOutWhenServiceMissing)
{
  auto client = client_node_->create_client<SetBool>("/service_call_test");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(coro_common::waitForService(client_node_, client, 700ms));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 5s);
}

TEST_F(ServiceCall, CallReturnsResponse)
{
  startServer();
  auto client = client_node_->create_client<SetBool>("/service_call_test");
  ASSERT_TRUE(coro_common::waitForService(client_node_, client, 10s));

  auto request = std::make_shared<SetBool::Request>();
  request->data = true;
  auto response = coro_common::callService(client_node_, client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "yes");

  request->data = false;
  response = coro_common::callService(client_node_, client, request);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "no");
}

TEST_F(ServiceCall, CallReturnsNullOnTimeout)
{
  auto client = client_node_->create_client<SetBool>("/service_call_test");  // no server
  auto request = std::make_shared<SetBool::Request>();
  EXPECT_EQ(coro_common::callService(client_node_, client, request, 500ms), nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
