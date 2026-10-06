#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "coro_common/spin.hpp"

using namespace std::chrono_literals;

class SpinOnce : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("spin_once_node");
    pub_ = node_->create_publisher<std_msgs::msg::Int32>("/spin_once_test", 1000);
    sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/spin_once_test", 1000, [this](const std_msgs::msg::Int32 & m) {
        ++received_;
        last_ = m.data;
      });
  }

  // Publish `count` messages and wait until they are queued in the subscription.
  void publish(int count)
  {
    for (int i = 1; i <= count; ++i) {
      std_msgs::msg::Int32 m;
      m.data = i;
      pub_->publish(m);
    }
    std::this_thread::sleep_for(300ms);
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  int received_{0};
  int last_{0};
};

TEST_F(SpinOnce, DrainsTheWholeBacklogInOneCall)
{
  publish(100);
  coro_common::spinOnce(node_, 500ms);
  EXPECT_EQ(received_, 100);
  EXPECT_EQ(last_, 100) << "the loop must see the newest message";
}

TEST_F(SpinOnce, ReturnsPromptlyWhenNothingIsPending)
{
  const auto start = std::chrono::steady_clock::now();
  coro_common::spinOnce(node_);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);
  EXPECT_EQ(received_, 0);
}

TEST_F(SpinOnce, SpinSomeWouldLeaveABacklog)
{
  // Documents why spinOnce exists: spin_some alone does not catch up with the publisher.
  publish(100);
  rclcpp::spin_some(node_);
  EXPECT_LT(received_, 100);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
