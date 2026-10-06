// Brick manager logic against a fake simulator (no Gazebo, no bridge).
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "lynxmotion_al5d_description/manager.hpp"
#include "lynxmotion_al5d_description/pose_utils.hpp"
#include "lynxmotion_al5d_description/srv/kill_brick.hpp"
#include "lynxmotion_al5d_description/srv/spawn_brick.hpp"
#include "lynxmotion_al5d_description/srv/teleport_absolute.hpp"
#include "lynxmotion_al5d_description/srv/teleport_relative.hpp"

using namespace std::chrono_literals;
namespace al = lynxmotion_al5d;
namespace ifc = lynxmotion_al5d_description;

namespace
{

class FakeBackend : public al::SimBackend
{
public:
  struct SpawnCall
  {
    std::string name;
    std::string sdf;
    geometry_msgs::msg::Pose pose;
  };

  al::SimResult spawn(
    const std::string & name, const std::string & sdf,
    const geometry_msgs::msg::Pose & pose) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    spawns.push_back({name, sdf, pose});
    return fail_spawn ? al::SimResult{false, "fake spawn failure"} : al::SimResult{true, ""};
  }
  al::SimResult remove(const std::string & name) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    removed.push_back(name);
    return fail_remove ? al::SimResult{false, "fake remove failure"} : al::SimResult{true, ""};
  }
  al::SimResult setPose(const std::string & name, const geometry_msgs::msg::Pose & pose) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    poses.emplace_back(name, pose);
    return {true, ""};
  }

  std::mutex mutex;
  std::atomic<bool> fail_spawn{false};
  std::atomic<bool> fail_remove{false};
  std::vector<SpawnCall> spawns;
  std::vector<std::string> removed;
  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> poses;
};

class ManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto backend = std::make_unique<FakeBackend>();
    fake_ = backend.get();
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"service_timeout", 1.0}});
    manager_ = std::make_shared<al::Manager>(options, std::move(backend));
    client_node_ = std::make_shared<rclcpp::Node>("manager_test_client");

    manager_exec_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    manager_exec_->add_node(manager_);
    manager_thread_ = std::thread([this]() {manager_exec_->spin();});

    client_exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    client_exec_->add_node(client_node_);
    client_thread_ = std::thread([this]() {client_exec_->spin();});

    spawn_client_ = client_node_->create_client<ifc::srv::SpawnBrick>("/lynxmotion_al5d/spawn_brick");
    kill_client_ = client_node_->create_client<ifc::srv::KillBrick>("/lynxmotion_al5d/kill_brick");
    clear_client_ = client_node_->create_client<std_srvs::srv::Empty>("/lynxmotion_al5d/clear");
    reset_client_ = client_node_->create_client<std_srvs::srv::Empty>("/lynxmotion_al5d/reset");
    ASSERT_TRUE(spawn_client_->wait_for_service(5s));
    ASSERT_TRUE(kill_client_->wait_for_service(5s));
    ASSERT_TRUE(clear_client_->wait_for_service(5s));
    ASSERT_TRUE(reset_client_->wait_for_service(5s));
  }

  void TearDown() override
  {
    manager_exec_->cancel();
    client_exec_->cancel();
    manager_thread_.join();
    client_thread_.join();
  }

  template<typename ClientT, typename Req>
  auto call(ClientT & client, const Req & request)
  {
    auto future = client->async_send_request(request);
    EXPECT_EQ(future.wait_for(10s), std::future_status::ready);
    return future.get();
  }

  std::shared_ptr<ifc::srv::SpawnBrick::Response> spawn(
    const std::string & color, const std::string & name = "", double x = 0.0)
  {
    auto request = std::make_shared<ifc::srv::SpawnBrick::Request>();
    request->color = color;
    request->name = name;
    request->pose.position.x = x;
    return call(spawn_client_, request);
  }

  std::shared_ptr<ifc::srv::KillBrick::Response> kill(const std::string & name)
  {
    auto request = std::make_shared<ifc::srv::KillBrick::Request>();
    request->name = name;
    return call(kill_client_, request);
  }

  FakeBackend * fake_{nullptr};
  std::shared_ptr<al::Manager> manager_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> manager_exec_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> client_exec_;
  std::thread manager_thread_;
  std::thread client_thread_;
  rclcpp::Client<ifc::srv::SpawnBrick>::SharedPtr spawn_client_;
  rclcpp::Client<ifc::srv::KillBrick>::SharedPtr kill_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
};

}  // namespace

TEST(ManagerValidation, Colours)
{
  EXPECT_TRUE(al::Manager::isValidColor("red"));
  EXPECT_TRUE(al::Manager::isValidColor("green"));
  EXPECT_TRUE(al::Manager::isValidColor("blue"));
  EXPECT_FALSE(al::Manager::isValidColor("Red"));
  EXPECT_FALSE(al::Manager::isValidColor(""));
  EXPECT_FALSE(al::Manager::isValidColor("purple"));
}

TEST(ManagerValidation, Names)
{
  EXPECT_TRUE(al::Manager::isValidName("brick1"));
  EXPECT_TRUE(al::Manager::isValidName("my_brick"));
  EXPECT_FALSE(al::Manager::isValidName(""));
  EXPECT_FALSE(al::Manager::isValidName("1brick"));
  EXPECT_FALSE(al::Manager::isValidName("a b"));
  EXPECT_FALSE(al::Manager::isValidName("a/b"));
  EXPECT_FALSE(al::Manager::isValidName("a-b"));
}

TEST_F(ManagerTest, SpawnAutoNamesAndSendsSdf)
{
  const auto r1 = spawn("red", "", 0.1);
  ASSERT_TRUE(r1->success) << r1->message;
  EXPECT_EQ(r1->name, "brick1");
  const auto r2 = spawn("blue");
  ASSERT_TRUE(r2->success);
  EXPECT_EQ(r2->name, "brick2");
  EXPECT_EQ(manager_->brickCount(), 2u);

  std::lock_guard<std::mutex> lock(fake_->mutex);
  ASSERT_EQ(fake_->spawns.size(), 2u);
  EXPECT_EQ(fake_->spawns[0].name, "brick1");
  EXPECT_NEAR(fake_->spawns[0].pose.position.x, 0.1, 1e-9);
  EXPECT_NE(fake_->spawns[0].sdf.find("<model"), std::string::npos);
  EXPECT_NE(fake_->spawns[0].sdf.find("0.9 0.0 0.0"), std::string::npos) << "red brick colour";
  EXPECT_NE(fake_->spawns[1].sdf.find("0.0 0.0 1.0"), std::string::npos) << "blue brick colour";
}

TEST_F(ManagerTest, SpawnWithExplicitName)
{
  const auto r = spawn("green", "my_brick");
  ASSERT_TRUE(r->success);
  EXPECT_EQ(r->name, "my_brick");
}

TEST_F(ManagerTest, SpawnRejectsBadRequests)
{
  EXPECT_FALSE(spawn("purple")->success);
  EXPECT_FALSE(spawn("red", "bad name")->success);
  ASSERT_TRUE(spawn("red", "dup")->success);
  const auto dup = spawn("red", "dup");
  EXPECT_FALSE(dup->success);
  EXPECT_NE(dup->message.find("already exists"), std::string::npos);
  EXPECT_EQ(manager_->brickCount(), 1u);
  std::lock_guard<std::mutex> lock(fake_->mutex);
  EXPECT_EQ(fake_->spawns.size(), 1u) << "rejected requests must not reach the simulator";
}

TEST_F(ManagerTest, FailedSpawnDoesNotConsumeName)
{
  fake_->fail_spawn = true;
  const auto failed = spawn("red");
  EXPECT_FALSE(failed->success);
  EXPECT_EQ(failed->message, "fake spawn failure");
  EXPECT_EQ(manager_->brickCount(), 0u);

  fake_->fail_spawn = false;
  const auto ok = spawn("red");
  ASSERT_TRUE(ok->success);
  EXPECT_EQ(ok->name, "brick1");
}

TEST_F(ManagerTest, KillRemovesBrickAndFreesName)
{
  ASSERT_TRUE(spawn("red")->success);
  const auto killed = kill("brick1");
  EXPECT_TRUE(killed->result) << killed->message;
  EXPECT_EQ(manager_->brickCount(), 0u);
  EXPECT_FALSE(kill("brick1")->result);
  EXPECT_EQ(spawn("red")->name, "brick1");
}

TEST_F(ManagerTest, KillUnknownBrickFails)
{
  const auto r = kill("nope");
  EXPECT_FALSE(r->result);
  EXPECT_NE(r->message.find("doesn't exist"), std::string::npos);
}

TEST_F(ManagerTest, KillKeepsBrickWhenSimulatorRefuses)
{
  ASSERT_TRUE(spawn("red")->success);
  fake_->fail_remove = true;
  EXPECT_FALSE(kill("brick1")->result);
  EXPECT_EQ(manager_->brickCount(), 1u);
}

TEST_F(ManagerTest, ClearRemovesEverythingAndResetsCounter)
{
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(spawn("green")->success);
  }
  call(clear_client_, std::make_shared<std_srvs::srv::Empty::Request>());
  EXPECT_EQ(manager_->brickCount(), 0u);
  {
    std::lock_guard<std::mutex> lock(fake_->mutex);
    EXPECT_EQ(fake_->removed.size(), 3u);
  }
  EXPECT_EQ(spawn("green")->name, "brick1");
}

TEST_F(ManagerTest, ResetSendsArmHome)
{
  std::mutex m;
  std::vector<double> received;
  auto sub = client_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/lynxmotion_al5d/joints_positions/commands", 10,
    [&](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(m);
      received = msg->data;
    });
  ASSERT_TRUE(spawn("red")->success);
  call(reset_client_, std::make_shared<std_srvs::srv::Empty::Request>());
  EXPECT_EQ(manager_->brickCount(), 0u);

  for (int i = 0; i < 100; ++i) {
    {
      std::lock_guard<std::mutex> lock(m);
      if (!received.empty()) {break;}
    }
    std::this_thread::sleep_for(50ms);
  }
  std::lock_guard<std::mutex> lock(m);
  ASSERT_EQ(received.size(), 6u);
  EXPECT_DOUBLE_EQ(received[1], 1.57);
  EXPECT_DOUBLE_EQ(received[2], -1.57);
  EXPECT_DOUBLE_EQ(received[5], 0.0);
}

TEST_F(ManagerTest, TeleportAbsoluteAndRelative)
{
  ASSERT_TRUE(spawn("red", "b", 1.0)->success);
  auto abs_client =
    client_node_->create_client<ifc::srv::TeleportAbsolute>("/lynxmotion_al5d/b/teleport_absolute");
  auto rel_client =
    client_node_->create_client<ifc::srv::TeleportRelative>("/lynxmotion_al5d/b/teleport_relative");
  ASSERT_TRUE(abs_client->wait_for_service(5s));
  ASSERT_TRUE(rel_client->wait_for_service(5s));

  auto abs_req = std::make_shared<ifc::srv::TeleportAbsolute::Request>();
  abs_req->pose.position.x = 0.3;
  abs_req->pose.position.y = 0.4;
  abs_req->pose.orientation.yaw = 1.0f;
  ASSERT_TRUE(call(abs_client, abs_req)->success);

  {
    std::lock_guard<std::mutex> lock(fake_->mutex);
    ASSERT_EQ(fake_->poses.size(), 1u);
    EXPECT_EQ(fake_->poses[0].first, "b");
    EXPECT_NEAR(fake_->poses[0].second.position.x, 0.3, 1e-9);
    EXPECT_NEAR(al::rpyFromQuaternion(fake_->poses[0].second.orientation).yaw, 1.0, 1e-6);
  }

  // Relative moves are applied on top of the brick's current pose (initially x = 1.0).
  auto rel_req = std::make_shared<ifc::srv::TeleportRelative::Request>();
  rel_req->pose.position.x = 0.5;
  rel_req->pose.orientation.yaw = 0.25f;
  ASSERT_TRUE(call(rel_client, rel_req)->success);
  std::lock_guard<std::mutex> lock(fake_->mutex);
  ASSERT_EQ(fake_->poses.size(), 2u);
  EXPECT_NEAR(fake_->poses[1].second.position.x, 1.5, 1e-9);
  EXPECT_NEAR(al::rpyFromQuaternion(fake_->poses[1].second.orientation).yaw, 0.25, 1e-6);
}

TEST_F(ManagerTest, PoseFollowsSimulatorStream)
{
  ASSERT_TRUE(spawn("blue", "tracked", 0.0)->success);

  std::mutex m;
  ifc::msg::Pose latest;
  bool have = false;
  auto sub = client_node_->create_subscription<ifc::msg::Pose>(
    "/lynxmotion_al5d/tracked/pose", 10, [&](ifc::msg::Pose::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(m);
      latest = *msg;
      have = true;
    });
  auto pub = client_node_->create_publisher<tf2_msgs::msg::TFMessage>(
    "/world/al5d/dynamic_pose/info", rclcpp::SensorDataQoS());

  tf2_msgs::msg::TFMessage message;
  geometry_msgs::msg::TransformStamped t;
  t.child_frame_id = "tracked";
  t.transform.translation.x = 0.42;
  t.transform.translation.z = 0.01;
  t.transform.rotation = al::quaternionFromRPY(0, 0, 0.5);
  message.transforms.push_back(t);
  t.child_frame_id = "someone_else";  // unrelated entity must be ignored
  t.transform.translation.x = 9.0;
  message.transforms.push_back(t);

  bool seen = false;
  for (int i = 0; i < 100 && !seen; ++i) {
    pub->publish(message);
    std::this_thread::sleep_for(50ms);
    std::lock_guard<std::mutex> lock(m);
    seen = have && std::abs(latest.position.x - 0.42) < 1e-6;
  }
  ASSERT_TRUE(seen) << "brick pose topic never reflected the simulator pose";
  std::lock_guard<std::mutex> lock(m);
  EXPECT_NEAR(latest.position.z, 0.01, 1e-6);
  EXPECT_NEAR(latest.orientation.yaw, 0.5, 1e-5);
}

TEST_F(ManagerTest, ConcurrentSpawnsGetDistinctNames)
{
  constexpr int kThreads = 8;
  std::vector<rclcpp::Client<ifc::srv::SpawnBrick>::FutureAndRequestId> pending;
  for (int i = 0; i < kThreads; ++i) {
    auto request = std::make_shared<ifc::srv::SpawnBrick::Request>();
    request->color = "green";
    pending.push_back(spawn_client_->async_send_request(request));
  }
  std::set<std::string> names;
  for (auto & p : pending) {
    ASSERT_EQ(p.future.wait_for(15s), std::future_status::ready);
    const auto response = p.future.get();
    ASSERT_TRUE(response->success) << response->message;
    names.insert(response->name);
  }
  EXPECT_EQ(names.size(), static_cast<std::size_t>(kThreads));
  EXPECT_EQ(manager_->brickCount(), static_cast<std::size_t>(kThreads));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
