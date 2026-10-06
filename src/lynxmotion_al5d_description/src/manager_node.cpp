#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lynxmotion_al5d_description/manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lynxmotion_al5d::Manager>();
  // Service handlers block on simulator requests, so several threads are required.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
