// Accepts the six-value arm command used by the course code
//   std_msgs/Float64MultiArray [Joint1 .. Joint5, Gripper] on <input_topic>
// and forwards the eight-value command that the simulated ros2_control arm controller needs
//   [Joint1 .. Joint5, Gripper, right_finger_joint, left_finger_joint] on <output_topic>.
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lynxmotion_al5d_description/gripper_mapping.hpp"

namespace lynxmotion_al5d
{

class CommandRelay : public rclcpp::Node
{
public:
  CommandRelay()
  : rclcpp::Node("command_relay")
  {
    const auto input = declare_parameter<std::string>(
      "input_topic", "/lynxmotion_al5d/joints_positions/commands");
    const auto output = declare_parameter<std::string>(
      "output_topic", "/lynxmotion_al5d/arm_controller/commands");
    geometry_.base_width = declare_parameter<double>("gripper_base_width", geometry_.base_width);
    geometry_.left_offset = declare_parameter<double>("left_finger_offset", geometry_.left_offset);

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(output, 10);
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      input, 10, [this](const std_msgs::msg::Float64MultiArray & msg) {
        const auto expanded = expandArmCommand(msg.data, geometry_);
        if (expanded.empty()) {
          RCLCPP_WARN(
            get_logger(), "ignoring command with %zu values: expected six finite values",
            msg.data.size());
          return;
        }
        std_msgs::msg::Float64MultiArray out;
        out.data = expanded;
        pub_->publish(out);
      });
  }

private:
  GripperGeometry geometry_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

}  // namespace lynxmotion_al5d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lynxmotion_al5d::CommandRelay>());
  rclcpp::shutdown();
  return 0;
}
