// Turns the joint states of the slider window (joint_state_publisher_gui) into arm commands:
//   sensor_msgs/JointState on <input_topic>  ->  std_msgs/Float64MultiArray on <output_topic>
// See slider_mapping.hpp for the rules (baseline message, limits, matching by name).
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lynxmotion_al5d_description/slider_mapping.hpp"

namespace lynxmotion_al5d
{

class SliderCommand : public rclcpp::Node
{
public:
  SliderCommand()
  : rclcpp::Node("slider_command")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/arm_sliders/joint_states");
    const auto output = declare_parameter<std::string>(
      "output_topic", "/lynxmotion_al5d/joints_positions/commands");
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(output, 10);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      input, 10, [this](const sensor_msgs::msg::JointState & msg) {
        const auto command = mapper_.update(msg.name, msg.position);
        if (command) {
          std_msgs::msg::Float64MultiArray out;
          out.data = *command;
          pub_->publish(out);
        }
      });
  }

private:
  SliderMapper mapper_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace lynxmotion_al5d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lynxmotion_al5d::SliderCommand>());
  rclcpp::shutdown();
  return 0;
}
