// Republishes Gazebo's dynamic pose stream as tf2_msgs/TFMessage with proper frame names, for
// the brick manager (see gz_pose_conversion.hpp for why ros_gz_bridge is not used here).
#include <memory>
#include <string>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "lynxmotion_al5d_description/gz_pose_conversion.hpp"

namespace lynxmotion_al5d
{

class GzPoseRelay : public rclcpp::Node
{
public:
  GzPoseRelay()
  : rclcpp::Node("gz_pose_relay")
  {
    const auto world = declare_parameter<std::string>("world_name", "al5d");
    const std::string topic = "/world/" + world + "/dynamic_pose/info";
    pub_ = create_publisher<tf2_msgs::msg::TFMessage>(topic, rclcpp::SensorDataQoS());
    if (!gz_node_.Subscribe(topic, &GzPoseRelay::onGzPoses, this)) {
      RCLCPP_ERROR(get_logger(), "cannot subscribe to the Gazebo topic %s", topic.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "relaying Gazebo %s to ROS", topic.c_str());
    }
  }

private:
  void onGzPoses(const gz::msgs::Pose_V & poses)
  {
    pub_->publish(poseVectorToTf(poses));
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
};

}  // namespace lynxmotion_al5d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lynxmotion_al5d::GzPoseRelay>());
  rclcpp::shutdown();
  return 0;
}
