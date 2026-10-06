#pragma once

#include <string>

#include "geometry_msgs/msg/pose.hpp"

namespace lynxmotion_al5d
{

/// Outcome of a simulator request.
struct SimResult
{
  bool ok{false};
  std::string message;
};

/// Minimal simulator interface used by the brick manager. The production implementation
/// talks to Gazebo (gz sim) through ros_gz_bridge; tests can substitute a fake.
class SimBackend
{
public:
  virtual ~SimBackend() = default;
  virtual SimResult spawn(
    const std::string & name, const std::string & sdf,
    const geometry_msgs::msg::Pose & pose) = 0;
  virtual SimResult remove(const std::string & name) = 0;
  virtual SimResult setPose(const std::string & name, const geometry_msgs::msg::Pose & pose) = 0;
};

}  // namespace lynxmotion_al5d
