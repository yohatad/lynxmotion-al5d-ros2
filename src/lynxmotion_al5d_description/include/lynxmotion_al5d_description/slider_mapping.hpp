#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace lynxmotion_al5d
{

/// Names and limits of the six commanded joints, in command order (as in the URDF).
struct JointRange
{
  const char * name;
  double low;
  double high;
};

inline const std::array<JointRange, 6> & commandedJoints()
{
  static const std::array<JointRange, 6> joints = {{
    {"Joint1", -3.14159, 3.14159},
    {"Joint2", 0.0, 3.14159},
    {"Joint3", -3.14159, 0.0},
    {"Joint4", -1.5708, 1.5708},
    {"Joint5", -3.14159, 3.14159},
    {"Gripper", 0.0, 0.03175},
  }};
  return joints;
}

/// Turns the joint states of a slider window into arm commands.
///
/// The slider window publishes its starting values at once, before anyone touches a slider, and
/// publishes them again every tenth of a second. Sending those would throw the arm to the slider
/// window's zero pose. The first valid message is therefore only a baseline, and a command is
/// produced only when a joint differs from the previous message. Joints are matched by name,
/// values are limited to the joint ranges, and messages that are incomplete or not finite are
/// ignored.
class SliderMapper
{
public:
  /// @return the six-value command if the sliders changed, std::nullopt otherwise
  std::optional<std::vector<double>> update(
    const std::vector<std::string> & names, const std::vector<double> & positions)
  {
    if (names.size() != positions.size()) {
      return std::nullopt;
    }
    std::vector<double> command;
    for (const auto & joint : commandedJoints()) {
      const auto it = std::find(names.begin(), names.end(), joint.name);
      if (it == names.end()) {
        return std::nullopt;
      }
      const double value = positions[static_cast<std::size_t>(it - names.begin())];
      if (!std::isfinite(value)) {
        return std::nullopt;
      }
      command.push_back(std::clamp(value, joint.low, joint.high));
    }

    const bool first = !previous_.has_value();
    bool changed = false;
    if (!first) {
      for (std::size_t i = 0; i < command.size(); ++i) {
        if (std::fabs(command[i] - (*previous_)[i]) > 1e-6) {
          changed = true;
        }
      }
    }
    previous_ = command;
    if (first || !changed) {
      return std::nullopt;
    }
    return command;
  }

private:
  std::optional<std::vector<double>> previous_;
};

}  // namespace lynxmotion_al5d
