#pragma once

#include <algorithm>
#include <array>
#include <vector>

namespace lynxmotion_al5d
{

/// Geometry of the AL5D gripper, as in urdf/lynxmotion_al5d_properties.urdf.xacro.
struct GripperGeometry
{
  double base_width{0.03175};   // opening between the fingers when fully open (m)
  double left_offset{-0.0025};  // left finger = right finger + offset (as the URDF mimic tag)
};

struct Fingers
{
  double right{0.0};
  double left{0.0};
};

/// The URDF mimic relations, evaluated here because Gazebo has no mimic support:
///   right_finger = width/2 - 0.5 * Gripper        (multiplier -0.5, offset width/2)
///   left_finger  = right_finger + left_offset
/// Both fingers are limited to [0, width/2], like their URDF joint limits.
inline Fingers fingersFromGripper(double gripper, const GripperGeometry & g = GripperGeometry())
{
  const double half = g.base_width / 2.0;
  const double opening = std::clamp(gripper, 0.0, g.base_width);
  Fingers f;
  f.right = std::clamp(half - 0.5 * opening, 0.0, half);
  f.left = std::clamp(f.right + g.left_offset, 0.0, half);
  return f;
}

/// Expand [Joint1..Joint5, Gripper] into the eight actuated joints
/// [Joint1..Joint5, Gripper, right_finger_joint, left_finger_joint].
/// The gripper opening is clamped to its physical range. Returns an empty vector when
/// `command` does not have exactly six finite values.
inline std::vector<double> expandArmCommand(
  const std::vector<double> & command, const GripperGeometry & g = GripperGeometry())
{
  if (command.size() != 6) {
    return {};
  }
  for (double v : command) {
    if (!(v == v) || v > 1e12 || v < -1e12) {  // NaN or absurd
      return {};
    }
  }
  std::vector<double> out(command.begin(), command.end());
  out[5] = std::clamp(out[5], 0.0, g.base_width);
  const Fingers f = fingersFromGripper(out[5], g);
  out.push_back(f.right);
  out.push_back(f.left);
  return out;
}

}  // namespace lynxmotion_al5d
