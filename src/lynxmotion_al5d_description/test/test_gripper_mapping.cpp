#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "lynxmotion_al5d_description/gripper_mapping.hpp"

using lynxmotion_al5d::expandArmCommand;
using lynxmotion_al5d::fingersFromGripper;
using lynxmotion_al5d::GripperGeometry;

TEST(GripperMapping, FullyOpenGripper)
{
  const GripperGeometry g;
  const auto f = fingersFromGripper(g.base_width, g);
  EXPECT_DOUBLE_EQ(f.right, 0.0);
  EXPECT_DOUBLE_EQ(f.left, 0.0);  // -0.0025 offset is clamped to the joint limit
}

TEST(GripperMapping, ClosedGripper)
{
  const GripperGeometry g;
  const auto f = fingersFromGripper(0.0, g);
  EXPECT_NEAR(f.right, g.base_width / 2.0, 1e-12);
  EXPECT_NEAR(f.left, g.base_width / 2.0 - 0.0025, 1e-12);
}

TEST(GripperMapping, MatchesUrdfMimicRelationInTheMiddle)
{
  const GripperGeometry g;
  const double gripper = 0.01;
  const auto f = fingersFromGripper(gripper, g);
  EXPECT_NEAR(f.right, g.base_width / 2.0 - 0.5 * gripper, 1e-12);  // multiplier -0.5, offset w/2
  EXPECT_NEAR(f.left, f.right - 0.0025, 1e-12);                     // mimic right, offset -0.0025
}

TEST(GripperMapping, FingersStayWithinJointLimitsForAnyInput)
{
  const GripperGeometry g;
  for (double gripper = -0.1; gripper <= 0.1; gripper += 0.0007) {
    const auto f = fingersFromGripper(gripper, g);
    EXPECT_GE(f.right, 0.0);
    EXPECT_LE(f.right, g.base_width / 2.0 + 1e-12);
    EXPECT_GE(f.left, 0.0);
    EXPECT_LE(f.left, g.base_width / 2.0 + 1e-12);
  }
}

TEST(GripperMapping, ClosingTheGripperNeverOpensTheFingers)
{
  const GripperGeometry g;
  double previous_right = -1.0;
  for (double gripper = g.base_width; gripper >= 0.0; gripper -= 0.001) {
    const auto f = fingersFromGripper(gripper, g);
    EXPECT_GE(f.right, previous_right - 1e-12);
    previous_right = f.right;
  }
}

TEST(ExpandArmCommand, SixValuesBecomeEight)
{
  const auto out = expandArmCommand({0.1, 1.5, -1.5, 0.2, 0.3, 0.01});
  ASSERT_EQ(out.size(), 8u);
  EXPECT_DOUBLE_EQ(out[0], 0.1);
  EXPECT_DOUBLE_EQ(out[4], 0.3);
  EXPECT_DOUBLE_EQ(out[5], 0.01);
  EXPECT_NEAR(out[6], 0.03175 / 2.0 - 0.005, 1e-12);
  EXPECT_NEAR(out[7], out[6] - 0.0025, 1e-12);
}

TEST(ExpandArmCommand, GripperIsClamped)
{
  EXPECT_DOUBLE_EQ(expandArmCommand({0, 0, 0, 0, 0, 1.0})[5], 0.03175);
  EXPECT_DOUBLE_EQ(expandArmCommand({0, 0, 0, 0, 0, -1.0})[5], 0.0);
}

TEST(ExpandArmCommand, RejectsWrongSizes)
{
  EXPECT_TRUE(expandArmCommand({}).empty());
  EXPECT_TRUE(expandArmCommand({1, 2, 3, 4, 5}).empty());
  EXPECT_TRUE(expandArmCommand({1, 2, 3, 4, 5, 6, 7}).empty());
}

TEST(ExpandArmCommand, RejectsNonFiniteValues)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(expandArmCommand({0, 0, nan, 0, 0, 0}).empty());
  EXPECT_TRUE(expandArmCommand({0, inf, 0, 0, 0, 0}).empty());
  EXPECT_TRUE(expandArmCommand({0, 0, 0, 0, -inf, 0}).empty());
}
