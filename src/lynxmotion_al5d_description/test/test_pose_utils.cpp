#include <gtest/gtest.h>

#include <cmath>

#include "lynxmotion_al5d_description/pose_utils.hpp"

using lynxmotion_al5d::degrees;
using lynxmotion_al5d::quaternionFromRPY;
using lynxmotion_al5d::radians;
using lynxmotion_al5d::rpyFromQuaternion;

TEST(PoseUtils, DegreesRadiansAreInverse)
{
  EXPECT_NEAR(radians(180.0), M_PI, 1e-12);
  EXPECT_NEAR(degrees(M_PI / 2.0), 90.0, 1e-12);
  for (double d = -720.0; d <= 720.0; d += 37.0) {
    EXPECT_NEAR(degrees(radians(d)), d, 1e-9);
  }
}

TEST(PoseUtils, ZeroRotationIsIdentityQuaternion)
{
  const auto q = quaternionFromRPY(0, 0, 0);
  EXPECT_DOUBLE_EQ(q.x, 0.0);
  EXPECT_DOUBLE_EQ(q.y, 0.0);
  EXPECT_DOUBLE_EQ(q.z, 0.0);
  EXPECT_DOUBLE_EQ(q.w, 1.0);
}

TEST(PoseUtils, NinetyDegreeYaw)
{
  const auto q = quaternionFromRPY(0, 0, M_PI / 2.0);
  EXPECT_NEAR(q.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(q.w, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(q.x, 0.0, 1e-12);
  EXPECT_NEAR(q.y, 0.0, 1e-12);
}

TEST(PoseUtils, QuaternionIsNormalised)
{
  const auto q = quaternionFromRPY(0.3, -0.7, 2.1);
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12);
}

TEST(PoseUtils, RoundTripRollPitchYaw)
{
  // Stay clear of gimbal lock (|pitch| < pi/2) and of the +-pi yaw wrap.
  for (double roll = -3.0; roll <= 3.0; roll += 0.75) {
    for (double pitch = -1.4; pitch <= 1.4; pitch += 0.7) {
      for (double yaw = -3.0; yaw <= 3.0; yaw += 0.75) {
        const auto rpy = rpyFromQuaternion(quaternionFromRPY(roll, pitch, yaw));
        EXPECT_NEAR(rpy.roll, roll, 1e-9);
        EXPECT_NEAR(rpy.pitch, pitch, 1e-9);
        EXPECT_NEAR(rpy.yaw, yaw, 1e-9);
      }
    }
  }
}

TEST(PoseUtils, UnnormalisedQuaternionIsAccepted)
{
  auto q = quaternionFromRPY(0.1, 0.2, 0.3);
  q.x *= 5;
  q.y *= 5;
  q.z *= 5;
  q.w *= 5;
  const auto rpy = rpyFromQuaternion(q);
  EXPECT_NEAR(rpy.roll, 0.1, 1e-9);
  EXPECT_NEAR(rpy.pitch, 0.2, 1e-9);
  EXPECT_NEAR(rpy.yaw, 0.3, 1e-9);
}
