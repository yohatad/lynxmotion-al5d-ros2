// Inverse kinematics, servo mapping and configuration parsing of the Lynxmotion AL5D utilities.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <module4/lynxmotionUtilities.h>

// The utilities call these hooks, which the applications normally provide.
void wait(int) {}
void prompt_and_exit(int status)
{
  ADD_FAILURE() << "prompt_and_exit(" << status << ") called";
  std::exit(status);
}

extern robotConfigurationDataType robotConfigurationData;

namespace
{

constexpr double kShoulderHeight = D1;  // mm
constexpr double kHumerus = A3;
constexpr double kUlna = A4;

struct Wrist
{
  double x, y, z;
};

// Forward kinematics of joints 1-3 (base, shoulder, elbow): position of the wrist in mm.
Wrist forward(const double q[5])
{
  const double reach = kHumerus * std::cos(q[1]) + kUlna * std::cos(q[1] + q[2]);
  return {reach * std::sin(q[0]), reach * std::cos(q[0]),
    kShoulderHeight + kHumerus * std::sin(q[1]) + kUlna * std::sin(q[1] + q[2])};
}

std::string dataFile(const std::string & name)
{
  return std::string(MODULE4_DATA_DIR) + "/" + name;
}

}  // namespace

TEST(Kinematics, DegreesAndRadiansAreInverse)
{
  EXPECT_NEAR(degrees(M_PI), 180.0, 1e-9);
  EXPECT_NEAR(radians(90.0), M_PI / 2, 1e-12);
  for (double d = -360; d <= 360; d += 15) {
    EXPECT_NEAR(degrees(radians(d)), d, 1e-9);
  }
}

TEST(Kinematics, InverseThenForwardReturnsTheRequestedWristPosition)
{
  int checked = 0;
  for (double x = -120; x <= 120; x += 40) {
    for (double y = 100; y <= 320; y += 55) {
      for (double z = 20; z <= 360; z += 70) {
        double q[5] = {};
        if (!computeJointAngles(x, y, z, 180.0, 0.0, q)) {
          continue;  // unreachable: covered by the next test
        }
        const Wrist w = forward(q);
        EXPECT_NEAR(w.x, x, 1e-2) << "pose " << x << "," << y << "," << z;
        EXPECT_NEAR(w.y, y, 1e-2) << "pose " << x << "," << y << "," << z;
        EXPECT_NEAR(w.z, z, 1e-2) << "pose " << x << "," << y << "," << z;
        ++checked;
      }
    }
  }
  EXPECT_GT(checked, 40) << "too few reachable poses were exercised";
}

TEST(Kinematics, UnreachablePosesAreRejected)
{
  double q[5] = {};
  EXPECT_FALSE(computeJointAngles(0, 600, 0, 180, 0, q));      // beyond the arm's reach
  EXPECT_FALSE(computeJointAngles(400, 400, 300, 180, 0, q));  // beyond the arm's reach
  EXPECT_FALSE(computeJointAngles(0, 0, D1, 180, 0, q));       // wrist at the shoulder axis
  EXPECT_FALSE(computeJointAngles(0, 1, D1 + 1, 180, 0, q));   // folded inside the minimum reach
}

TEST(Kinematics, BaseAngleFollowsTheDirectionOfTheTarget)
{
  double q[5] = {};
  ASSERT_TRUE(computeJointAngles(0, 200, 150, 180, 0, q));
  EXPECT_NEAR(q[0], 0.0, 1e-9);
  ASSERT_TRUE(computeJointAngles(100, 100, 150, 180, 0, q));
  EXPECT_NEAR(q[0], M_PI / 4, 1e-6);
  ASSERT_TRUE(computeJointAngles(-100, 100, 150, 180, 0, q));
  EXPECT_NEAR(q[0], -M_PI / 4, 1e-6);
}

TEST(Kinematics, ElbowIsBentTheSameWayForEveryReachablePose)
{
  for (double z = 40; z <= 340; z += 60) {
    double q[5] = {};
    if (computeJointAngles(0, 200, z, 180, 0, q)) {
      EXPECT_LT(q[2], 0.0) << "elbow joint must stay negative (elbow-up solution), z=" << z;
    }
  }
}

TEST(Kinematics, WristRollCompensatesBaseRotationWhenPointingDown)
{
  double a[5] = {}, b[5] = {};
  ASSERT_TRUE(computeJointAngles(0, 200, 100, 180, 0, a));
  ASSERT_TRUE(computeJointAngles(100, 173.2, 100, 180, 0, b));  // base rotated by 30 degrees
  // pointing down: roll = requested_roll - base + 90 degrees
  EXPECT_NEAR(degrees(a[4]), 90.0, 1e-6);
  EXPECT_NEAR(degrees(b[4]), 90.0 - degrees(b[0]), 1e-4);
}

TEST(WorkingEnvelope, InsideAndOutside)
{
  EXPECT_EQ(pose_within_working_env(0, 200, 100), 1);
  EXPECT_EQ(pose_within_working_env(MAX_X, MAX_Y, MAX_Z), 1);
  EXPECT_EQ(pose_within_working_env(MAX_X + 5, 200, 100), 0);
  EXPECT_EQ(pose_within_working_env(0, MIN_Y - 5, 100), 0);
  EXPECT_EQ(pose_within_working_env(0, 200, MAX_Z + 5), 0);
  EXPECT_EQ(pose_within_working_env(MIN_X - 5, 200, 100), 0);
}

class RobotConfiguration : public ::testing::TestWithParam<int> {};

TEST_P(RobotConfiguration, ShippedFileParses)
{
  const std::string path = dataFile("robot_" + std::to_string(GetParam()) + "_config.txt");
  char filename[MAX_FILENAME_LENGTH];
  std::strncpy(filename, path.c_str(), sizeof(filename) - 1);
  filename[sizeof(filename) - 1] = '\0';

  std::memset(&robotConfigurationData, 0, sizeof(robotConfigurationData));
  readRobotConfigurationData(filename);

  EXPECT_STREQ(robotConfigurationData.com, "/dev/ttyUSB0");
  EXPECT_EQ(robotConfigurationData.baud, 9600);
  EXPECT_GT(robotConfigurationData.speed, 0);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(robotConfigurationData.channel[i] >= 0, true);
    EXPECT_GT(robotConfigurationData.home[i], 0) << "servo " << i;
    EXPECT_LE(robotConfigurationData.home[i], MAX_PW) << "servo " << i;
    if (i < 5) {  // the gripper servo (index 5) uses a lower pulse-width range
      EXPECT_GE(robotConfigurationData.home[i], MIN_PW) << "servo " << i;
    }
    EXPECT_GT(robotConfigurationData.degree[i], 0.0f) << "servo " << i;
  }
  EXPECT_FALSE(robotConfigurationData.simulator) << "shipped configurations drive the real robot";
}

INSTANTIATE_TEST_SUITE_P(AllRobots, RobotConfiguration, ::testing::Values(1, 2, 3, 4, 5));

TEST(ServoMapping, HomePoseMapsToTheHomeServoPositions)
{
  const std::string path = dataFile("robot_1_config.txt");
  char filename[MAX_FILENAME_LENGTH];
  std::strncpy(filename, path.c_str(), sizeof(filename) - 1);
  filename[sizeof(filename) - 1] = '\0';
  readRobotConfigurationData(filename);

  const double home_pose[5] = {0.0, 1.57, -1.57, 0.0, 0.0};  // CURRENT line of the config
  int positions[6] = {};
  ASSERT_TRUE(computeServoPositions(const_cast<double *>(home_pose), positions));
  for (int i = 0; i < 5; ++i) {
    // the home offset is truncated to whole degrees: allow one degree of pulse width
    EXPECT_NEAR(positions[i], robotConfigurationData.home[i], robotConfigurationData.degree[i] + 1)
      << "servo " << i;
  }
}

TEST(ServoMapping, ServoPositionsMoveWithTheJointAngle)
{
  const std::string path = dataFile("robot_1_config.txt");
  char filename[MAX_FILENAME_LENGTH];
  std::strncpy(filename, path.c_str(), sizeof(filename) - 1);
  filename[sizeof(filename) - 1] = '\0';
  readRobotConfigurationData(filename);

  double q[5] = {0.0, 1.57, -1.57, 0.0, 0.0};
  int before[6] = {}, after[6] = {};
  ASSERT_TRUE(computeServoPositions(q, before));
  q[0] += radians(10.0);  // rotate the base by 10 degrees
  ASSERT_TRUE(computeServoPositions(q, after));
  // 10 degrees at degree[0] microseconds per degree
  EXPECT_NEAR(after[0] - before[0], 10.0 * robotConfigurationData.degree[0], 2);
  for (int i = 1; i < 5; ++i) {
    EXPECT_EQ(after[i], before[i]) << "servo " << i << " must not move";
  }
}
