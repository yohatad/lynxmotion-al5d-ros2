// The shipped locomotion parameter file must parse into the documented values.
#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include <module3/goToPoseCreate.h>

TEST(LocomotionParameters, ShippedFileParses)
{
  char filename[MAX_FILENAME_LENGTH];
  const std::string path = std::string(MODULE3_DATA_DIR) + "/parameters.txt";
  ASSERT_LT(path.size(), static_cast<size_t>(MAX_FILENAME_LENGTH));
  std::strcpy(filename, path.c_str());

  locomotionParameterDataType data{};
  readLocomotionParameterData(filename, &data);

  EXPECT_FLOAT_EQ(data.position_tolerance, 0.01f);
  EXPECT_FLOAT_EQ(data.angle_tolerance_orienting, 0.075f);
  EXPECT_FLOAT_EQ(data.angle_tolerance_going, 0.075f);
  EXPECT_FLOAT_EQ(data.position_gain_dq, 0.3f);
  EXPECT_FLOAT_EQ(data.angle_gain_dq, 0.3f);
  EXPECT_FLOAT_EQ(data.position_gain_mimo, 0.2f);
  EXPECT_FLOAT_EQ(data.angle_gain_mimo, 0.5f);
  EXPECT_FLOAT_EQ(data.min_linear_velocity, 0.015f);
  EXPECT_FLOAT_EQ(data.max_linear_velocity, 0.2f);
  EXPECT_FLOAT_EQ(data.min_angular_velocity, 0.09f);
  EXPECT_FLOAT_EQ(data.max_angular_velocity, 1.0f);
}

TEST(LocomotionParameters, VelocityLimitsAreConsistent)
{
  char filename[MAX_FILENAME_LENGTH];
  const std::string path = std::string(MODULE3_DATA_DIR) + "/parameters.txt";
  std::strcpy(filename, path.c_str());
  locomotionParameterDataType data{};
  readLocomotionParameterData(filename, &data);

  EXPECT_GT(data.min_linear_velocity, 0.0f);
  EXPECT_LT(data.min_linear_velocity, data.max_linear_velocity);
  EXPECT_GT(data.min_angular_velocity, 0.0f);
  EXPECT_LT(data.min_angular_velocity, data.max_angular_velocity);
}

// goToPoseCreate.h declares ros_node; the tests never use ROS but must satisfy the linker
// only through the implementation object, which defines it.
TEST(LocomotionParameters, NodeIsNotCreatedUntilMain)
{
  EXPECT_EQ(ros_node, nullptr);
}
