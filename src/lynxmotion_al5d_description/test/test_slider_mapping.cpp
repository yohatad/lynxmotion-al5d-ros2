#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "lynxmotion_al5d_description/slider_mapping.hpp"

using lynxmotion_al5d::SliderMapper;

namespace
{
const std::vector<std::string> kNames = {
  "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Gripper"};
}

TEST(SliderMapper, FirstMessageIsOnlyABaseline)
{
  SliderMapper mapper;
  EXPECT_FALSE(mapper.update(kNames, {0, 0, 0, 0, 0, 0}).has_value());
}

TEST(SliderMapper, RepeatedIdenticalMessagesProduceNothing)
{
  SliderMapper mapper;
  mapper.update(kNames, {0, 0, 0, 0, 0, 0});
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(mapper.update(kNames, {0, 0, 0, 0, 0, 0}).has_value());
  }
}

TEST(SliderMapper, ChangingASliderProducesTheCommand)
{
  SliderMapper mapper;
  mapper.update(kNames, {0, 0, 0, 0, 0, 0});
  const auto command = mapper.update(kNames, {0.5, 0, 0, 0, 0, 0});
  ASSERT_TRUE(command.has_value());
  ASSERT_EQ(command->size(), 6u);
  EXPECT_DOUBLE_EQ((*command)[0], 0.5);
  // the same message again is no new change
  EXPECT_FALSE(mapper.update(kNames, {0.5, 0, 0, 0, 0, 0}).has_value());
}

TEST(SliderMapper, ReturningToTheStartValueIsAChangeToo)
{
  SliderMapper mapper;
  mapper.update(kNames, {0, 0, 0, 0, 0, 0});
  mapper.update(kNames, {0.5, 0, 0, 0, 0, 0});
  const auto back = mapper.update(kNames, {0, 0, 0, 0, 0, 0});
  ASSERT_TRUE(back.has_value());
  EXPECT_DOUBLE_EQ((*back)[0], 0.0);
}

TEST(SliderMapper, JointsAreMatchedByNameNotByPosition)
{
  SliderMapper mapper;
  const std::vector<std::string> shuffled = {
    "Gripper", "Joint5", "Joint4", "Joint3", "Joint2", "Joint1"};
  mapper.update(shuffled, {0, 0, 0, 0, 0, 0});
  const auto command = mapper.update(shuffled, {0.02, 0.5, 0.4, -0.3, 1.2, 0.1});
  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ((*command)[0], 0.1);    // Joint1
  EXPECT_DOUBLE_EQ((*command)[1], 1.2);    // Joint2
  EXPECT_DOUBLE_EQ((*command)[2], -0.3);   // Joint3
  EXPECT_DOUBLE_EQ((*command)[3], 0.4);    // Joint4
  EXPECT_DOUBLE_EQ((*command)[4], 0.5);    // Joint5
  EXPECT_DOUBLE_EQ((*command)[5], 0.02);   // Gripper
}

TEST(SliderMapper, ExtraJointsAreIgnored)
{
  SliderMapper mapper;
  std::vector<std::string> names = kNames;
  names.push_back("right_finger_joint");
  mapper.update(names, {0, 0, 0, 0, 0, 0, 0});
  const auto command = mapper.update(names, {0, 1.0, 0, 0, 0, 0, 0.01});
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->size(), 6u);
}

TEST(SliderMapper, ValuesAreLimitedToTheJointRanges)
{
  SliderMapper mapper;
  mapper.update(kNames, {0, 1, -1, 0, 0, 0.01});
  const auto command = mapper.update(kNames, {10, -5, 5, 3, -10, 1.0});
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR((*command)[0], 3.14159, 1e-6);     // Joint1 upper limit
  EXPECT_DOUBLE_EQ((*command)[1], 0.0);          // Joint2 lower limit
  EXPECT_DOUBLE_EQ((*command)[2], 0.0);          // Joint3 upper limit
  EXPECT_NEAR((*command)[3], 1.5708, 1e-6);      // Joint4 upper limit
  EXPECT_NEAR((*command)[4], -3.14159, 1e-6);    // Joint5 lower limit
  EXPECT_NEAR((*command)[5], 0.03175, 1e-9);     // Gripper upper limit
}

TEST(SliderMapper, IncompleteMessagesAreIgnoredAndDoNotResetTheBaseline)
{
  SliderMapper mapper;
  mapper.update(kNames, {0, 0, 0, 0, 0, 0});
  EXPECT_FALSE(mapper.update({"Joint1", "Joint2"}, {1.0, 1.0}).has_value());
  EXPECT_FALSE(mapper.update({}, {}).has_value());
  EXPECT_FALSE(mapper.update(kNames, {0, 0, 0}).has_value());  // sizes differ
  EXPECT_FALSE(mapper.update(kNames, {0, 0, 0, 0, 0, 0}).has_value());
  EXPECT_TRUE(mapper.update(kNames, {0.1, 0, 0, 0, 0, 0}).has_value());
}

TEST(SliderMapper, NonFiniteValuesAreIgnored)
{
  SliderMapper mapper;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  mapper.update(kNames, {0, 0, 0, 0, 0, 0});
  EXPECT_FALSE(mapper.update(kNames, {nan, 0, 0, 0, 0, 0}).has_value());
  EXPECT_FALSE(mapper.update(kNames, {0, inf, 0, 0, 0, 0}).has_value());
  EXPECT_FALSE(mapper.update(kNames, {0, 0, 0, 0, 0, 0}).has_value());
}
