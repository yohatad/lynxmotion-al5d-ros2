#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "lynxmotion_al5d_description/cli_args.hpp"

using lynxmotion_al5d::ParseStatus;
using lynxmotion_al5d::parseKillArgs;
using lynxmotion_al5d::parseSpawnArgs;

TEST(SpawnArgs, MinimalColourOnly)
{
  const auto r = parseSpawnArgs({"-c", "red"});
  ASSERT_EQ(r.status, ParseStatus::Ok) << r.error;
  EXPECT_EQ(r.args.color, "red");
  EXPECT_EQ(r.args.name, "");
  EXPECT_DOUBLE_EQ(r.args.x, 0.0);
}

TEST(SpawnArgs, AllOptions)
{
  const auto r = parseSpawnArgs(
    {"-c", "blue", "-n", "b1", "-x", "0.1", "-y", "-0.2", "-z", "0.05", "-R", "1", "-P", "2",
      "-Y", "3"});
  ASSERT_EQ(r.status, ParseStatus::Ok) << r.error;
  EXPECT_EQ(r.args.color, "blue");
  EXPECT_EQ(r.args.name, "b1");
  EXPECT_DOUBLE_EQ(r.args.x, 0.1);
  EXPECT_DOUBLE_EQ(r.args.y, -0.2);
  EXPECT_DOUBLE_EQ(r.args.z, 0.05);
  EXPECT_DOUBLE_EQ(r.args.roll, 1.0);
  EXPECT_DOUBLE_EQ(r.args.pitch, 2.0);
  EXPECT_DOUBLE_EQ(r.args.yaw, 3.0);
}

TEST(SpawnArgs, ColourIsRequired)
{
  EXPECT_EQ(parseSpawnArgs({"-x", "1"}).status, ParseStatus::Error);
  EXPECT_EQ(parseSpawnArgs({}).status, ParseStatus::Error);
}

TEST(SpawnArgs, RejectsBadInput)
{
  EXPECT_EQ(parseSpawnArgs({"-c"}).status, ParseStatus::Error);                 // missing value
  EXPECT_EQ(parseSpawnArgs({"-c", "red", "-x", "abc"}).status, ParseStatus::Error);
  EXPECT_EQ(parseSpawnArgs({"-c", "red", "-x", "1.5m"}).status, ParseStatus::Error);
  EXPECT_EQ(parseSpawnArgs({"-c", "red", "-x", "nan"}).status, ParseStatus::Error);
  EXPECT_EQ(parseSpawnArgs({"-c", "red", "-q", "1"}).status, ParseStatus::Error);
  EXPECT_EQ(parseSpawnArgs({"-c", "red", "stray"}).status, ParseStatus::Error);
}

TEST(SpawnArgs, ErrorMentionsOffendingArgument)
{
  const auto r = parseSpawnArgs({"-c", "red", "-x", "abc"});
  EXPECT_NE(r.error.find("abc"), std::string::npos);
}

TEST(SpawnArgs, Help)
{
  EXPECT_EQ(parseSpawnArgs({"-h"}).status, ParseStatus::Help);
  EXPECT_EQ(parseSpawnArgs({"--help"}).status, ParseStatus::Help);
}

TEST(KillArgs, Parses)
{
  const auto r = parseKillArgs({"brick3"});
  ASSERT_EQ(r.status, ParseStatus::Ok);
  EXPECT_EQ(r.name, "brick3");
  EXPECT_EQ(parseKillArgs({}).status, ParseStatus::Error);
  EXPECT_EQ(parseKillArgs({"a", "b"}).status, ParseStatus::Error);
  EXPECT_EQ(parseKillArgs({"--help"}).status, ParseStatus::Help);
}
