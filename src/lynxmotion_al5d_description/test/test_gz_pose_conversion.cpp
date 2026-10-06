#include <gtest/gtest.h>

#include <gz/msgs/pose_v.pb.h>

#include "lynxmotion_al5d_description/gz_pose_conversion.hpp"

using lynxmotion_al5d::poseVectorToTf;

namespace
{
gz::msgs::Pose * addPose(gz::msgs::Pose_V & v, const std::string & name, double x, double yaw_z)
{
  auto * p = v.add_pose();
  p->set_name(name);
  p->mutable_position()->set_x(x);
  p->mutable_orientation()->set_z(yaw_z);
  p->mutable_orientation()->set_w(1.0);
  return p;
}
}  // namespace

TEST(GzPoseConversion, NamesBecomeChildFrames)
{
  gz::msgs::Pose_V poses;
  poses.mutable_header()->mutable_stamp()->set_sec(12);
  poses.mutable_header()->mutable_stamp()->set_nsec(500);
  addPose(poses, "brick1", 0.25, 0.0);
  addPose(poses, "brick2", -1.0, 0.5);

  const auto tf = poseVectorToTf(poses);
  ASSERT_EQ(tf.transforms.size(), 2u);
  EXPECT_EQ(tf.transforms[0].child_frame_id, "brick1");
  EXPECT_EQ(tf.transforms[0].header.frame_id, "world");
  EXPECT_EQ(tf.transforms[0].header.stamp.sec, 12);
  EXPECT_EQ(tf.transforms[0].header.stamp.nanosec, 500u);
  EXPECT_DOUBLE_EQ(tf.transforms[0].transform.translation.x, 0.25);
  EXPECT_EQ(tf.transforms[1].child_frame_id, "brick2");
  EXPECT_DOUBLE_EQ(tf.transforms[1].transform.translation.x, -1.0);
  EXPECT_DOUBLE_EQ(tf.transforms[1].transform.rotation.z, 0.5);
  EXPECT_DOUBLE_EQ(tf.transforms[1].transform.rotation.w, 1.0);
}

TEST(GzPoseConversion, UnnamedEntriesAreDropped)
{
  gz::msgs::Pose_V poses;
  addPose(poses, "", 1.0, 0.0);
  addPose(poses, "kept", 2.0, 0.0);
  const auto tf = poseVectorToTf(poses);
  ASSERT_EQ(tf.transforms.size(), 1u);
  EXPECT_EQ(tf.transforms[0].child_frame_id, "kept");
}

TEST(GzPoseConversion, EmptyInputGivesEmptyOutput)
{
  EXPECT_TRUE(poseVectorToTf(gz::msgs::Pose_V()).transforms.empty());
}

TEST(GzPoseConversion, CustomWorldFrame)
{
  gz::msgs::Pose_V poses;
  addPose(poses, "a", 0.0, 0.0);
  EXPECT_EQ(poseVectorToTf(poses, "map").transforms[0].header.frame_id, "map");
}
