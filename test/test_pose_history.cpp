// Copyright 2026. Licensed under the MIT License.
#include <gtest/gtest.h>
#include "pose_compare_plotter/pose_history.hpp"

using pose_compare_plotter::Quantity;
using pose_compare_plotter::Stream;

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, int seconds, unsigned nanoseconds = 0)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = "optitrack";
  message.header.stamp.sec = seconds;
  message.header.stamp.nanosec = nanoseconds;
  message.pose.position.x = x;
  message.pose.orientation.w = 1;
  return message;
}
}  // namespace

TEST(PoseHistory, UserQuaternionsShowHalfTurnWithoutTreatingNegativeWAsRotation)
{
  Stream raw, vision;
  auto message = pose(0.017161427, 100);
  auto & q = message.pose.orientation;
  q.x = 0.0017468433361500502;
  q.y = -0.0007252198993228376;
  q.z = -0.000843463116325438;
  q.w = -0.9999978542327881;
  ASSERT_TRUE(raw.append(message, 0, 30));
  EXPECT_NEAR(raw.samples.back().rpy[2], 0.0965085, 1e-5);
  EXPECT_NEAR(raw.samples.back().rpy[0], -0.2001036, 1e-5);
  EXPECT_LT(raw.samples.back().quaternion[3], 0);
  q.x = 0.0006989179589564222;
  q.y = 0.0017365295077226635;
  q.z = -0.9999978989226532;
  q.w = 0.0008355412824491722;
  ASSERT_TRUE(vision.append(message, 0, 30));
  EXPECT_NEAR(vision.samples.back().rpy[2], -179.904393, 1e-5);
  for (auto * component : {&q.x, &q.y, &q.z, &q.w}) {*component *= -2;}
  ASSERT_TRUE(vision.append(message, 0.1, 30));
  EXPECT_NEAR(vision.samples.back().rpy[2], -179.904393, 1e-5);
}

TEST(PoseHistory, ZeroPreservesOppositeSignsAndRequiresFreshSamples)
{
  Stream raw, vision;
  EXPECT_FALSE(raw.zero(0));
  raw.append(pose(1, 100), 0, 30);
  vision.append(pose(-1, 100), 0, 30);
  EXPECT_TRUE(raw.zero(0));
  EXPECT_TRUE(vision.zero(0));
  raw.append(pose(1.2, 100, 100000000), 0.1, 30);
  vision.append(pose(-1.2, 100, 100000000), 0.1, 30);
  EXPECT_NEAR(raw.value(raw.samples.back(), Quantity::Position, 0), 0.2, 1e-8);
  EXPECT_NEAR(vision.value(vision.samples.back(), Quantity::Position, 0), -0.2, 1e-8);
  EXPECT_FALSE(raw.zero(2));
}

TEST(PoseHistory, VelocityUsesHeaderIntervalsAndSkipsDiscontinuities)
{
  Stream stream;
  stream.append(pose(0, 100), 0, 30);
  EXPECT_TRUE(std::isnan(stream.samples.back().velocity[0]));
  stream.append(pose(0.1, 100, 100000000), 0.2, 30);
  EXPECT_NEAR(stream.samples.back().velocity[0], 1, 1e-6);
  stream.append(pose(0.2, 100, 100000000), 0.21, 30);  // Duplicate stamp.
  EXPECT_TRUE(std::isnan(stream.samples.back().velocity[0]));
  stream.append(pose(0.3, 99), 0.22, 30);  // Clock reset.
  EXPECT_TRUE(std::isnan(stream.samples.back().velocity[0]));
  stream.append(pose(0.4, 101), 0.23, 30);  // Long timestamp interval.
  EXPECT_TRUE(std::isnan(stream.samples.back().velocity[0]));
  stream.append(pose(0.5, 101, 100000000), 2, 30);  // Reception outage.
  EXPECT_TRUE(std::isnan(stream.samples.back().velocity[0]));
  stream.append(pose(0.6, 0), 2.1, 30);  // Unstamped.
  EXPECT_TRUE(std::isnan(stream.samples.back().velocity[0]));
}

TEST(PoseHistory, InvalidInputIsVisibleAndFrameChangesResetHistoryAndOrigin)
{
  Stream stream;
  auto message = pose(2, 100);
  stream.append(message, 0, 30);
  stream.zero(0);
  message.pose.orientation.w = 0;
  EXPECT_FALSE(stream.append(message, 0.1, 30));
  message.pose.orientation.w = 1;
  message.pose.position.x = pose_compare_plotter::kNan;
  EXPECT_FALSE(stream.append(message, 0.2, 30));
  EXPECT_EQ(stream.rejected, 2u);
  EXPECT_EQ(stream.accepted, 1u);
  message = pose(7, 101);
  message.header.frame_id = "new_frame";
  ASSERT_TRUE(stream.append(message, 0.3, 30));
  EXPECT_EQ(stream.samples.size(), 1u);
  EXPECT_EQ(stream.frame_changes, 1u);
  EXPECT_FALSE(stream.relative);
  EXPECT_TRUE(std::isnan(stream.samples.back().velocity[0]));
}

TEST(PoseHistory, HistoryAndRateStayBoundedAndExpireWhenStreamStops)
{
  Stream stream;
  for (int i = 0; i < 100; ++i) {stream.append(pose(i, 100), i * 0.1, 5);}
  EXPECT_LE(stream.samples.size(), 51u);
  EXPECT_NEAR(stream.rate(9.9), 10, 1e-8);
  EXPECT_FALSE(stream.fresh(12));
  EXPECT_EQ(stream.rate(12), 0);
  stream.prune(16, 5);
  EXPECT_TRUE(stream.samples.empty());
  for (std::size_t i = 0; i < pose_compare_plotter::kMaxSamples + 10; ++i) {
    stream.append(pose(0, 100), 20, 30);
  }
  EXPECT_EQ(stream.samples.size(), pose_compare_plotter::kMaxSamples);
}
