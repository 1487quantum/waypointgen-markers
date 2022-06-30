#include <gtest/gtest.h>

#include "waypointgen/waypointgen_utils.h"

std_msgs::Header mHeader;
std::string frameID = "world";

waypointgen_utils wputil;

auto genPoint(const double &x, const double &y, const double &z) {
  geometry_msgs::Point a;
  a.x = x;
  a.y = y;
  a.z = z;
  return a;
}

auto genQuaternion(const double &x, const double &y, const double &z,
                   const double &w) {
  tf::Quaternion quat(x, y, z, w);
  return quat;
}

// Util Test
TEST(waypointgen_utils, frameStrTest) {
  wputil.update_header(mHeader, frameID);
  EXPECT_EQ(mHeader.frame_id, frameID);
  wputil.update_header(mHeader, "");
  EXPECT_EQ(mHeader.frame_id, mHeader.frame_id);
  wputil.update_header(mHeader, " ");
  EXPECT_EQ(mHeader.frame_id, mHeader.frame_id);
}

TEST(waypointgen_utils, addPosePositiveTest) {
  auto qt1{genQuaternion(0, 0, 0, 1)};
  auto pt1{genPoint(1, 2, 3)};
  auto p1{wputil.addPoseCov(pt1, qt1)};
  EXPECT_EQ(p1.pose.position, pt1);

  auto pt2{genPoint(2.0f, 1.5f, 100.23f)};
  auto p2{wputil.addPoseCov(pt2, qt1)};
  EXPECT_EQ(p2.pose.position, pt2);

  auto pt3{genPoint(3.1415926535897932382, 5.18346364, 1.618)};
  auto p3{wputil.addPoseCov(pt3, qt1)};
  EXPECT_EQ(p3.pose.position, pt3);
}

TEST(waypointgen_utils, addPoseNegativeTest) {
  auto qt1{genQuaternion(0, 0, 0, 1)};
  auto pt1{genPoint(-1, -2, -3)};
  auto p1{wputil.addPoseCov(pt1, qt1)};
  EXPECT_EQ(p1.pose.position, pt1);

  auto pt2{genPoint(-2.0f, -1.5f, -100.23f)};
  auto p2{wputil.addPoseCov(pt2, qt1)};
  EXPECT_EQ(p2.pose.position, pt2);

  auto pt3{genPoint(-3.1415926535897932382, -5.18346364, -1.618)};
  auto p3{wputil.addPoseCov(pt3, qt1)};
  EXPECT_EQ(p3.pose.position, pt3);
}

TEST(waypointgen_utils, addPoseZeroTest) {
  auto qt1{genQuaternion(0, 0, 0, 1)};
  auto pt1{genPoint(0, 0, 0)};
  auto p1{wputil.addPoseCov(pt1, qt1)};
  EXPECT_EQ(p1.pose.position, pt1);

  auto pt2{genPoint(0.0f, 0.0f, 0.0f)};
  auto p2{wputil.addPoseCov(pt2, qt1)};
  EXPECT_EQ(p2.pose.position, pt2);

  auto pt3{genPoint(0.0, 0.0, 0.0)};
  auto p3{wputil.addPoseCov(pt3, qt1)};
  EXPECT_EQ(p3.pose.position, pt3);
}

TEST(waypointgen_utils, addPoseUndefinedTest) {
  auto qt1{genQuaternion(0, 0, 0, 1)};
  auto pt1{genPoint(sqrt(-1), sqrt(-1), sqrt(-1))};
  auto p1{wputil.addPoseCov(pt1, qt1)};
  EXPECT_EQ(p1.pose.position, genPoint(-1, -1, -1));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_waypointgen");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}