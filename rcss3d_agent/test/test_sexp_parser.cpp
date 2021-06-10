// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include "rcss3d_agent/sexp_parser.hpp"
#include "rcss3d_agent/deg2rad.hpp"
#include "rcss3d_agent/polar_to_point.hpp"

// Taken from https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#vision-perceptors
// There is one extra backet (typo) in the original wiki text which has been fixed here
static const char * sexp =
  "(See (G2R (pol 17.55 -3.33 4.31)) "
  "(G1R (pol 17.52 3.27 4.07)) "
  "(F1R (pol 18.52 18.94 1.54)) "
  "(F2R (pol 18.52 -18.91 1.52)) "
  "(B (pol 8.51 -0.21 -0.17)) "
  "(P (team teamRed) (id 1) "
  "(head (pol 16.98 -0.21 3.19)) "
  "(rlowerarm (pol 16.83 -0.06 2.80)) "
  "(llowerarm (pol 16.86 -0.36 3.10)) "
  "(rfoot (pol 17.00 0.29 1.68)) "
  "(lfoot (pol 16.95 -0.51 1.32))) "
  "(P (team teamBlue) (id 3) "
  "(rlowerarm (pol 0.18 -33.55 -20.16)) "
  "(llowerarm (pol 0.18 34.29 -19.80))) "
  "(L (pol 12.11 -40.77 -2.40) (pol 12.95 -37.76 -2.41)) "
  "(L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20)))";

static const char * sexp_empty = "(See )";
static const char * sexp_none = "";

TEST(TestGoalposts, TestHasGoalposts)
{
  SexpParser parser(sexp);
  auto [found, goalposts] = parser.getGoalposts();
  ASSERT_EQ(found, true);
  ASSERT_EQ(goalposts.posts.size(), 2u);

  // Checks in order of: G1L, G1R, G2L, G2R

  // (G1R (pol 17.52 3.27 4.07)) 
  soccer_vision_msgs::msg::Goalpost & post1 = goalposts.posts.at(0);
  EXPECT_EQ(post1.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(post1.observed_top, true);
  geometry_msgs::msg::Point point1 = polar_to_point(17.52, deg2rad(3.27), deg2rad(4.07));
  EXPECT_NEAR(post1.point.x, point1.x, 0.01);
  EXPECT_NEAR(post1.point.y, point1.y, 0.01);
  EXPECT_NEAR(post1.point.z, point1.z, 0.01);

  // (G2R (pol 17.55 -3.33 4.31))
  soccer_vision_msgs::msg::Goalpost & post2 = goalposts.posts.at(1);
  EXPECT_EQ(post2.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(post2.observed_top, true);
  geometry_msgs::msg::Point point2 = polar_to_point(17.55, deg2rad(-3.33), deg2rad(4.31));
  EXPECT_NEAR(post2.point.x, point2.x, 0.01);
  EXPECT_NEAR(post2.point.y, point2.y, 0.01);
  EXPECT_NEAR(post2.point.z, point2.z, 0.01);
}

TEST(TestGoalposts, TestNoGoalposts)
{
  SexpParser parser(sexp_empty);
  auto [found, goalposts] = parser.getGoalposts();
  ASSERT_EQ(found, false);
}

TEST(TestGoalposts, TestNoVisionData)
{
  SexpParser parser(sexp_none);
  auto [found, goalposts] = parser.getGoalposts();
  ASSERT_EQ(found, false);
}

TEST(TestBall, TestHasBall)
{
  SexpParser parser(sexp);
  auto [ball_found, ball] = parser.getBall();
  ASSERT_EQ(ball_found, true);

  // (B (pol 8.51 -0.21 -0.17))
  EXPECT_EQ(ball.header.frame_id, "CameraTop_frame");
  geometry_msgs::msg::Point point = polar_to_point(8.51, deg2rad(-0.21), deg2rad(-0.17));
  EXPECT_EQ(ball.point.x, point.x);
  EXPECT_EQ(ball.point.y, point.y);
  EXPECT_EQ(ball.point.z, point.z);
}

TEST(TestBall, TestNoBall)
{
  SexpParser parser(sexp_empty);
  auto [found, ball] = parser.getBall();
  ASSERT_EQ(found, false);
}

TEST(TestBall, TestNoVisionData)
{
  SexpParser parser(sexp_none);
  auto [found, ball] = parser.getBall();
  ASSERT_EQ(found, false);
}

TEST(TestFieldLines, TestHasFieldLines)
{
  SexpParser parser(sexp);
  auto [lines_found, lines] = parser.getFieldLines();
  ASSERT_EQ(lines_found, true);
  ASSERT_EQ(lines.lines.size(), 2u);

  // (L (pol 12.11 -40.77 -2.40) (pol 12.95 -37.76 -2.41))
  soccer_vision_msgs::msg::FieldLine & line1 = lines.lines.at(0);
  EXPECT_EQ(line1.header.frame_id, "CameraTop_frame");
  geometry_msgs::msg::Point start1 = polar_to_point(12.11, deg2rad(-40.77), deg2rad(-2.40));
  geometry_msgs::msg::Point end1 = polar_to_point(12.95, deg2rad(-37.76), deg2rad(-2.41));
  EXPECT_NEAR(line1.start.x, start1.x, 0.01);
  EXPECT_NEAR(line1.start.y, start1.y, 0.01);
  EXPECT_NEAR(line1.start.z, start1.z, 0.01);
  EXPECT_NEAR(line1.end.x, end1.x, 0.01);
  EXPECT_NEAR(line1.end.y, end1.y, 0.01);
  EXPECT_NEAR(line1.end.z, end1.z, 0.01);

  // (L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20))
  soccer_vision_msgs::msg::FieldLine & line2 = lines.lines.at(1);
  EXPECT_EQ(line2.header.frame_id, "CameraTop_frame");
  geometry_msgs::msg::Point start2 = polar_to_point(12.97, deg2rad(-37.56), deg2rad(-2.24));
  geometry_msgs::msg::Point end2 = polar_to_point(13.32, deg2rad(-32.98), deg2rad(-2.20));
  EXPECT_NEAR(line2.start.x, start2.x, 0.01);
  EXPECT_NEAR(line2.start.y, start2.y, 0.01);
  EXPECT_NEAR(line2.start.z, start2.z, 0.01);
  EXPECT_NEAR(line2.end.x, end2.x, 0.01);
  EXPECT_NEAR(line2.end.y, end2.y, 0.01);
  EXPECT_NEAR(line2.end.z, end2.z, 0.01);
}

TEST(TestFieldLines, TestNoFieldLines)
{
  SexpParser parser(sexp_empty);
  auto [found, lines] = parser.getFieldLines();
  ASSERT_EQ(found, false);
}

TEST(TestFieldLines, TestNoVisionData)
{
  SexpParser parser(sexp_none);
  auto [found, lines] = parser.getFieldLines();
  ASSERT_EQ(found, false);
}

TEST(TestRobots, TestHasRobots)
{
  // Currently, WE ONLY DETECT THE "HEAD" of the robot
  // If a limb is sent, we ignore it. That's why the
  // second robot is not counted as a robot in thie test.
  SexpParser parser(sexp);
  auto [found, robots] = parser.getRobots();
  ASSERT_EQ(found, true);
  ASSERT_EQ(robots.robots.size(), 1u);

  // (head (pol 16.98 -0.21 3.19))
  soccer_vision_msgs::msg::Robot & robot1 = robots.robots.at(0);
  EXPECT_EQ(robot1.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(robot1.team, "teamRed");
  EXPECT_EQ(robot1.id, 1);
  geometry_msgs::msg::Point head = polar_to_point(16.98, deg2rad(-0.21), deg2rad(3.19));
  EXPECT_NEAR(robot1.head.x, head.x, 0.01);
  EXPECT_NEAR(robot1.head.y, head.y, 0.01);
  EXPECT_NEAR(robot1.head.z, head.z, 0.01);
}

TEST(TestRobots, TestNoRobots)
{
  SexpParser parser(sexp_empty);
  auto [found, robots] = parser.getRobots();
  ASSERT_EQ(found, false);
}

TEST(TestRobots, TestNoVisionData)
{
  SexpParser parser(sexp_none);
  auto [found, robots] = parser.getRobots();
  ASSERT_EQ(found, false);
}

TEST(TestFlags, TestHasFlags)
{
  SexpParser parser(sexp);
  auto [found, flags] = parser.getFlags();
  ASSERT_EQ(found, true);
  ASSERT_EQ(flags.flags.size(), 2u);

  // Checks in order of: F1L, F1R, F2L, F2R

  // "(F1R (pol 18.52 18.94 1.54)) "
  soccer_vision_msgs::msg::Flag & flag1 = flags.flags.at(0);
  EXPECT_EQ(flag1.header.frame_id, "CameraTop_frame");
  geometry_msgs::msg::Point base1 = polar_to_point(18.52, deg2rad(18.94), deg2rad(1.54));
  EXPECT_NEAR(flag1.base.x, base1.x, 0.01);
  EXPECT_NEAR(flag1.base.y, base1.y, 0.01);
  EXPECT_NEAR(flag1.base.z, base1.z, 0.01);

  // "(F2R (pol 18.52 -18.91 1.52)) "
  soccer_vision_msgs::msg::Flag & flag2 = flags.flags.at(1);
  EXPECT_EQ(flag2.header.frame_id, "CameraTop_frame");
  geometry_msgs::msg::Point base2 = polar_to_point(18.52, deg2rad(-18.91), deg2rad(1.52));
  EXPECT_NEAR(flag2.base.x, base2.x, 0.01);
  EXPECT_NEAR(flag2.base.y, base2.y, 0.01);
  EXPECT_NEAR(flag2.base.z, base2.z, 0.01);  
}

TEST(TestFlags, TestNoFlags)
{
  SexpParser parser(sexp_empty);
  auto [found, flags] = parser.getFlags();
  ASSERT_EQ(found, false);
}

TEST(TestFlags, TestNoVisionData)
{
  SexpParser parser(sexp_none);
  auto [found, flags] = parser.getFlags();
  ASSERT_EQ(found, false);
}