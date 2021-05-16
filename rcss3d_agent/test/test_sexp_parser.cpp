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

// static const std::string sexp = "(See (B (pol 8.51 -0.21 -0.17)))";

TEST(TestSexpParser, TestGoalPosts)
{
  SexpParser parser(sexp);
  auto [found, goalposts] = parser.getGoalposts();
  ASSERT_EQ(found, true);
  ASSERT_EQ(goalposts.posts.size(), 2u);

  // Check in order of: G1L, G1R, G2L, G2R
  naosoccer_interfaces::msg::Goalpost & post1 = goalposts.posts.at(0);
  EXPECT_EQ(post1.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(post1.observed_top.data, true);
  EXPECT_NEAR(post1.top.x, 17.4473, 0.01);
  EXPECT_NEAR(post1.top.y, 0.9968, 0.01);
  EXPECT_NEAR(post1.top.z, 1.2434, 0.01);

  naosoccer_interfaces::msg::Goalpost & post2 = goalposts.posts.at(1);
  EXPECT_EQ(post2.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(post2.observed_top.data, true);
  EXPECT_NEAR(post2.top.x, 17.4708, 0.01);
  EXPECT_NEAR(post2.top.y, -1.0165, 0.01);
  EXPECT_NEAR(post2.top.z, 1.3189, 0.01);
}

TEST(TestSexpParser, TestBall)
{
  SexpParser parser(sexp);
  auto [ball_found, ball] = parser.getBall();
  ASSERT_EQ(ball_found, true);

  EXPECT_EQ(ball.header.frame_id, "CameraTop_frame");
  EXPECT_NEAR(ball.point.x, 8.5099, 0.01);
  EXPECT_NEAR(ball.point.y, -0.0312, 0.01);
  EXPECT_NEAR(ball.point.z, -0.0252, 0.01);
}
