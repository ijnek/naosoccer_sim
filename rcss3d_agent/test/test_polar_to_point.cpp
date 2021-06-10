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
#include "rcss3d_agent/polar_to_point.hpp"
#include "rcss3d_agent/deg2rad.hpp"
#include "geometry_msgs/msg/point.hpp"

TEST(TestPolarToPoint, Test1)
{
  float dist = 17.55;
  float phi = deg2rad(-3.33);
  float theta = deg2rad(4.31);

  geometry_msgs::msg::Point point = polar_to_point(dist, phi, theta);

  // True values were manually calculated by hand
  EXPECT_NEAR(point.x, 17.4708, 0.01);
  EXPECT_NEAR(point.y, -1.0165, 0.01);
  EXPECT_NEAR(point.z, 1.3189, 0.01);
}

TEST(TestPolarToPoint, Test2)
{
  float dist = 8.51;
  float phi = deg2rad(-0.21);
  float theta = deg2rad(-0.17);

  // True values were manually calculated by hand
  geometry_msgs::msg::Point point = polar_to_point(dist, phi, theta);
  EXPECT_NEAR(point.x, 8.5099, 0.01);
  EXPECT_NEAR(point.y, -0.0312, 0.01);
  EXPECT_NEAR(point.z, -0.0252, 0.01);
}
