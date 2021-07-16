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

#include "rcss3d_agent/complementary_filter.hpp"

#include <cmath>

#define SIM_DT 1.0 / 50.0

#define ACC_WEIGHT 0.01
#define GYR_WEIGHT (1.0 - ACC_WEIGHT)

// Using implementation provided here: https://www.youtube.com/watch?v=whSw42XddsU&ab_channel=BrianDouglas
// Axes of NAO are explained here: http://doc.aldebaran.com/2-1/family/robots/inertial_robot.html

nao_sensor_msgs::msg::Angle ComplementaryFilter::update(
  const nao_sensor_msgs::msg::Accelerometer & acc,
  const nao_sensor_msgs::msg::Gyroscope & gyr)
{
  float angle_x_from_gyr = x_ + gyr.x * SIM_DT;
  float angle_y_from_gyr = y_ + gyr.y * SIM_DT;

  float angle_x_from_acc = atan2(acc.y, acc.z);
  float angle_y_from_acc = atan2(-acc.x, acc.z);

  x_ = GYR_WEIGHT * angle_x_from_gyr + ACC_WEIGHT * angle_x_from_acc;
  y_ = GYR_WEIGHT * angle_y_from_gyr + ACC_WEIGHT * angle_y_from_acc;

  return getAngle();
}

nao_sensor_msgs::msg::Angle ComplementaryFilter::getAngle()
{
  nao_sensor_msgs::msg::Angle angles;
  angles.x = x_;
  angles.y = y_;
  return angles;
}
