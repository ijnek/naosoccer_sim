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
#include <string>
#include <utility>
#include <vector>
#include <map>
#include "rcss3d_agent/sim_to_nao.hpp"
#include "nao_sensor_msgs/msg/joint_indexes.hpp"

void test(
  std::vector<std::pair<std::string, float>> sim_joints,
  std::map<int, float> expected)
{
  nao_sensor_msgs::msg::JointPositions converted = sim_to_nao(sim_joints);

  for (auto const & [key, val] : expected) {
    EXPECT_EQ(converted.positions.at(key), val);
  }
}

TEST(TestSimToNao, Test)
{
  std::vector<std::pair<std::string, float>> sim_joints = {
    {"hj1", -0.01},
    {"hj2", -0.02},
    {"laj1", 0.01},
    {"laj2", 0.02},
    {"laj3", 0.03},
    {"laj4", 0.04},
    {"llj1", 0.05},
    {"llj2", 0.06},
    {"llj3", 0.07},
    {"llj4", 0.08},
    {"llj5", 0.09},
    {"llj6", 0.10},
    {"rlj2", 0.11},
    {"rlj3", 0.12},
    {"rlj4", 0.13},
    {"rlj5", 0.14},
    {"rlj6", 0.15},
    {"raj1", 0.16},
    {"raj2", 0.17},
    {"raj3", 0.18},
    {"raj4", 0.19}};

  std::map<int, float> expected_nao_joint_positions = {
    {nao_sensor_msgs::msg::JointIndexes::HEADYAW, -0.01},
    {nao_sensor_msgs::msg::JointIndexes::HEADPITCH, 0.02},
    {nao_sensor_msgs::msg::JointIndexes::LSHOULDERPITCH, -0.01},
    {nao_sensor_msgs::msg::JointIndexes::LSHOULDERROLL, 0.02},
    {nao_sensor_msgs::msg::JointIndexes::LELBOWYAW, 0.03},
    {nao_sensor_msgs::msg::JointIndexes::LELBOWROLL, 0.04},
    {nao_sensor_msgs::msg::JointIndexes::LHIPYAWPITCH, 0.05},
    {nao_sensor_msgs::msg::JointIndexes::LHIPROLL, 0.06},
    {nao_sensor_msgs::msg::JointIndexes::LHIPPITCH, -0.07},
    {nao_sensor_msgs::msg::JointIndexes::LKNEEPITCH, -0.08},
    {nao_sensor_msgs::msg::JointIndexes::LANKLEPITCH, -0.09},
    {nao_sensor_msgs::msg::JointIndexes::LANKLEROLL, 0.10},
    {nao_sensor_msgs::msg::JointIndexes::RHIPROLL, 0.11},
    {nao_sensor_msgs::msg::JointIndexes::RHIPPITCH, -0.12},
    {nao_sensor_msgs::msg::JointIndexes::RKNEEPITCH, -0.13},
    {nao_sensor_msgs::msg::JointIndexes::RANKLEPITCH, -0.14},
    {nao_sensor_msgs::msg::JointIndexes::RANKLEROLL, 0.15},
    {nao_sensor_msgs::msg::JointIndexes::RSHOULDERPITCH, -0.16},
    {nao_sensor_msgs::msg::JointIndexes::RSHOULDERROLL, 0.17},
    {nao_sensor_msgs::msg::JointIndexes::RELBOWYAW, 0.18},
    {nao_sensor_msgs::msg::JointIndexes::RELBOWROLL, 0.19},

    // below should be 0 because they don't exist in simulation.
    {nao_sensor_msgs::msg::JointIndexes::LWRISTYAW, 0.0},
    {nao_sensor_msgs::msg::JointIndexes::RWRISTYAW, 0.0},
    {nao_sensor_msgs::msg::JointIndexes::LHAND, 0.0},
    {nao_sensor_msgs::msg::JointIndexes::RHAND, 0.0}};

  test(sim_joints, expected_nao_joint_positions);
}
