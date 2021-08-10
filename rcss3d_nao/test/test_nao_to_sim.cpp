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
#include <map>
#include <utility>
#include <vector>
#include <algorithm>
#include <string>
#include "rcss3d_nao/nao_to_sim.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"

void test(
  nao_command_msgs::msg::JointPositions nao_joints,
  rcss3d_controller_msgs::msg::JointPositionCommand expected_sim_joints)
{
  auto converted = rcss3d_nao::nao_to_sim::getJointPositionCommand(nao_joints);

  ASSERT_EQ(converted.name.size(), expected_sim_joints.name.size());
  ASSERT_EQ(converted.position.size(), expected_sim_joints.position.size());

  for (unsigned i = 0; i < expected_sim_joints.name.size(); ++i) {
    // Check that same name exists in converted vector
    auto it =
      std::find(converted.name.begin(), converted.name.end(), expected_sim_joints.name.at(i));
    EXPECT_NE(it, converted.name.end());

    // If found, check values are the same
    if (it != converted.name.end()) {
      auto i2 = it - converted.name.begin();
      EXPECT_EQ(converted.position.at(i2), expected_sim_joints.position.at(i));
    }
  }
}

nao_command_msgs::msg::JointPositions createJointPositionsMsg(
  std::vector<std::pair<int,
  float>> nao_joints_vec)
{
  nao_command_msgs::msg::JointPositions naoJoints;

  for (auto const &[key, val] : nao_joints_vec) {
    naoJoints.indexes.push_back(key);
    naoJoints.positions.push_back(val);
  }
  return naoJoints;
}


TEST(TestJointsNaoToSim, TestOneJoint)
{
  nao_command_msgs::msg::JointPositions nao_joints;
  nao_joints.indexes.push_back(nao_command_msgs::msg::JointIndexes::LHIPROLL);
  nao_joints.positions.push_back(0.4);

  rcss3d_controller_msgs::msg::JointPositionCommand expected_sim_joints;
  expected_sim_joints.name.push_back("lle2");
  expected_sim_joints.position.push_back(0.4);

  test(nao_joints, expected_sim_joints);
}

TEST(TestJointsNaoToSim, TestOneJointInversion)
{
  nao_command_msgs::msg::JointPositions nao_joints;
  nao_joints.indexes.push_back(nao_command_msgs::msg::JointIndexes::LHIPPITCH);
  nao_joints.positions.push_back(0.6);

  rcss3d_controller_msgs::msg::JointPositionCommand expected_sim_joints;
  expected_sim_joints.name.push_back("lle3");
  expected_sim_joints.position.push_back(-0.6);

  test(nao_joints, expected_sim_joints);
}


TEST(TestJointsNaoToSim, Test)
{
  // The order of joints can be in any order.
  // In this case, headpitch is missing and headyaw is at end of vector.
  std::vector<std::pair<int, float>> nao_joints_vec = {
    {nao_command_msgs::msg::JointIndexes::HEADPITCH, 0},
    {nao_command_msgs::msg::JointIndexes::LSHOULDERPITCH, 0.01},
    {nao_command_msgs::msg::JointIndexes::LSHOULDERROLL, 0.02},
    {nao_command_msgs::msg::JointIndexes::LELBOWYAW, 0.03},
    {nao_command_msgs::msg::JointIndexes::LELBOWROLL, 0.04},
    {nao_command_msgs::msg::JointIndexes::LWRISTYAW, 0.05},
    {nao_command_msgs::msg::JointIndexes::LHIPYAWPITCH, 0.06},
    {nao_command_msgs::msg::JointIndexes::LHIPROLL, 0.07},
    {nao_command_msgs::msg::JointIndexes::LHIPPITCH, 0.08},
    {nao_command_msgs::msg::JointIndexes::LKNEEPITCH, 0.09},
    {nao_command_msgs::msg::JointIndexes::LANKLEPITCH, 0.10},
    {nao_command_msgs::msg::JointIndexes::LANKLEROLL, 0.11},
    {nao_command_msgs::msg::JointIndexes::RHIPROLL, 0.12},
    {nao_command_msgs::msg::JointIndexes::RHIPPITCH, 0.13},
    {nao_command_msgs::msg::JointIndexes::RKNEEPITCH, 0.14},
    {nao_command_msgs::msg::JointIndexes::RANKLEPITCH, 0.15},
    {nao_command_msgs::msg::JointIndexes::RANKLEROLL, 0.16},
    {nao_command_msgs::msg::JointIndexes::RSHOULDERPITCH, 0.17},
    {nao_command_msgs::msg::JointIndexes::RSHOULDERROLL, 0.18},
    {nao_command_msgs::msg::JointIndexes::RELBOWYAW, 0.19},
    {nao_command_msgs::msg::JointIndexes::RELBOWROLL, 0.20},
    {nao_command_msgs::msg::JointIndexes::RWRISTYAW, 0.21},
    {nao_command_msgs::msg::JointIndexes::LHAND, 0.22},
    {nao_command_msgs::msg::JointIndexes::RHAND, 0.23},
    {nao_command_msgs::msg::JointIndexes::HEADYAW, -0.01}};

  std::map<std::string, float> expected_sim_joints_map = {
    {"he1", -0.01},
    {"he2", 0},
    {"lae1", -0.01},
    {"lae2", 0.02},
    {"lae3", 0.03},
    {"lae4", 0.04},
    {"lle1", 0.06},
    {"lle2", 0.07},
    {"lle3", -0.08},
    {"lle4", -0.09},
    {"lle5", -0.10},
    {"lle6", 0.11},
    {"rle1", 0.06},
    {"rle2", 0.12},
    {"rle3", -0.13},
    {"rle4", -0.14},
    {"rle5", -0.15},
    {"rle6", 0.16},
    {"rae1", -0.17},
    {"rae2", 0.18},
    {"rae3", 0.19},
    {"rae4", 0.20},
  };

  rcss3d_controller_msgs::msg::JointPositionCommand expected_sim_joints;
  for (auto const & [name, position] : expected_sim_joints_map) {
    expected_sim_joints.name.push_back(name);
    expected_sim_joints.position.push_back(position);
  }

  nao_command_msgs::msg::JointPositions nao_joints = createJointPositionsMsg(nao_joints_vec);
  test(nao_joints, expected_sim_joints);
}
