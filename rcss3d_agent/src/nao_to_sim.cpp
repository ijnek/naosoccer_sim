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

#include <map>
#include <vector>
#include <utility>
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent/nao_to_sim.hpp"
#include "rcss3d_agent/sim_joints.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"

static rclcpp::Logger nao_to_sim_logger = rclcpp::get_logger("nao_to_sim");

// LWristYaw, LHand, RWristYaw and RHand don't exist in sim so we ignore.
static const std::map<int, int> index_nao_to_sim = {
  {nao_command_msgs::msg::JointIndexes::HEADYAW, he1},
  {nao_command_msgs::msg::JointIndexes::HEADPITCH, he2},
  {nao_command_msgs::msg::JointIndexes::LSHOULDERPITCH, lae1},
  {nao_command_msgs::msg::JointIndexes::LSHOULDERROLL, lae2},
  {nao_command_msgs::msg::JointIndexes::LELBOWYAW, lae3},
  {nao_command_msgs::msg::JointIndexes::LELBOWROLL, lae4},
  {nao_command_msgs::msg::JointIndexes::LHIPYAWPITCH, lle1},
  {nao_command_msgs::msg::JointIndexes::LHIPROLL, lle2},
  {nao_command_msgs::msg::JointIndexes::LHIPPITCH, lle3},
  {nao_command_msgs::msg::JointIndexes::LKNEEPITCH, lle4},
  {nao_command_msgs::msg::JointIndexes::LANKLEPITCH, lle5},
  {nao_command_msgs::msg::JointIndexes::LANKLEROLL, lle6},
  {nao_command_msgs::msg::JointIndexes::RHIPROLL, rle2},
  {nao_command_msgs::msg::JointIndexes::RHIPPITCH, rle3},
  {nao_command_msgs::msg::JointIndexes::RKNEEPITCH, rle4},
  {nao_command_msgs::msg::JointIndexes::RANKLEPITCH, rle5},
  {nao_command_msgs::msg::JointIndexes::RANKLEROLL, rle6},
  {nao_command_msgs::msg::JointIndexes::RSHOULDERPITCH, rae1},
  {nao_command_msgs::msg::JointIndexes::RSHOULDERROLL, rae2},
  {nao_command_msgs::msg::JointIndexes::RELBOWYAW, rae3},
  {nao_command_msgs::msg::JointIndexes::RELBOWROLL, rae4},
};

std::vector<int> nao_joints_to_invert_ = {
  nao_command_msgs::msg::JointIndexes::HEADPITCH,
  nao_command_msgs::msg::JointIndexes::LSHOULDERPITCH,
  nao_command_msgs::msg::JointIndexes::LHIPPITCH,
  nao_command_msgs::msg::JointIndexes::LKNEEPITCH,
  nao_command_msgs::msg::JointIndexes::LANKLEPITCH,
  nao_command_msgs::msg::JointIndexes::RHIPPITCH,
  nao_command_msgs::msg::JointIndexes::RKNEEPITCH,
  nao_command_msgs::msg::JointIndexes::RANKLEPITCH,
  nao_command_msgs::msg::JointIndexes::RSHOULDERPITCH};

SimJointCommands nao_to_sim(const nao_command_msgs::msg::JointPositions & nao_joints)
{
  SimJointCommands sim_joints;

  for (unsigned i = 0; i < nao_joints.indexes.size(); ++i) {
    unsigned nao_index = nao_joints.indexes.at(i);

    // Check index is valid
    if (nao_index >= nao_command_msgs::msg::JointIndexes::NUMJOINTS) {
      RCLCPP_ERROR(
        nao_to_sim_logger,
        "A value in indexes of nao_command_msgs::msg::JointPositions is not valid. Value: %d",
        nao_index);
      continue;
    }

    // Check whether this joint exists in simulation
    int sim_index;
    try {
      sim_index = index_nao_to_sim.at(nao_index);
    } catch (std::out_of_range &) {
      // This joint doesn't exist in simulation, ignore it
      continue;
    }

    float position = nao_joints.positions.at(i);

    if (std::find(
        nao_joints_to_invert_.begin(),
        nao_joints_to_invert_.end(), nao_index) != nao_joints_to_invert_.end())
    {
      position *= -1;
    }

    sim_joints.insert(std::make_pair(sim_index, position));

    if (nao_index == nao_command_msgs::msg::JointIndexes::LHIPYAWPITCH) {
      sim_joints.insert(std::make_pair(rle1, position));
    }
  }

  return sim_joints;
}
