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

#ifndef RCSS3D_NAO__NAO_TO_SIM_HPP_
#define RCSS3D_NAO__NAO_TO_SIM_HPP_

#include <map>
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "rcss3d_controller_msgs/msg/joint_position_command.hpp"

namespace rcss3d_nao
{
namespace nao_to_sim
{

rcss3d_controller_msgs::msg::JointPositionCommand getJointPositionCommand(
  const nao_command_msgs::msg::JointPositions & nao_joints);

}  // namespace nao_to_sim
}  // namespace rcss3d_nao

#endif  // RCSS3D_NAO__NAO_TO_SIM_HPP_
