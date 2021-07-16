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

#ifndef RCSS3D_AGENT__NAO_JOINTS_PID_HPP_
#define RCSS3D_AGENT__NAO_JOINTS_PID_HPP_

#include <vector>
#include "rcss3d_agent/joint_pid.hpp"
#include "rcss3d_agent/sim_joints.hpp"

class NaoJointsPid
{
public:
  NaoJointsPid();
  void setTarget(SimJointCommands target);

  SimJoints update(SimJoints nextJoints);

private:
  JointPid<float, NUM_SIM_JOINTS> jointPid;
  SimJoints target;
};

#endif  // RCSS3D_AGENT__NAO_JOINTS_PID_HPP_
