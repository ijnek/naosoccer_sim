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

#include "rcss3d_agent/nao_joints_pid.hpp"

#define P 28.0
#define I 0.0
#define D 0.0

NaoJointsPid::NaoJointsPid()
: jointPid(P, I, D)
{}

void NaoJointsPid::setTarget(SimJoints target)
{
  this->target = target;
}

SimJoints NaoJointsPid::update(SimJoints current)
{
  SimJoints out = jointPid.update(current, target);
  return out;
}
