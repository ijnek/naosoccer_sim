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

#ifndef NAO_TO_SIM_HPP
#define NAO_TO_SIM_HPP

#include "nao_interfaces/msg/joints.hpp"
#include "rcss3d_agent/sim_joints.hpp"
#include <map>

SimJoints nao_to_sim(const nao_interfaces::msg::Joints &nao_joints);

#endif // NAO_TO_SIM_HPP