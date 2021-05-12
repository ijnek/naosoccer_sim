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

#ifndef RCSS3D_AGENT__SIM_TO_NAO_HPP_
#define RCSS3D_AGENT__SIM_TO_NAO_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>
#include "nao_interfaces/msg/joints.hpp"

nao_interfaces::msg::Joints sim_to_nao(
  const std::vector<std::pair<std::string, float>> & sim_joints);

#endif  // RCSS3D_AGENT__SIM_TO_NAO_HPP_
