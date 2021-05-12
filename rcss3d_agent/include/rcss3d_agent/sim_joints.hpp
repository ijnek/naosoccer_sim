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

#ifndef RCSS3D_AGENT__SIM_JOINTS_HPP_
#define RCSS3D_AGENT__SIM_JOINTS_HPP_

#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <utility>

enum SimJointIndex
{
  he1,
  he2,
  lae1,
  lae2,
  lae3,
  lae4,
  lle1,
  lle2,
  lle3,
  lle4,
  lle5,
  lle6,
  rle1,
  rle2,
  rle3,
  rle4,
  rle5,
  rle6,
  rae1,
  rae2,
  rae3,
  rae4,
  NUM_SIM_JOINTS
};

const std::array<std::string, NUM_SIM_JOINTS> jointNames = {
  "he1",
  "he2",
  "lae1",
  "lae2",
  "lae3",
  "lae4",
  "lle1",
  "lle2",
  "lle3",
  "lle4",
  "lle5",
  "lle6",
  "rle1",
  "rle2",
  "rle3",
  "rle4",
  "rle5",
  "rle6",
  "rae1",
  "rae2",
  "rae3",
  "rae4",
};

const std::array<std::string, NUM_SIM_JOINTS> jointPerceptorNames = {
  "hj1",
  "hj2",
  "laj1",
  "laj2",
  "laj3",
  "laj4",
  "llj1",
  "llj2",
  "llj3",
  "llj4",
  "llj5",
  "llj6",
  "rlj1",
  "rlj2",
  "rlj3",
  "rlj4",
  "rlj5",
  "rlj6",
  "raj1",
  "raj2",
  "raj3",
  "raj4",
};

typedef std::array<float, NUM_SIM_JOINTS> SimJoints;

SimJoints toSimJoints(std::vector<std::pair<std::string, float>> in);
std::vector<std::pair<std::string, float>> fromSimJoints(SimJoints & in);

#endif  // RCSS3D_AGENT__SIM_JOINTS_HPP_
