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

#include "rcss3d_agent/sim_joints.hpp"

SimJoints toSimJoints(std::vector<std::pair<std::string, float>> in)
{
    SimJoints simJoints;

    for (auto const &[name, speed] : in)
    {
        auto itr = std::find(jointPerceptorNames.begin(), jointPerceptorNames.end(), name);
        
        if (itr != jointPerceptorNames.end())
        {
            int idx = itr - jointPerceptorNames.begin();
            simJoints.at(idx) = speed;
        }
    }
    return simJoints;
}

std::vector<std::pair<std::string, float>> fromSimJoints(SimJoints &in)
{
    std::vector<std::pair<std::string, float>> out;

    for (unsigned i = 0 ; i < NUM_SIM_JOINTS; ++i)
    {
        out.push_back(std::make_pair(jointNames.at(i), in.at(i)));
    }

    return out;
}
