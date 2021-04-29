#include "naosoccer_sim/sim_joints.hpp"

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
