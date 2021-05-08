#include <gtest/gtest.h>
#include "rcss3d_agent/nao_to_sim.hpp"

void test(
    nao_interfaces::msg::Joints nao_joints,
    std::map<SimJointIndex, float> expected_sim_joints)
{
    SimJoints converted = nao_to_sim(nao_joints);

    for (auto const &[sim_index, val] : expected_sim_joints)
    {
        EXPECT_EQ(converted.at(sim_index), val);
    }
}

nao_interfaces::msg::Joints createJointsMsg(std::map<int, float> nao_joints_map)
{
    nao_interfaces::msg::Joints naoJoints;

    for (auto const &[key, val] : nao_joints_map)
    {
        naoJoints.angles.at(key) = val;
    }
    return naoJoints;
}


TEST(TestJointsNaoToSim, Test)
{
    std::map<int, float> nao_joints_map = {
        {nao_interfaces::msg::Joints::HEADYAW, -0.01},
        {nao_interfaces::msg::Joints::HEADPITCH, 0},
        {nao_interfaces::msg::Joints::LSHOULDERPITCH, 0.01},
        {nao_interfaces::msg::Joints::LSHOULDERROLL, 0.02},
        {nao_interfaces::msg::Joints::LELBOWYAW, 0.03},
        {nao_interfaces::msg::Joints::LELBOWROLL, 0.04},
        {nao_interfaces::msg::Joints::LWRISTYAW, 0.05},
        {nao_interfaces::msg::Joints::LHIPYAWPITCH, 0.06},
        {nao_interfaces::msg::Joints::LHIPROLL, 0.07},
        {nao_interfaces::msg::Joints::LHIPPITCH, 0.08},
        {nao_interfaces::msg::Joints::LKNEEPITCH, 0.09},
        {nao_interfaces::msg::Joints::LANKLEPITCH, 0.10},
        {nao_interfaces::msg::Joints::LANKLEROLL, 0.11},
        {nao_interfaces::msg::Joints::RHIPROLL, 0.12},
        {nao_interfaces::msg::Joints::RHIPPITCH, 0.13},
        {nao_interfaces::msg::Joints::RKNEEPITCH, 0.14},
        {nao_interfaces::msg::Joints::RANKLEPITCH, 0.15},
        {nao_interfaces::msg::Joints::RANKLEROLL, 0.16},
        {nao_interfaces::msg::Joints::RSHOULDERPITCH, 0.17},
        {nao_interfaces::msg::Joints::RSHOULDERROLL, 0.18},
        {nao_interfaces::msg::Joints::RELBOWYAW, 0.19},
        {nao_interfaces::msg::Joints::RELBOWROLL, 0.20},
        {nao_interfaces::msg::Joints::RWRISTYAW, 0.21},
        {nao_interfaces::msg::Joints::LHAND, 0.22},
        {nao_interfaces::msg::Joints::RHAND, 0.23}};

    std::map<SimJointIndex, float> expected_sim_joints = {
        {he1, -0.01},
        {he2, 0},
        {lae1, -0.01},
        {lae2, 0.02},
        {lae3, 0.03},
        {lae4, 0.04},
        {lle1, 0.06},
        {lle2, 0.07},
        {lle3, -0.08},
        {lle4, -0.09},
        {lle5, -0.10},
        {lle6, 0.11},
        {rle1, 0.06},
        {rle2, 0.12},
        {rle3, -0.13},
        {rle4, -0.14},
        {rle5, -0.15},
        {rle6, 0.16},
        {rae1, -0.17},
        {rae2, 0.18},
        {rae3, 0.19},
        {rae4, 0.20},
    };

    nao_interfaces::msg::Joints nao_joints = createJointsMsg(nao_joints_map);
    test(nao_joints, expected_sim_joints);
}