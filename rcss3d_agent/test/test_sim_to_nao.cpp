#include <gtest/gtest.h>
#include "rcss3d_agent/sim_to_nao.hpp"

void test(
    std::vector<std::pair<std::string, float>> sim_joints,
    std::map<int, float> expected)
{
    nao_interfaces::msg::Joints converted = sim_to_nao(sim_joints);

    for (auto const& [key, val] : expected)
    {
        EXPECT_EQ(converted.angles.at(key), val);
    }
}

TEST(TestSimToNao, Test)
{
    std::vector<std::pair<std::string, float>> sim_joints = {
        {"hj1", -0.01},
        {"hj2", -0.02},
        {"laj1", 0.01},
        {"laj2", 0.02},
        {"laj3", 0.03},
        {"laj4", 0.04},
        {"llj1", 0.05},
        {"llj2", 0.06},
        {"llj3", 0.07},
        {"llj4", 0.08},
        {"llj5", 0.09},
        {"llj6", 0.10},
        {"rlj2", 0.11},
        {"rlj3", 0.12},
        {"rlj4", 0.13},
        {"rlj5", 0.14},
        {"rlj6", 0.15},
        {"raj1", 0.16},
        {"raj2", 0.17},
        {"raj3", 0.18},
        {"raj4", 0.19}};

    std::map<int, float> expected_nao_joint_positions = {
        {nao_interfaces::msg::Joints::HEADYAW, -0.01},
        {nao_interfaces::msg::Joints::HEADPITCH, 0.02},
        {nao_interfaces::msg::Joints::LSHOULDERPITCH, -0.01},
        {nao_interfaces::msg::Joints::LSHOULDERROLL, 0.02},
        {nao_interfaces::msg::Joints::LELBOWYAW, 0.03},
        {nao_interfaces::msg::Joints::LELBOWROLL, 0.04},
        {nao_interfaces::msg::Joints::LHIPYAWPITCH, 0.05},
        {nao_interfaces::msg::Joints::LHIPROLL, 0.06},
        {nao_interfaces::msg::Joints::LHIPPITCH, -0.07},
        {nao_interfaces::msg::Joints::LKNEEPITCH, -0.08},
        {nao_interfaces::msg::Joints::LANKLEPITCH, -0.09},
        {nao_interfaces::msg::Joints::LANKLEROLL, 0.10},
        {nao_interfaces::msg::Joints::RHIPROLL, 0.11},
        {nao_interfaces::msg::Joints::RHIPPITCH, -0.12},
        {nao_interfaces::msg::Joints::RKNEEPITCH, -0.13},
        {nao_interfaces::msg::Joints::RANKLEPITCH, -0.14},
        {nao_interfaces::msg::Joints::RANKLEROLL, 0.15},
        {nao_interfaces::msg::Joints::RSHOULDERPITCH, -0.16},
        {nao_interfaces::msg::Joints::RSHOULDERROLL, 0.17},
        {nao_interfaces::msg::Joints::RELBOWYAW, 0.18},
        {nao_interfaces::msg::Joints::RELBOWROLL, 0.19},

        // below should be 0 because they don't exist in simulation.
        {nao_interfaces::msg::Joints::LWRISTYAW, 0.0},
        {nao_interfaces::msg::Joints::RWRISTYAW, 0.0},
        {nao_interfaces::msg::Joints::LHAND, 0.0},
        {nao_interfaces::msg::Joints::RHAND, 0.0}};

    test(sim_joints, expected_nao_joint_positions);
}