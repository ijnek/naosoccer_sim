#include "rcss3d_agent/nao_to_sim.hpp"
#include "rcss3d_agent/sim_joints.hpp"

// LWristYaw, LHand, RWristYaw and RHand don't exist in sim so we ignore.
static const std::map<int, int> index_nao_to_sim = {
    {nao_interfaces::msg::Joints::HEADYAW, he1},
    {nao_interfaces::msg::Joints::HEADPITCH, he2},
    {nao_interfaces::msg::Joints::LSHOULDERPITCH, lae1},
    {nao_interfaces::msg::Joints::LSHOULDERROLL, lae2},
    {nao_interfaces::msg::Joints::LELBOWYAW, lae3},
    {nao_interfaces::msg::Joints::LELBOWROLL, lae4},
    {nao_interfaces::msg::Joints::LHIPYAWPITCH, lle1},
    {nao_interfaces::msg::Joints::LHIPROLL, lle2},
    {nao_interfaces::msg::Joints::LHIPPITCH, lle3},
    {nao_interfaces::msg::Joints::LKNEEPITCH, lle4},
    {nao_interfaces::msg::Joints::LANKLEPITCH, lle5},
    {nao_interfaces::msg::Joints::LANKLEROLL, lle6},
    {nao_interfaces::msg::Joints::RHIPROLL, rle2},
    {nao_interfaces::msg::Joints::RHIPPITCH, rle3},
    {nao_interfaces::msg::Joints::RKNEEPITCH, rle4},
    {nao_interfaces::msg::Joints::RANKLEPITCH, rle5},
    {nao_interfaces::msg::Joints::RANKLEROLL, rle6},
    {nao_interfaces::msg::Joints::RSHOULDERPITCH, rae1},
    {nao_interfaces::msg::Joints::RSHOULDERROLL, rae2},
    {nao_interfaces::msg::Joints::RELBOWYAW, rae3},
    {nao_interfaces::msg::Joints::RELBOWROLL, rae4},
};

std::vector<int> nao_joints_to_invert_ = {
    nao_interfaces::msg::Joints::HEADPITCH,
    nao_interfaces::msg::Joints::LSHOULDERPITCH,
    nao_interfaces::msg::Joints::LHIPPITCH,
    nao_interfaces::msg::Joints::LKNEEPITCH,
    nao_interfaces::msg::Joints::LANKLEPITCH,
    nao_interfaces::msg::Joints::RHIPPITCH,
    nao_interfaces::msg::Joints::RKNEEPITCH,
    nao_interfaces::msg::Joints::RANKLEPITCH,
    nao_interfaces::msg::Joints::RSHOULDERPITCH};

SimJoints nao_to_sim(const nao_interfaces::msg::Joints &nao_joints)
{
    SimJoints sim_joints;

    for (auto const &[nao_index, sim_index] : index_nao_to_sim)
    {
        float angle = nao_joints.angles.at(nao_index);

        if (std::find(nao_joints_to_invert_.begin(), nao_joints_to_invert_.end(), nao_index) != nao_joints_to_invert_.end())
        {
            angle *= -1;
        }

        sim_joints.at(sim_index) = angle;

        if (nao_index == nao_interfaces::msg::Joints::LHIPYAWPITCH)
        {
            sim_joints.at(rle1) = angle;
        }
    }
    return sim_joints;
}