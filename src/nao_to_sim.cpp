#include "naosoccer_sim/nao_to_sim.hpp"

// LWristYaw, LHand, RWristYaw and RHand don't exist in sim so we ignore.
static const std::map<int, std::string> name_nao_to_sim = {
    {nao_interfaces::msg::Joints::HEADYAW, "he1"},
    {nao_interfaces::msg::Joints::HEADPITCH, "he2"},
    {nao_interfaces::msg::Joints::LSHOULDERPITCH, "lae1"},
    {nao_interfaces::msg::Joints::LSHOULDERROLL, "lae2"},
    {nao_interfaces::msg::Joints::LELBOWYAW, "lae3"},
    {nao_interfaces::msg::Joints::LELBOWROLL, "lae4"},
    {nao_interfaces::msg::Joints::LHIPYAWPITCH, "lle1"},
    {nao_interfaces::msg::Joints::LHIPROLL, "lle2"},
    {nao_interfaces::msg::Joints::LHIPPITCH, "lle3"},
    {nao_interfaces::msg::Joints::LKNEEPITCH, "lle4"},
    {nao_interfaces::msg::Joints::LANKLEPITCH, "lle5"},
    {nao_interfaces::msg::Joints::LANKLEROLL, "lle6"},
    {nao_interfaces::msg::Joints::RHIPROLL, "rle2"},
    {nao_interfaces::msg::Joints::RHIPPITCH, "rle3"},
    {nao_interfaces::msg::Joints::RKNEEPITCH, "rle4"},
    {nao_interfaces::msg::Joints::RANKLEPITCH, "rle5"},
    {nao_interfaces::msg::Joints::RANKLEROLL, "rle6"},
    {nao_interfaces::msg::Joints::RSHOULDERPITCH, "rae1"},
    {nao_interfaces::msg::Joints::RSHOULDERROLL, "rae2"},
    {nao_interfaces::msg::Joints::RELBOWYAW, "rae3"},
    {nao_interfaces::msg::Joints::RELBOWROLL, "rae4"},
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

std::vector<std::pair<std::string, float>> nao_to_sim(const nao_interfaces::msg::Joints &nao_joints)
{
    std::vector<std::pair<std::string, float>> sim_joints;

    for (auto const &[key, name] : name_nao_to_sim)
    {
        float angle = nao_joints.angles[key];

        if (std::find(nao_joints_to_invert_.begin(), nao_joints_to_invert_.end(), key) != nao_joints_to_invert_.end())
        {
            angle *= -1;
        }

        sim_joints.push_back(std::make_pair(name, angle));

        if (key == nao_interfaces::msg::Joints::LHIPYAWPITCH)
        {
            sim_joints.push_back(std::make_pair("rle1", angle));
        }
    }
    return sim_joints;
}