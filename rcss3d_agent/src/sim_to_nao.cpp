#include "rcss3d_agent/sim_to_nao.hpp"

// Ignore rlj1 (RHipYawPitch) because its not an actual joint on the Nao
std::map<std::string, int> name_sim_to_nao = {
    {"hj1", nao_interfaces::msg::Joints::HEADYAW},
    {"hj2", nao_interfaces::msg::Joints::HEADPITCH},
    {"laj1", nao_interfaces::msg::Joints::LSHOULDERPITCH},
    {"laj2", nao_interfaces::msg::Joints::LSHOULDERROLL},
    {"laj3", nao_interfaces::msg::Joints::LELBOWYAW},
    {"laj4", nao_interfaces::msg::Joints::LELBOWROLL},
    {"llj1", nao_interfaces::msg::Joints::LHIPYAWPITCH},
    {"llj2", nao_interfaces::msg::Joints::LHIPROLL},
    {"llj3", nao_interfaces::msg::Joints::LHIPPITCH},
    {"llj4", nao_interfaces::msg::Joints::LKNEEPITCH},
    {"llj5", nao_interfaces::msg::Joints::LANKLEPITCH},
    {"llj6", nao_interfaces::msg::Joints::LANKLEROLL},
    {"rlj2", nao_interfaces::msg::Joints::RHIPROLL},
    {"rlj3", nao_interfaces::msg::Joints::RHIPPITCH},
    {"rlj4", nao_interfaces::msg::Joints::RKNEEPITCH},
    {"rlj5", nao_interfaces::msg::Joints::RANKLEPITCH},
    {"rlj6", nao_interfaces::msg::Joints::RANKLEROLL},
    {"raj1", nao_interfaces::msg::Joints::RSHOULDERPITCH},
    {"raj2", nao_interfaces::msg::Joints::RSHOULDERROLL},
    {"raj3", nao_interfaces::msg::Joints::RELBOWYAW},
    {"raj4", nao_interfaces::msg::Joints::RELBOWROLL}};

std::vector<int> nao_joints_to_invert = {
    nao_interfaces::msg::Joints::HEADPITCH,
    nao_interfaces::msg::Joints::LSHOULDERPITCH,
    nao_interfaces::msg::Joints::LHIPPITCH,
    nao_interfaces::msg::Joints::LKNEEPITCH,
    nao_interfaces::msg::Joints::LANKLEPITCH,
    nao_interfaces::msg::Joints::RHIPPITCH,
    nao_interfaces::msg::Joints::RKNEEPITCH,
    nao_interfaces::msg::Joints::RANKLEPITCH,
    nao_interfaces::msg::Joints::RSHOULDERPITCH};

nao_interfaces::msg::Joints sim_to_nao(const std::vector<std::pair<std::string, float>> &sim_joints)
{
    nao_interfaces::msg::Joints nao_joints = nao_interfaces::msg::Joints{};

    for (auto joint : sim_joints)
    {
        auto sim_joint_name = joint.first;
        auto it = name_sim_to_nao.find(sim_joint_name);

        if (it != name_sim_to_nao.end())
        {
            float joint_index = it->second;

            auto sim_joint_position = joint.second;

            if (std::find(nao_joints_to_invert.begin(), nao_joints_to_invert.end(), joint_index) != nao_joints_to_invert.end())
            {
                sim_joint_position *= -1;
            }

            nao_joints.angles[joint_index] = sim_joint_position;
        }
    }
    return nao_joints;
}
