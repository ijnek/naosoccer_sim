#include "naosoccer_sim/nao_to_sim.hpp"
#include <string>

#define P_GAIN 28

// LWristYaw, LHand, RWristYaw and RHand don't exist in sim so we ignore.
static const std::map<std::string, std::string> name_nao_to_sim = {
    {"HeadYaw", "he1"},
    {"HeadPitch", "he2"},
    {"LShoulderPitch", "lae1"},
    {"LShoulderRoll", "lae2"},
    {"LElbowYaw", "lae3"},
    {"LElbowRoll", "lae4"},
    {"LHipYawPitch", "lle1"},
    {"LHipRoll", "lle2"},
    {"LHipPitch", "lle3"},
    {"LKneePitch", "lle4"},
    {"LAnklePitch", "lle5"},
    {"LAnkleRoll", "lle6"},
    {"RHipRoll", "rle2"},
    {"RHipPitch", "rle3"},
    {"RKneePitch", "rle4"},
    {"RAnklePitch", "rle5"},
    {"RAnkleRoll", "rle6"},
    {"RShoulderPitch", "rae1"},
    {"RShoulderRoll", "rae2"},
    {"RElbowYaw", "rae3"},
    {"RElbowRoll", "rae4"},
};

static const std::vector<std::string> nao_joints_to_invert = {
    "HeadPitch",
    "LShoulderPitch",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RShoulderPitch"};

NaoToSim::NaoToSim()
    : Node("NaoToSim")
{
    joint_pos_pub = create_publisher<rcss3d_controller_msgs::msg::JointPositionCommand>("/joint_positions", 1);

    joint_pos_sub =
        create_subscription<nao_interfaces::msg::JointCommands>(
            "/joint_commands", 1,
            [this](nao_interfaces::msg::JointCommands::UniquePtr nao_joint_commands) {
                auto sim = nao_to_sim(std::move(nao_joint_commands));
                joint_pos_pub->publish(std::move(sim));
            });
}

rcss3d_controller_msgs::msg::JointPositionCommand::UniquePtr NaoToSim::nao_to_sim(nao_interfaces::msg::JointCommands::UniquePtr nao_joint_commands)
{
    auto rcsJointCommands = std::make_unique<rcss3d_controller_msgs::msg::JointPositionCommand>();
    for (auto i = 0u; i < nao_joint_commands->name.size(); ++i)
    {
        auto nao_joint_name = nao_joint_commands->name[i];

        auto it = name_nao_to_sim.find(nao_joint_name);
        if (it != name_nao_to_sim.end())
        {
            rcsJointCommands->name.push_back(it->second);

            auto nao_joint_position = nao_joint_commands->position[i];

            if (std::find(nao_joints_to_invert.begin(), nao_joints_to_invert.end(), nao_joint_name) != nao_joints_to_invert.end())
            {
                nao_joint_position *= -1;
            }

            rcsJointCommands->position.push_back(nao_joint_position);
            rcsJointCommands->p_gain.push_back(P_GAIN);

            // If LHipYawPitch, copy joint request for rle1 (RHipYawPitch)
            if (nao_joint_name == "LHipYawPitch")
            {
                rcsJointCommands->name.push_back("rle1");
                rcsJointCommands->position.push_back(nao_joint_position);
                rcsJointCommands->p_gain.push_back(P_GAIN);
            }
        }
        else
        {
            // RCLCPP_ERROR(this->get_logger(), nao_joint_name + std::string(" isn't a valid joint to command, ignoring."));
        }
    }

    return rcsJointCommands;
}
