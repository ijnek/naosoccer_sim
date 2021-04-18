#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <nao_interfaces/msg/joint_position_state.hpp>

class SimToNao : public rclcpp::Node
{
public:
    SimToNao()
        : Node("SimToNao")
    {
        pub = create_publisher<nao_interfaces::msg::JointPositionState>("/joint_position_state", 1);

        sub = 
            create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 1,
                [this](sensor_msgs::msg::JointState::UniquePtr joint_states) {
                    sim_to_nao(std::move(joint_states));
                });
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;
    rclcpp::Publisher<nao_interfaces::msg::JointPositionState>::SharedPtr pub;

    void sim_to_nao(sensor_msgs::msg::JointState::UniquePtr joint_states)
    {
        auto jointPositionState = std::make_unique<nao_interfaces::msg::JointPositionState>();
        
        for (auto i = 0u; i < joint_states->name.size(); ++i)
        {
            auto sim_joint_name = joint_states->name[i];

            std::map<std::string, std::string>::iterator it = name_sim_to_nao.find(sim_joint_name);
            if (it != name_sim_to_nao.end())
            {
                auto nao_joint_name = it->second;
                
                jointPositionState->name.push_back(nao_joint_name);

                auto sim_joint_position = joint_states->position[i];
                
                if (std::find(nao_joints_to_invert.begin(), nao_joints_to_invert.end(), nao_joint_name) != nao_joints_to_invert.end())
                {
                    sim_joint_position *= -1;
                }

                jointPositionState->position.push_back(sim_joint_position);
            }
        }

        pub->publish(std::move(jointPositionState));

    }

    // Ignore rlj1 (RHipYawPitch) because its not an actual joint on the Nao
    std::map<std::string, std::string> name_sim_to_nao = {
        {"hj1", "HeadYaw"},
        {"hj2", "HeadPitch"},
        {"laj1", "LShoulderPitch"},
        {"laj2", "LShoulderRoll"},
        {"laj3", "LElbowYaw"},
        {"laj4", "LElbowRoll"},
        {"llj1", "LHipYawPitch"},
        {"llj2", "LHipRoll"},
        {"llj3", "LHipPitch"},
        {"llj4", "LKneePitch"},
        {"llj5", "LAnklePitch"},
        {"llj6", "LAnkleRoll"},
        {"rlj2", "RHipRoll"},
        {"rlj3", "RHipPitch"},
        {"rlj4", "RKneePitch"},
        {"rlj5", "RAnklePitch"},
        {"rlj6", "RAnkleRoll"},
        {"raj1", "RShoulderPitch"},
        {"raj2", "RShoulderRoll"},
        {"raj3", "RElbowYaw"},
        {"raj4", "RElbowRoll"}
    };

    std::vector<std::string> nao_joints_to_invert = {
        "HeadPitch",
        "LShoulderPitch",
        "LHipPitch",
        "LKneePitch",
        "LAnklePitch",
        "RHipPitch",
        "RKneePitch",
        "RAnklePitch",
        "RShoulderPitch"
    };
};

int
main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimToNao>());
    rclcpp::shutdown();
    return 0;
}