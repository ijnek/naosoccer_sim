#include "rclcpp/rclcpp.hpp"
#include <nao_interfaces/msg/joint_position_command.hpp>
#include <rcss3d_controller_msgs/msg/joint_position_command.hpp>

class Sim : public rclcpp::Node
{
public:
    Sim()
        : Node("sim")
    {
        joint_pos_pub = create_publisher<rcss3d_controller_msgs::msg::JointPositionCommand>("/joint_positions", 1);

        joint_pos_sub =
            create_subscription<nao_interfaces::msg::JointPositionCommand>("/nao_joint_positions", 1,
                [this](nao_interfaces::msg::JointPositionCommand::UniquePtr nao_joint_positions) {
                    auto rcsJointCommands = std::make_unique<rcss3d_controller_msgs::msg::JointPositionCommand>();
                    for (auto i = 0u; i < nao_joint_positions->name.size(); ++i)
                    {
                        auto joint_name = nao_joint_positions->name[i];
                        RCLCPP_INFO(this->get_logger(), joint_name);
                    }
                });
    }

private:
    rclcpp::Subscription<nao_interfaces::msg::JointPositionCommand>::SharedPtr joint_pos_sub;
    rclcpp::Publisher<rcss3d_controller_msgs::msg::JointPositionCommand>::SharedPtr joint_pos_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim>());
    rclcpp::shutdown();
    return 0;
}