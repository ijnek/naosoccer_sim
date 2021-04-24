#ifndef NAO_TO_SIM_HPP
#define NAO_TO_SIM_HPP

#include "rclcpp/rclcpp.hpp"
#include <nao_interfaces/msg/joint_position_command.hpp>
#include <rcss3d_controller_msgs/msg/joint_position_command.hpp>
#include <map>

class NaoToSim : public rclcpp::Node
{
public:
    NaoToSim();

private:
    rclcpp::Subscription<nao_interfaces::msg::JointPositionCommand>::SharedPtr joint_pos_sub;
    rclcpp::Publisher<rcss3d_controller_msgs::msg::JointPositionCommand>::SharedPtr joint_pos_pub;

    rcss3d_controller_msgs::msg::JointPositionCommand::UniquePtr nao_to_sim(nao_interfaces::msg::JointPositionCommand::UniquePtr nao_joint_positions);
};

#endif // NAO_TO_SIM_HPP