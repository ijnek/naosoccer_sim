#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <nao_interfaces/msg/joint_positions.hpp>

class SimToNao : public rclcpp::Node
{
public:
    SimToNao();

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;
    rclcpp::Publisher<nao_interfaces::msg::JointPositions>::SharedPtr pub;

    void sim_to_nao(sensor_msgs::msg::JointState::UniquePtr joint_states);
};
