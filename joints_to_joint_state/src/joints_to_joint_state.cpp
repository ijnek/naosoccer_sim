#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nao_interfaces/msg/joints.hpp"

std::vector<std::string> joint_names = {
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "LHand",
    "RHand",
};

class JointsToJointState : public rclcpp::Node
{
public:
    JointsToJointState() : Node("JointsToJointState")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        subscriber_ = this->create_subscription<nao_interfaces::msg::Joints>(
            "sensors/joints", 1,
            [this](nao_interfaces::msg::Joints::SharedPtr sensor_joints) {
                publisher_->publish(convert(*sensor_joints));
            });
    }

private:
    sensor_msgs::msg::JointState convert(nao_interfaces::msg::Joints &sensor_joints)
    {
        sensor_msgs::msg::JointState joint_states;
        joint_states.header.stamp = now();
        for (unsigned i = 0; i < nao_interfaces::msg::Joints::NUMJOINTS; ++i)
        {
            joint_states.name.push_back(joint_names[i]);
            joint_states.position.push_back(sensor_joints.angles[i]);
        }
        return joint_states;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<nao_interfaces::msg::Joints>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointsToJointState>());
    rclcpp::shutdown();
    return 0;
}