#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "nao_interfaces/msg/joint_commands.hpp"
#include "rcss3d_controller_msgs/msg/joint_position_command.hpp"

using namespace std::chrono_literals;

class TestNaoToSim : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestCase()
    {
        rclcpp::shutdown();
    }

    void SetUp()
    {
        node = std::make_shared<rclcpp::Node>("node");

        subscription =
            node->create_subscription<rcss3d_controller_msgs::msg::JointPositionCommand>(
                "/joint_positions", 1,
                [this](rcss3d_controller_msgs::msg::JointPositionCommand::UniquePtr sim_joint_positions) {
                    this->sim_joint_positions = std::move(sim_joint_positions);
                    received = true;
                });

        publisher = node->create_publisher<nao_interfaces::msg::JointCommands>("/joint_commands", 1);

        received = false;
    }

    void
    TearDown()
    {
        publisher.reset();
        subscription.reset();
        node.reset();
    }

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<nao_interfaces::msg::JointCommands>::SharedPtr publisher;
    rclcpp::Subscription<rcss3d_controller_msgs::msg::JointPositionCommand>::SharedPtr subscription;

    rcss3d_controller_msgs::msg::JointPositionCommand::UniquePtr sim_joint_positions;
    bool received;

    void test(
        std::map<std::string, float> nao_joint_commands_to_send,
        std::map<std::string, float> expected_sim_joint_positions);
};

void TestNaoToSim::test(
    std::map<std::string, float> nao_joint_commands_to_send,
    std::map<std::string, float> expected_sim_joint_positions)
{

    nao_interfaces::msg::JointCommands naoJointCommandsToSend;

    for (auto const &[key, val] : nao_joint_commands_to_send)
    {
        naoJointCommandsToSend.name.push_back(key);
        naoJointCommandsToSend.position.push_back(val);
    }

    while (!received)
    {
        publisher->publish(naoJointCommandsToSend);
        std::this_thread::sleep_for(1s);
        rclcpp::spin_some(node);
    }

    for (unsigned i = 0; i < sim_joint_positions->name.size(); ++i)
    {
        std::string name = sim_joint_positions->name[i];
        EXPECT_EQ(sim_joint_positions->position[i], expected_sim_joint_positions.at(name));
    }
}

TEST_F(TestNaoToSim, Test)
{
    std::map<std::string, float> nao_joint_commands_to_send = {
        {"HeadYaw", -0.01},
        {"HeadPitch", 0},
        {"LShoulderPitch", 0.01},
        {"LShoulderRoll", 0.02},
        {"LElbowYaw", 0.03},
        {"LElbowRoll", 0.04},
        {"LWristYaw", 0.05},
        {"LHipYawPitch", 0.06},
        {"LHipRoll", 0.07},
        {"LHipPitch", 0.08},
        {"LKneePitch", 0.09},
        {"LAnklePitch", 0.10},
        {"LAnkleRoll", 0.11},
        {"RHipRoll", 0.12},
        {"RHipPitch", 0.13},
        {"RKneePitch", 0.14},
        {"RAnklePitch", 0.15},
        {"RAnkleRoll", 0.16},
        {"RShoulderPitch", 0.17},
        {"RShoulderRoll", 0.18},
        {"RElbowYaw", 0.19},
        {"RElbowRoll", 0.20},
        {"RWristYaw", 0.21},
        {"LHand", 0.22},
        {"RHand", 0.23}};

    std::map<std::string, float> expected_sim_joint_positions = {
        {"he1", -0.01},
        {"he2", 0},
        {"lae1", -0.01},
        {"lae2", 0.02},
        {"lae3", 0.03},
        {"lae4", 0.04},
        {"lle1", 0.06},
        {"lle2", 0.07},
        {"lle3", -0.08},
        {"lle4", -0.09},
        {"lle5", -0.10},
        {"lle6", 0.11},
        {"rle1", 0.06},
        {"rle2", 0.12},
        {"rle3", -0.13},
        {"rle4", -0.14},
        {"rle5", -0.15},
        {"rle6", 0.16},
        {"rae1", -0.17},
        {"rae2", 0.18},
        {"rae3", 0.19},
        {"rae4", 0.20},
    };

    test(nao_joint_commands_to_send, expected_sim_joint_positions);
}
