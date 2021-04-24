#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <nao_interfaces/msg/joint_position_state.hpp>

using namespace std::chrono_literals;

class TestSimToNao : public ::testing::Test
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
            node->create_subscription<nao_interfaces::msg::JointPositionState>(
                "/joint_position_state", 1,
                [this](nao_interfaces::msg::JointPositionState::UniquePtr nao_joint_positions) {
                    this->nao_joint_positions = std::move(nao_joint_positions);
                    received = true;
                });

        publisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);

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
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::Subscription<nao_interfaces::msg::JointPositionState>::SharedPtr subscription;

    nao_interfaces::msg::JointPositionState::UniquePtr nao_joint_positions;
    bool received;

    void test(
        std::map<std::string, float> sim_joint_positions_to_send,
        std::map<std::string, float> expected_nao_joint_positions);
};

void TestSimToNao::test(
    std::map<std::string, float> sim_joint_positions_to_send,
    std::map<std::string, float> expected_nao_joint_positions)
{
    sensor_msgs::msg::JointState simJointsPositionsToSend;

    for (auto const &[key, val] : sim_joint_positions_to_send)
    {
        simJointsPositionsToSend.name.push_back(key);
        simJointsPositionsToSend.position.push_back(val);
    }

    while (!received)
    {
        publisher->publish(simJointsPositionsToSend);
        std::this_thread::sleep_for(1s);
        rclcpp::spin_some(node);
    }

    for (unsigned i = 0; i < nao_joint_positions->name.size(); ++i)
    {
        std::string name = nao_joint_positions->name[i];
        EXPECT_EQ(nao_joint_positions->position[i], expected_nao_joint_positions.at(name));
    }
}

TEST_F(TestSimToNao, Test)
{
    std::map<std::string, float> sim_joint_positions_to_send = {
        {"hj1", -0.01},
        {"hj2", -0.02},
        {"laj1", 0.01},
        {"laj2", 0.02},
        {"laj3", 0.03},
        {"laj4", 0.04},
        {"llj1", 0.05},
        {"llj2", 0.06},
        {"llj3", 0.07},
        {"llj4", 0.08},
        {"llj5", 0.09},
        {"llj6", 0.10},
        {"rlj2", 0.11},
        {"rlj3", 0.12},
        {"rlj4", 0.13},
        {"rlj5", 0.14},
        {"rlj6", 0.15},
        {"raj1", 0.16},
        {"raj2", 0.17},
        {"raj3", 0.18},
        {"raj4", 0.19}};

    std::map<std::string, float> expected_nao_joint_positions = {
        {"HeadYaw", -0.01},
        {"HeadPitch", 0.02},
        {"LShoulderPitch", -0.01},
        {"LShoulderRoll", 0.02},
        {"LElbowYaw", 0.03},
        {"LElbowRoll", 0.04},
        {"LHipYawPitch", 0.05},
        {"LHipRoll", 0.06},
        {"LHipPitch", -0.07},
        {"LKneePitch", -0.08},
        {"LAnklePitch", -0.09},
        {"LAnkleRoll", 0.10},
        {"RHipRoll", 0.11},
        {"RHipPitch", -0.12},
        {"RKneePitch", -0.13},
        {"RAnklePitch", -0.14},
        {"RAnkleRoll", 0.15},
        {"RShoulderPitch", -0.16},
        {"RShoulderRoll", 0.17},
        {"RElbowYaw", 0.18},
        {"RElbowRoll", 0.19}};

    test(sim_joint_positions_to_send, expected_nao_joint_positions);
}