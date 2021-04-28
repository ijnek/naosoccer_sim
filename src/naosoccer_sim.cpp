#include "naosoccer_sim/naosoccer_sim.hpp"
#include "rclcpp/rclcpp.hpp"
#include "naosoccer_sim/socket.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "naosoccer_sim/sexp_creator.hpp"

NaoSoccerSim::NaoSoccerSim()
    : Node("NaoSoccerSim")
{
    RCLCPP_DEBUG(get_logger(), "Declare parameters");
    this->declare_parameter<std::string>("host", "127.0.0.1");
    this->declare_parameter<int>("port", 3100);
    this->declare_parameter<std::string>("team_name", "Anonymous");
    this->declare_parameter<int>("player_number", 0);
    this->declare_parameter("starting_pose", std::vector<double>{0, 0, 0});

    RCLCPP_DEBUG(get_logger(), "Initialise publishers");
    joints_pub = create_publisher<nao_interfaces::msg::Joints>("/sensors/joints", 10);
    buttons_pub = create_publisher<nao_interfaces::msg::Buttons>("/sensors/buttons", 10);
    accelerometer_pub = create_publisher<nao_interfaces::msg::Accelerometer>("/sensors/accelerometer", 10);
    gyroscope_pub = create_publisher<nao_interfaces::msg::Gyroscope>("/sensors/gyroscope", 10);
    angle_pub = create_publisher<nao_interfaces::msg::Angle>("/sensors/angle", 10);
    sonar_pub = create_publisher<nao_interfaces::msg::Sonar>("/sensors/sonar", 10);
    fsr_pub = create_publisher<nao_interfaces::msg::FSR>("/sensors/fsr", 10);
    touch_pub = create_publisher<nao_interfaces::msg::Touch>("/sensors/touch", 10);

    RCLCPP_DEBUG(get_logger(), "Initialise subscriptions");
    jsc_sub =
        create_subscription<naosoccer_sim_interfaces::msg::JointSpeedCommand>(
            "/joint_speed_commands", 10,
            [this](naosoccer_sim_interfaces::msg::JointSpeedCommand::SharedPtr cmd) {
                std::string joint_message = SexpCreator::createJointMessage(cmd);
                RCLCPP_DEBUG(this->get_logger(), "Sending: " + joint_message);
                connection.send(joint_message);
            });

    // Initialise connection
    connection.initialise(
        get_parameter("host").as_string(),
        get_parameter("port").as_int());

    // Create the robot
    connection.send(SexpCreator::createCreateMessage());

    // Receive, this is needed for the init message to be sent next
    connection.receive();

    // Send init
    connection.send(SexpCreator::createInitMessage(
        get_parameter("team_name").as_string(), get_parameter("player_number").as_int()));

    // Start receive loop
    while (true)
    {
        std::string recv = connection.receive();
        RCLCPP_DEBUG(this->get_logger(), "Received: " + recv);
    }
}
