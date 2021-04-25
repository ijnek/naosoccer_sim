#include "naosoccer_sim/naosoccer_sim.hpp"

NaoSoccerSim::NaoSoccerSim()
: Node("NaoSoccerSim")
{
    RCLCPP_DEBUG(get_logger(), "Declare parameters");
    this->declare_parameter<std::string>("host", "127.0.0.1");
    this->declare_parameter<int>("port", 3100);
    this->declare_parameter<std::string>("team", "Anonymous");
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
}