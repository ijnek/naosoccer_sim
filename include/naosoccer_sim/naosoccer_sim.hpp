#ifndef NAOSOCCER_SIM
#define NAOSOCCER_SIM

#include "rclcpp/rclcpp.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "nao_interfaces/msg/buttons.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/angle.hpp"
#include "nao_interfaces/msg/sonar.hpp"
#include "nao_interfaces/msg/fsr.hpp"
#include "nao_interfaces/msg/touch.hpp"
#include "naosoccer_sim/connection.hpp"


class NaoSoccerSim : public rclcpp::Node
{
public:
    NaoSoccerSim();

private:
    // Parameters (things that don't change)
    // - team number
    // - player number
    // - ip
    // - port
    // - starting position

    // Subscriptions
    // - JointCommand
    // - Beam (not for now)

    // Publishers
    // - Battery (not for sim)
    // - Buttons
    // - Joints
    //   - position
    //   - stiffness
    //   - temperature
    //   - current
    // - Accelerometer
    // - Gyroscope
    // - Angle
    // - Sonar
    // - FSR
    // - Touch

    rclcpp::Publisher<nao_interfaces::msg::Joints>::SharedPtr joints_pub;
    rclcpp::Publisher<nao_interfaces::msg::Buttons>::SharedPtr buttons_pub;
    rclcpp::Publisher<nao_interfaces::msg::Accelerometer>::SharedPtr accelerometer_pub;
    rclcpp::Publisher<nao_interfaces::msg::Gyroscope>::SharedPtr gyroscope_pub;
    rclcpp::Publisher<nao_interfaces::msg::Angle>::SharedPtr angle_pub;
    rclcpp::Publisher<nao_interfaces::msg::Sonar>::SharedPtr sonar_pub;
    rclcpp::Publisher<nao_interfaces::msg::FSR>::SharedPtr fsr_pub;
    rclcpp::Publisher<nao_interfaces::msg::Touch>::SharedPtr touch_pub;

    Connection connection;
};

#endif // NAOSOCCER_SIM