#include "rclcpp/rclcpp.hpp"
#include "naosoccer_sim/nao_to_sim.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NaoToSim>());
    rclcpp::shutdown();
    return 0;
}