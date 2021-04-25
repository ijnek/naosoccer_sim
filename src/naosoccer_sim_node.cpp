#include "rclcpp/rclcpp.hpp"
#include "naosoccer_sim/naosoccer_sim.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NaoSoccerSim>());
    rclcpp::shutdown();
    return 0;
}