#include "rclcpp/rclcpp.hpp"
#include "naosoccer_sim/sim_to_nao.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimToNao>());
    rclcpp::shutdown();
    return 0;
}