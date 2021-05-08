#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent/rcss3d_agent.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NaoSoccerSim>());
    rclcpp::shutdown();
    return 0;
}