#include "rclcpp/rclcpp.hpp"

class Sim : public rclcpp::Node
{
public:
    Sim()
    : Node("sim")
    {
        RCLCPP_INFO(this->get_logger(), "sim.cpp Node initialised");
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sim>());
    rclcpp::shutdown();
    return 0;
}