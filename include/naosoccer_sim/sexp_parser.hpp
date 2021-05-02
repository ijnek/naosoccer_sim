#ifndef SEXP_PARSER_HPP
#define SEXP_PARSER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

#include "nao_interfaces/msg/joints.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/fsr.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// See https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#forceresistance-perceptor
// to figure out which perceptors are available on each cycle, and which are not.
// Every cycle: joints, accelerometer, gyroscope, fsr
// Every third cycle: vision perceptors

class SexpParser
{
public:
    SexpParser(std::string msg) 
        : sexp(sexpresso::parse(msg)), 
          logger(rclcpp::get_logger("sexp_parser")){}
    std::vector<std::pair<std::string, float>> getJoints();
    nao_interfaces::msg::Accelerometer getAccelerometer();
    nao_interfaces::msg::Gyroscope getGyroscope();
    nao_interfaces::msg::FSR getFSR();
    std::tuple<bool, geometry_msgs::msg::PointStamped> getBall();

    // gamestate
    // observation
        // ball (geometry_msgs::PointStamped)
        // goal post
        // player (geometry_msgs::PoseStamped)
        // field feature (geometry_msgs::PoseStamped)
    // true information
        // my pos
        // my orien
        // ball pos

private:
    sexpresso::Sexp sexp;
    rclcpp::Logger logger;
};

#endif // SEXP_PARSER_HPP