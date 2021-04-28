#ifndef SEXP_PARSER_HPP
#define SEXP_PARSER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

#include "naosoccer_sim_interfaces/msg/joints.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"

class SexpParser
{
public:
    SexpParser(std::string msg) 
        : sexp(sexpresso::parse(msg)), 
          logger(rclcpp::get_logger("sexp_parser")){}
    naosoccer_sim_interfaces::msg::Joints getJoints();
    std::tuple<bool, nao_interfaces::msg::Accelerometer> getAccelerometer();
    std::tuple<bool, nao_interfaces::msg::Gyroscope> getGyroscope();

private:
    sexpresso::Sexp sexp;
    rclcpp::Logger logger;
};

#endif // SEXP_PARSER_HPP