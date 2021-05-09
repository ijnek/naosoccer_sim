// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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