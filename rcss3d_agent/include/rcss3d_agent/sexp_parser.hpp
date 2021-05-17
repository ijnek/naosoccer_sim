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

#ifndef RCSS3D_AGENT__SEXP_PARSER_HPP_
#define RCSS3D_AGENT__SEXP_PARSER_HPP_

#include <string>
#include <utility>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

#include "nao_interfaces/msg/joints.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/fsr.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "naosoccer_interfaces/msg/goalpost_array.hpp"
#include "naosoccer_interfaces/msg/field_line_array.hpp"
#include "naosoccer_interfaces/msg/robot_array.hpp"

// See https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#forceresistance-perceptor
// to figure out which perceptors are available on each cycle, and which are not.
// Every cycle: joints, accelerometer, gyroscope, fsr
// Every third cycle: vision perceptors

class SexpParser
{
public:
  explicit SexpParser(std::string msg)
  : sexp(sexpresso::parse(msg)),
    logger(rclcpp::get_logger("sexp_parser")) {}
  std::vector<std::pair<std::string, float>> getJoints();
  nao_interfaces::msg::Accelerometer getAccelerometer();
  nao_interfaces::msg::Gyroscope getGyroscope();
  nao_interfaces::msg::FSR getFSR();
  std::tuple<bool, geometry_msgs::msg::PointStamped> getBall();
  std::tuple<bool, naosoccer_interfaces::msg::GoalpostArray> getGoalposts();
  std::tuple<bool, naosoccer_interfaces::msg::FieldLineArray> getFieldLines();
  std::tuple<bool, naosoccer_interfaces::msg::RobotArray> getRobots();

  // gamestate
  // observation
  // field feature (geometry_msgs::PoseStamped)
  // true information
  // my pos
  // my orien
  // ball pos

private:
  sexpresso::Sexp sexp;
  rclcpp::Logger logger;
  geometry_msgs::msg::Point polar_to_point(
    float distance, float horizontal_angle_phi, float vertical_angle_theta);
};

#endif  // RCSS3D_AGENT__SEXP_PARSER_HPP_
