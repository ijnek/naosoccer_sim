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

#include <string>
#include <utility>
#include <vector>
#include <tuple>
#include "rcss3d_agent/sexp_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent/fsr.hpp"

static constexpr double deg2rad(double rad) {return rad * 3.141592654 / 180.0;}

std::vector<std::pair<std::string, float>> SexpParser::getJoints()
{
  std::vector<std::pair<std::string, float>> joints;
  for (auto const & arg : sexp.arguments()) {
    // Joint expressions have form: (HJ (n llj2) (ax -0.00))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "HJ") {
      std::string name = s.at(1).value.sexp.at(1).value.str;
      float position = deg2rad(std::stod(s.at(2).value.sexp.at(1).value.str));
      joints.push_back(std::make_pair(name, position));
    }
  }
  return joints;
}

// eg. (ACC (n torso) (a 0.00 0.00 9.81))
nao_interfaces::msg::Accelerometer SexpParser::getAccelerometer()
{
  nao_interfaces::msg::Accelerometer accelerometerMsg;

  auto const * accSexp = sexp.getChildByPath("ACC/a");
  bool found = (accSexp != nullptr);
  if (found) {
    RCLCPP_DEBUG(logger, "Found accelerometer information");

    auto const & aSexp = accSexp->value.sexp;
    accelerometerMsg.x = std::stod(aSexp.at(2).value.str);
    accelerometerMsg.y = -std::stod(aSexp.at(1).value.str);
    accelerometerMsg.z = std::stod(aSexp.at(3).value.str);
  } else {
    RCLCPP_ERROR(logger, "No accelerometer information found");
  }

  return accelerometerMsg;
}

// eg. (GYR (n torso) (rt 0.01 0.07 0.46))
nao_interfaces::msg::Gyroscope SexpParser::getGyroscope()
{
  nao_interfaces::msg::Gyroscope gyroscopeMsg;

  auto const * gyrSexp = sexp.getChildByPath("GYR/rt");
  bool found = (gyrSexp != nullptr);
  if (found) {
    RCLCPP_DEBUG(logger, "Found gyroscope information");

    auto const & rateSexp = gyrSexp->value.sexp;
    gyroscopeMsg.x = deg2rad(std::stod(rateSexp.at(2).value.str));
    gyroscopeMsg.y = -deg2rad(std::stod(rateSexp.at(1).value.str));
    gyroscopeMsg.z = deg2rad(std::stod(rateSexp.at(3).value.str));
  } else {
    RCLCPP_ERROR(logger, "No gyroscope information found");
  }

  return gyroscopeMsg;
}


// Eg. (FRP (n lf) (c 0.00 -0.01 -0.01) (f -0.2 0.00 22.67))
//     (FRP (n rf) (c 0.00 -0.01 -0.01) (f -0.2 0.00 22.67))
nao_interfaces::msg::FSR SexpParser::getFSR()
{
  nao_interfaces::msg::FSR fsrMsg;

  for (unsigned i = 0; i < sexp.childCount(); ++i) {
    auto const & s = sexp.getChild(i).value.sexp;
    if (s.at(0).value.str == "FRP") {
      auto const & footSexp = s.at(1).value.sexp;
      FSR::Foot foot = (footSexp.at(1).value.str == "lf") ? FSR::Left : FSR::Right;

      auto const & coordSexp = s.at(2).value.sexp;
      float cx = std::stof(coordSexp.at(1).value.str);
      float cy = std::stof(coordSexp.at(2).value.str);
      // float cz = std::stof(coordSexp.at(3).value.str);

      auto const & forceSexp = s.at(3).value.sexp;
      float fx = std::stof(forceSexp.at(1).value.str);
      float fy = std::stof(forceSexp.at(2).value.str);
      float fz = std::stof(forceSexp.at(3).value.str);
      float totalForce = sqrt(fx * fx + fy * fy + fz * fz);

      float dists[FSR::NUM_LOCATION];
      float totalDist = 0;
      for (unsigned i = 0; i < FSR::NUM_LOCATION; ++i) {
        float offset_x = FSR::FSR_POS[foot][i][FSR::X];
        float offset_y = FSR::FSR_POS[foot][i][FSR::Y];
        float dist = sqrt(pow(cx - offset_x, 2) + pow(cy - offset_y, 2));
        dists[i] = dist;
        totalDist += dist;
      }

      if (foot == FSR::Left) {
        fsrMsg.l_foot_front_left = totalForce * dists[FSR::FrontLeft] / totalDist;
        fsrMsg.l_foot_front_right = totalForce * dists[FSR::FrontRight] / totalDist;
        fsrMsg.l_foot_back_left = totalForce * dists[FSR::BackLeft] / totalDist;
        fsrMsg.l_foot_back_right = totalForce * dists[FSR::BackRight] / totalDist;
      } else {
        fsrMsg.r_foot_front_left = totalForce * dists[FSR::FrontLeft] / totalDist;
        fsrMsg.r_foot_front_right = totalForce * dists[FSR::FrontRight] / totalDist;
        fsrMsg.r_foot_back_left = totalForce * dists[FSR::BackLeft] / totalDist;
        fsrMsg.r_foot_back_right = totalForce * dists[FSR::BackRight] / totalDist;
      }
    }
  }

  return fsrMsg;
}


// Eg. (See (B (pol 8.51 -0.21 -0.17)))
std::tuple<bool, geometry_msgs::msg::PointStamped> SexpParser::getBall()
{
  geometry_msgs::msg::PointStamped ballPoint;

  auto const * ballSexp = sexp.getChildByPath("See/B/pol");
  bool found = (ballSexp != nullptr);
  if (found) {
    RCLCPP_DEBUG(logger, "Found ball information");

    ballPoint.header.frame_id = "CameraTop_frame";
    ballPoint.point = polar_to_point(
      std::stof(ballSexp->value.sexp.at(1).value.str),
      deg2rad(std::stof(ballSexp->value.sexp.at(2).value.str)),
      deg2rad(std::stof(ballSexp->value.sexp.at(3).value.str)));
  }

  return std::make_tuple(found, ballPoint);
}


// Eg. (See (G2R (pol 17.55 -3.33 4.31))
//          (G1R (pol 17.52 3.27 4.07)))
std::tuple<bool, soccer_vision_msgs::msg::GoalpostArray> SexpParser::getGoalposts()
{
  soccer_vision_msgs::msg::GoalpostArray goalpostArray;

  for (std::string & postName :
    std::vector<std::string>{"G1L", "G1R", "G2L", "G2R"})
  {
    auto const * postSexp = sexp.getChildByPath("See/" + postName + "/pol");
    bool found = (postSexp != nullptr);
    if (found) {
      RCLCPP_DEBUG(logger, "Found post information");

      soccer_vision_msgs::msg::Goalpost post;
      post.header.frame_id = "CameraTop_frame";
      post.observed_top = true;
      post.point = polar_to_point(
        std::stof(postSexp->value.sexp.at(1).value.str),
        deg2rad(std::stof(postSexp->value.sexp.at(2).value.str)),
        deg2rad(std::stof(postSexp->value.sexp.at(3).value.str)));

      goalpostArray.posts.push_back(post);
    }
  }

  return std::make_tuple(goalpostArray.posts.size() > 0, goalpostArray);
}

// Eg. (See (L (pol 12.11 -40.77 -2.40) (pol 12.95 -37.76 -2.41))
//          (L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20)))
std::tuple<bool, soccer_vision_msgs::msg::FieldLineArray> SexpParser::getFieldLines()
{
  soccer_vision_msgs::msg::FieldLineArray fieldLineArray;

  auto const * seeSexp = sexp.getChildByPath("See");
  bool seeFound = (seeSexp != nullptr);
  if (seeFound) {
    for (auto const & arg : sexp.getChildByPath("See")->arguments()) {
      auto const & s = arg.value.sexp;
      if (s.at(0).value.str == "L") {
        soccer_vision_msgs::msg::FieldLine line;
        line.header.frame_id = "CameraTop_frame";

        line.start = polar_to_point(
          std::stof(s.at(1).value.sexp.at(1).value.str),
          deg2rad(std::stof(s.at(1).value.sexp.at(2).value.str)),
          deg2rad(std::stof(s.at(1).value.sexp.at(3).value.str)));

        line.end = polar_to_point(
          std::stof(s.at(2).value.sexp.at(1).value.str),
          deg2rad(std::stof(s.at(2).value.sexp.at(2).value.str)),
          deg2rad(std::stof(s.at(2).value.sexp.at(3).value.str)));

        fieldLineArray.lines.push_back(line);
      }
    }
  }

  return std::make_tuple(fieldLineArray.lines.size() > 0, fieldLineArray);
}

// Eg. (See (P (team teamRed) (id 1)
//             (head (pol 16.98 -0.21 3.19))
//             (rlowerarm (pol 16.83 -0.06 2.80))
//             (llowerarm (pol 16.86 -0.36 3.10))
//             (rfoot (pol 17.00 0.29 1.68))
//             (lfoot (pol 16.95 -0.51 1.32))))
std::tuple<bool, soccer_vision_msgs::msg::RobotArray> SexpParser::getRobots()
{
  soccer_vision_msgs::msg::RobotArray robotArray;

  auto * seeSexp = sexp.getChildByPath("See");
  bool seeFound = (seeSexp != nullptr);
  if (seeFound) {
    for (auto & arg : seeSexp->arguments()) {
      // Ignore if its not player info
      if (arg.value.sexp.at(0).value.str != "P") {
        continue;
      }

      // Ignore if we don't have head info
      auto * player_head = arg.getChildByPath("head");
      if (player_head == nullptr) {
        continue;
      }

      soccer_vision_msgs::msg::Robot robot;
      robot.header.frame_id = "CameraTop_frame";
      robot.team = arg.getChildByPath("team")->value.sexp.at(1).value.str;
      robot.id = std::stoi(arg.getChildByPath("id")->value.sexp.at(1).value.str);

      auto & pol = arg.getChildByPath("head/pol")->value.sexp;
      robot.head = polar_to_point(
        std::stof(pol.at(1).value.str),
        deg2rad(std::stof(pol.at(2).value.str)),
        deg2rad(std::stof(pol.at(3).value.str)));

      robotArray.robots.push_back(robot);
    }
  }

  return std::make_tuple(robotArray.robots.size() > 0, robotArray);
}


geometry_msgs::msg::Point SexpParser::polar_to_point(
  float distance, float horizontal_angle_phi, float vertical_angle_theta)
{
  geometry_msgs::msg::Point point;

  point.x = distance * cos(horizontal_angle_phi) * cos(vertical_angle_theta);
  point.y = distance * sin(horizontal_angle_phi) * cos(vertical_angle_theta);
  point.z = distance * sin(vertical_angle_theta);

  return point;
}
