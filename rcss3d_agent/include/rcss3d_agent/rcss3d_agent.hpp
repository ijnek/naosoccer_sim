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

#ifndef RCSS3D_AGENT__RCSS3D_AGENT_HPP_
#define RCSS3D_AGENT__RCSS3D_AGENT_HPP_

#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "nao_interfaces/msg/buttons.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/angle.hpp"
#include "nao_interfaces/msg/sonar.hpp"
#include "nao_interfaces/msg/fsr.hpp"
#include "nao_interfaces/msg/touch.hpp"
#include "rcss3d_agent/connection.hpp"
#include "rcss3d_agent/nao_joints_pid.hpp"
#include "rcss3d_agent/complementary_filter.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "soccer_vision_msgs/msg/goalpost_array.hpp"
#include "soccer_vision_msgs/msg/field_line_array.hpp"
#include "soccer_vision_msgs/msg/robot_array.hpp"
#include "soccer_vision_msgs/msg/flag_array.hpp"

class Rcss3dAgent : public rclcpp::Node
{
public:
  Rcss3dAgent();
  virtual ~Rcss3dAgent();

private:
  // Parameters (things that don't change)
  // - team number
  // - player number
  // - ip
  // - port
  // - starting position

  // Subscriptions
  // - JointCommand
  // - Beam (not for now)

  // Publishers
  // - Battery (not for sim)
  // - Buttons
  // - Joints
  //   - position
  //   - stiffness
  //   - temperature
  //   - current
  // - Accelerometer
  // - Gyroscope
  // - Angle
  // - Sonar
  // - FSR
  // - Touch

  rclcpp::Publisher<nao_interfaces::msg::Joints>::SharedPtr joints_pub;
  rclcpp::Publisher<nao_interfaces::msg::Buttons>::SharedPtr buttons_pub;
  rclcpp::Publisher<nao_interfaces::msg::Accelerometer>::SharedPtr accelerometer_pub;
  rclcpp::Publisher<nao_interfaces::msg::Gyroscope>::SharedPtr gyroscope_pub;
  rclcpp::Publisher<nao_interfaces::msg::Angle>::SharedPtr angle_pub;
  rclcpp::Publisher<nao_interfaces::msg::Sonar>::SharedPtr sonar_pub;
  rclcpp::Publisher<nao_interfaces::msg::FSR>::SharedPtr fsr_pub;
  rclcpp::Publisher<nao_interfaces::msg::Touch>::SharedPtr touch_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ball_pub;
  rclcpp::Publisher<soccer_vision_msgs::msg::GoalpostArray>::SharedPtr posts_pub;
  rclcpp::Publisher<soccer_vision_msgs::msg::FieldLineArray>::SharedPtr lines_pub;
  rclcpp::Publisher<soccer_vision_msgs::msg::RobotArray>::SharedPtr robots_pub;
  rclcpp::Publisher<soccer_vision_msgs::msg::FlagArray>::SharedPtr flags_pub;

  rclcpp::Subscription<nao_interfaces::msg::Joints>::SharedPtr joints_sub;

  Connection connection;

  NaoJointsPid naoJointsPid;
  ComplementaryFilter complementaryFilter;

  std::thread receive_thread_;
};

#endif  // RCSS3D_AGENT__RCSS3D_AGENT_HPP_
