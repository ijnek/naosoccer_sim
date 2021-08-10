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

#ifndef RCSS3D_NAO__RCSS3D_NAO_HPP_
#define RCSS3D_NAO__RCSS3D_NAO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/accelerometer.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "rcss3d_controller_msgs/msg/joint_position_command.hpp"

namespace rcss3d_nao
{

class Rcss3DNao : public rclcpp::Node
{
public:
  explicit Rcss3DNao(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<nao_sensor_msgs::msg::JointPositions>::SharedPtr joint_positions_pub;
  rclcpp::Publisher<nao_sensor_msgs::msg::Accelerometer>::SharedPtr accelerometer_pub;
  rclcpp::Publisher<nao_sensor_msgs::msg::Gyroscope>::SharedPtr gyroscope_pub;
  rclcpp::Publisher<nao_sensor_msgs::msg::Angle>::SharedPtr angle_pub;

  rclcpp::Publisher<rcss3d_controller_msgs::msg::JointPositionCommand>::SharedPtr joint_command_pub;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  rclcpp::Subscription<nao_command_msgs::msg::JointPositions>::SharedPtr
    joint_command_positions_sub;
};

}  // namespace rcss3d_nao

#endif  // RCSS3D_NAO__RCSS3D_NAO_HPP_
