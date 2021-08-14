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

#include <memory>
#include "rcss3d_nao/rcss3d_nao.hpp"
#include "rcss3d_nao/sim_to_nao.hpp"
#include "rcss3d_nao/nao_to_sim.hpp"

namespace rcss3d_nao
{

Rcss3DNao::Rcss3DNao(const rclcpp::NodeOptions & options)
: Node("rcss3d_nao", options)
{
  // Publishers
  joint_positions_pub = create_publisher<nao_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 10);
  accelerometer_pub = create_publisher<nao_sensor_msgs::msg::Accelerometer>(
    "sensors/accelerometer", 10);
  gyroscope_pub = create_publisher<nao_sensor_msgs::msg::Gyroscope>("sensors/gyroscope", 10);
  angle_pub = create_publisher<nao_sensor_msgs::msg::Angle>("sensors/angle", 10);

  joint_command_pub = create_publisher<rcss3d_controller_msgs::msg::JointPositionCommand>(
    "_joint_positions", 1);

  // Subscriptions
  joint_state_sub =
    create_subscription<sensor_msgs::msg::JointState>(
    "_joint_states",
    10,
    [this](sensor_msgs::msg::JointState::UniquePtr js) {
      RCLCPP_DEBUG(this->get_logger(), "Received _joint_states");
      joint_positions_pub->publish(sim_to_nao::getJointPositions(*js));
    });

  imu_sub =
    create_subscription<sensor_msgs::msg::Imu>(
    "_imu/data_raw",
    10,
    [this](sensor_msgs::msg::Imu::UniquePtr imuPtr) {
      RCLCPP_DEBUG(this->get_logger(), "Received _imu/data_raw");
      sensor_msgs::msg::Imu imu = *imuPtr;
      nao_sensor_msgs::msg::Accelerometer acc = sim_to_nao::getAccelerometer(imu);
      nao_sensor_msgs::msg::Gyroscope gyr = sim_to_nao::getGyroscope(imu);
      nao_sensor_msgs::msg::Angle ang = complementaryFilter.update(acc, gyr, imu.header.stamp);

      accelerometer_pub->publish(acc);
      gyroscope_pub->publish(gyr);
      angle_pub->publish(ang);
    });

  joint_command_positions_sub =
    create_subscription<nao_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 10,
    [this](nao_command_msgs::msg::JointPositions::UniquePtr cmd_nao) {
      RCLCPP_DEBUG(this->get_logger(), "Received effectors/joint_positions");
      auto cmd_sim = nao_to_sim::getJointPositionCommand(*cmd_nao);
      joint_command_pub->publish(
        std::make_unique<rcss3d_controller_msgs::msg::JointPositionCommand>(
          cmd_sim));
    });
}

}  // namespace rcss3d_nao
