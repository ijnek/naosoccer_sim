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

#ifndef RCSS3D_NAO__SIM_TO_NAO_HPP_
#define RCSS3D_NAO__SIM_TO_NAO_HPP_

#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/accelerometer.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace rcss3d_nao
{
namespace sim_to_nao
{

nao_sensor_msgs::msg::JointPositions getJointPositions(
  const sensor_msgs::msg::JointState & sim_joints);

nao_sensor_msgs::msg::Accelerometer getAccelerometer(
  const sensor_msgs::msg::Imu & imu);

nao_sensor_msgs::msg::Gyroscope getGyroscope(
  const sensor_msgs::msg::Imu & imu);

}  // namespace sim_to_nao
}  // namespace rcss3d_nao


#endif  // RCSS3D_NAO__SIM_TO_NAO_HPP_
