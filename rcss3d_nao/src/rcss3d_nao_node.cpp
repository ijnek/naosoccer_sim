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
#include "rclcpp/rclcpp.hpp"
#include "rcss3d_controller/rcss3d_controller.hpp"
#include "rcss3d_controller/rcss3d_joint_controller.hpp"
#include "rcss3d_nao/rcss3d_nao.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::NodeOptions options_controller_node = rclcpp::NodeOptions()
    .use_intra_process_comms(true)
    .arguments(
  {
    "--ros-args",
    "--remap", "joint_states:=_joint_states",
    "--remap", "imu/data_raw:=_imu/data_raw",
    "--remap", "joint_commands:=_joint_commands"
  }).parameter_overrides(
  {
    {"camera_frame", "CameraTop_frame"}
  });
  auto rcss3d_controller_node = std::make_shared<rcss3d_controller::Rcss3DController>(
    options_controller_node);

  rclcpp::NodeOptions options_joint_controller_node = rclcpp::NodeOptions()
    .use_intra_process_comms(true)
    .arguments(
  {
    "--ros-args",
    "--remap", "joint_states:=_joint_states",
    "--remap", "joint_positions:=_joint_positions",
    "--remap", "joint_commands:=_joint_commands",
  });
  auto rcss3d_joint_controller_node = std::make_shared<rcss3d_controller::Rcss3DJointController>(
    options_joint_controller_node);

  rclcpp::NodeOptions options_nao_node = rclcpp::NodeOptions()
    .use_intra_process_comms(true);
  auto rcss3d_nao_node = std::make_shared<rcss3d_nao::Rcss3DNao>(options_nao_node);

  executor.add_node(rcss3d_controller_node);
  executor.add_node(rcss3d_joint_controller_node);
  executor.add_node(rcss3d_nao_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
