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
#include "rcss3d_agent/rcss3d_agent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent/socket.hpp"
#include "rcss3d_agent/sexp_creator.hpp"
#include "rcss3d_agent/sexp_parser.hpp"
#include "rcss3d_agent/sim_to_nao.hpp"
#include "rcss3d_agent/nao_to_sim.hpp"

Rcss3dAgent::Rcss3dAgent()
: Node("Rcss3dAgent")
{
  RCLCPP_DEBUG(get_logger(), "Declare parameters");
  this->declare_parameter<std::string>("host", "127.0.0.1");
  this->declare_parameter<int>("port", 3100);
  this->declare_parameter<std::string>("team", "Anonymous");
  this->declare_parameter<int>("number", 2);
  this->declare_parameter<double>("x", 0.0);
  this->declare_parameter<double>("y", 0.0);
  this->declare_parameter<double>("theta", 0.0);

  RCLCPP_DEBUG(get_logger(), "Initialise publishers");
  joint_positions_pub = create_publisher<nao_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 10);
  accelerometer_pub = create_publisher<nao_sensor_msgs::msg::Accelerometer>(
    "sensors/accelerometer", 10);
  gyroscope_pub = create_publisher<nao_sensor_msgs::msg::Gyroscope>("sensors/gyroscope", 10);
  angle_pub = create_publisher<nao_sensor_msgs::msg::Angle>("sensors/angle", 10);
  fsr_pub = create_publisher<nao_sensor_msgs::msg::FSR>("sensors/fsr", 10);
  ball_pub = create_publisher<geometry_msgs::msg::PointStamped>("vision/ball", 10);
  posts_pub = create_publisher<soccer_vision_msgs::msg::GoalpostArray>("vision/goalposts", 10);
  lines_pub = create_publisher<soccer_vision_msgs::msg::FieldLineArray>("vision/field_lines", 10);
  robots_pub = create_publisher<soccer_vision_msgs::msg::RobotArray>("vision/robots", 10);
  flags_pub = create_publisher<soccer_vision_msgs::msg::FlagArray>("vision/flags", 10);

  RCLCPP_DEBUG(get_logger(), "Initialise subscriptions");
  joint_positions_sub =
    create_subscription<nao_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 10,
    [this](nao_command_msgs::msg::JointPositions::SharedPtr cmd_nao) {
      RCLCPP_DEBUG(this->get_logger(), "Received effectors/joint_positions");

      SimJointCommands cmd_sim = nao_to_sim(*cmd_nao);
      naoJointsPid.setTarget(cmd_sim);
    });

  // Initialise connection
  connection.initialise(
    get_parameter("host").as_string(),
    get_parameter("port").as_int());

  // Create the robot
  connection.send(SexpCreator::createCreateMessage());

  // Receive, this is needed for the init message to be sent next
  connection.receive();

  // Send init
  connection.send(
    SexpCreator::createInitMessage(
      get_parameter("team").as_string(), get_parameter("number").as_int()));

  // Receive, this is needed for the beam message to be sent next
  connection.receive();

  // Send beam
  connection.send(
    SexpCreator::createBeamMessage(
      get_parameter("x").as_double(),
      get_parameter("y").as_double(),
      get_parameter("theta").as_double()));

  // Start receive and send loop
  receive_thread_ = std::thread(
    [this]() {
      while (rclcpp::ok()) {
        std::string recv = connection.receive();
        RCLCPP_DEBUG(this->get_logger(), ("Received: " + recv).c_str());

        SexpParser parsed(recv);

        std::vector<std::pair<std::string, float>> joints = parsed.getJoints();
        joint_positions_pub->publish(sim_to_nao(joints));

        nao_sensor_msgs::msg::Accelerometer acc_val = parsed.getAccelerometer();
        accelerometer_pub->publish(acc_val);

        nao_sensor_msgs::msg::Gyroscope gyr_val = parsed.getGyroscope();
        gyroscope_pub->publish(gyr_val);

        angle_pub->publish(complementaryFilter.update(acc_val, gyr_val));

        fsr_pub->publish(parsed.getFSR());

        auto [ball_found, ball] = parsed.getBall();
        if (ball_found) {
          ball.header.stamp = now();
          ball_pub->publish(ball);
        }

        auto [posts_found, posts] = parsed.getGoalposts();
        if (posts_found) {
          for (auto & p : posts.posts) {
            p.header.stamp = now();
          }
          posts_pub->publish(posts);
        }

        auto [lines_found, lines] = parsed.getFieldLines();
        if (lines_found) {
          for (auto & l : lines.lines) {
            l.header.stamp = now();
          }
          lines_pub->publish(lines);
        }

        auto [robots_found, robots] = parsed.getRobots();
        if (robots_found) {
          for (auto & r : robots.robots) {
            r.header.stamp = now();
          }
          robots_pub->publish(robots);
        }

        auto [flags_found, flags] = parsed.getFlags();
        if (flags_found) {
          for (auto & r : flags.flags) {
            r.header.stamp = now();
          }
          flags_pub->publish(flags);
        }

        SimJoints speed_cmd_sim = naoJointsPid.update(toSimJoints(joints));
        std::string msg = SexpCreator::createJointMessage(fromSimJoints(speed_cmd_sim));
        RCLCPP_DEBUG(this->get_logger(), ("Sending: " + msg).c_str());
        connection.send(msg);
      }
    });
}

Rcss3dAgent::~Rcss3dAgent()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}
