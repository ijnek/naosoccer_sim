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
#include "nao_interfaces/msg/joints.hpp"
#include "rcss3d_agent/sexp_creator.hpp"
#include "rcss3d_agent/sexp_parser.hpp"
#include "rcss3d_agent/sim_to_nao.hpp"
#include "rcss3d_agent/nao_to_sim.hpp"

NaoSoccerSim::NaoSoccerSim()
: Node("NaoSoccerSim")
{
  RCLCPP_DEBUG(get_logger(), "Declare parameters");
  this->declare_parameter<std::string>("host", "127.0.0.1");
  this->declare_parameter<int>("port", 3100);
  this->declare_parameter<std::string>("team", "Anonymous");
  this->declare_parameter<int>("player_number", 0);
  this->declare_parameter<double>("initial_pose_x", 0.0);
  this->declare_parameter<double>("initial_pose_y", 0.0);
  this->declare_parameter<double>("initial_pose_theta", 0.0);

  RCLCPP_DEBUG(get_logger(), "Initialise publishers");
  joints_pub = create_publisher<nao_interfaces::msg::Joints>("sensors/joints", 10);
  buttons_pub = create_publisher<nao_interfaces::msg::Buttons>("sensors/buttons", 10);
  accelerometer_pub = create_publisher<nao_interfaces::msg::Accelerometer>(
    "sensors/accelerometer", 10);
  gyroscope_pub = create_publisher<nao_interfaces::msg::Gyroscope>("sensors/gyroscope", 10);
  angle_pub = create_publisher<nao_interfaces::msg::Angle>("sensors/angle", 10);
  sonar_pub = create_publisher<nao_interfaces::msg::Sonar>("sensors/sonar", 10);
  fsr_pub = create_publisher<nao_interfaces::msg::FSR>("sensors/fsr", 10);
  touch_pub = create_publisher<nao_interfaces::msg::Touch>("sensors/touch", 10);
  ball_pub = create_publisher<geometry_msgs::msg::PointStamped>("vision/ball", 10);
  posts_pub = create_publisher<naosoccer_interfaces::msg::Goalposts>("vision/posts", 10);

  RCLCPP_DEBUG(get_logger(), "Initialise subscriptions");
  joints_sub =
    create_subscription<nao_interfaces::msg::Joints>(
    "effectors/joints", 10,
    [this](nao_interfaces::msg::Joints::SharedPtr cmd_nao) {
      RCLCPP_DEBUG(this->get_logger(), "Received effectors/joints");

      SimJoints cmd_sim = nao_to_sim(*cmd_nao);
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
      get_parameter("team").as_string(), get_parameter("player_number").as_int()));

  // Receive, this is needed for the beam message to be sent next
  connection.receive();

  // Send beam
  connection.send(
    SexpCreator::createBeamMessage(
      get_parameter("initial_pose_x").as_double(),
      get_parameter("initial_pose_y").as_double(),
      get_parameter("initial_pose_theta").as_double()));

  // Start receive and send loop
  receive_thread_ = std::thread(
    [this]() {
      while (rclcpp::ok()) {
        std::string recv = connection.receive();
        RCLCPP_DEBUG(this->get_logger(), "Received: " + recv);

        SexpParser parsed(recv);

        std::vector<std::pair<std::string, float>> joints = parsed.getJoints();
        joints_pub->publish(sim_to_nao(joints));

        nao_interfaces::msg::Accelerometer acc_val = parsed.getAccelerometer();
        accelerometer_pub->publish(acc_val);

        nao_interfaces::msg::Gyroscope gyr_val = parsed.getGyroscope();
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

        SimJoints speed_cmd_sim = naoJointsPid.update(toSimJoints(joints));
        std::string msg = SexpCreator::createJointMessage(fromSimJoints(speed_cmd_sim));
        RCLCPP_DEBUG(this->get_logger(), "Sending: " + msg);
        connection.send(msg);
      }
    });
}

NaoSoccerSim::~NaoSoccerSim()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}
