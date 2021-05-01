#include "naosoccer_sim/naosoccer_sim.hpp"
#include "rclcpp/rclcpp.hpp"
#include "naosoccer_sim/socket.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "naosoccer_sim/sexp_creator.hpp"
#include "naosoccer_sim/sexp_parser.hpp"
#include "naosoccer_sim/sim_to_nao.hpp"
#include "naosoccer_sim/nao_to_sim.hpp"

NaoSoccerSim::NaoSoccerSim()
    : Node("NaoSoccerSim", "sim")
{
    RCLCPP_DEBUG(get_logger(), "Declare parameters");
    this->declare_parameter<std::string>("host", "127.0.0.1");
    this->declare_parameter<int>("port", 3100);
    this->declare_parameter<std::string>("team_name", "Anonymous");
    this->declare_parameter<int>("player_number", 0);
    this->declare_parameter("starting_pose", std::vector<double>{0, 0, 0});

    RCLCPP_DEBUG(get_logger(), "Initialise publishers");
    joints_pub = create_publisher<nao_interfaces::msg::Joints>("/sensors/joints", 10);
    buttons_pub = create_publisher<nao_interfaces::msg::Buttons>("/sensors/buttons", 10);
    accelerometer_pub = create_publisher<nao_interfaces::msg::Accelerometer>("/sensors/accelerometer", 10);
    gyroscope_pub = create_publisher<nao_interfaces::msg::Gyroscope>("/sensors/gyroscope", 10);
    angle_pub = create_publisher<nao_interfaces::msg::Angle>("/sensors/angle", 10);
    sonar_pub = create_publisher<nao_interfaces::msg::Sonar>("/sensors/sonar", 10);
    fsr_pub = create_publisher<nao_interfaces::msg::FSR>("/sensors/fsr", 10);
    touch_pub = create_publisher<nao_interfaces::msg::Touch>("/sensors/touch", 10);
    ball_pub = create_publisher<geometry_msgs::msg::Vector3Stamped>("/vision/ball", 10);

    RCLCPP_DEBUG(get_logger(), "Initialise subscriptions");
    joints_sub =
        create_subscription<nao_interfaces::msg::Joints>(
            "/effectors/joints", 10,
            [this](nao_interfaces::msg::Joints::SharedPtr cmd_nao) {
                RCLCPP_DEBUG(this->get_logger(), "Received /effectors/joints");

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
    connection.send(SexpCreator::createInitMessage(
        get_parameter("team_name").as_string(), get_parameter("player_number").as_int()));

    // Start receive and send loop
    receive_thread_ = std::thread(
        [this]() {
            while (rclcpp::ok())
            {
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
                if (ball_found)
                {
                    ball.header.stamp = now();
                    ball_pub->publish(ball);
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
    if (receive_thread_.joinable())
        receive_thread_.join();
}