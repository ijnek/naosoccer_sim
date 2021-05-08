#ifndef NAO_TO_SIM_HPP
#define NAO_TO_SIM_HPP

#include "nao_interfaces/msg/joints.hpp"
#include "rcss3d_agent/sim_joints.hpp"
#include <map>

SimJoints nao_to_sim(const nao_interfaces::msg::Joints &nao_joints);

#endif // NAO_TO_SIM_HPP