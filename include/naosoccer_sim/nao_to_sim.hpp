#ifndef NAO_TO_SIM_HPP
#define NAO_TO_SIM_HPP

#include "nao_interfaces/msg/joints.hpp"
#include <map>

std::vector<std::pair<std::string, float>> nao_to_sim(const nao_interfaces::msg::Joints &nao_joints);

#endif // NAO_TO_SIM_HPP