#ifndef SIM_TO_NAO_HPP
#define SIM_TO_NAO_HPP

#include "nao_interfaces/msg/joints.hpp"
#include <map>

nao_interfaces::msg::Joints sim_to_nao(std::vector<std::pair<std::string, float>> sim_joints);

#endif // SIM_TO_NAO_HPP