#ifndef SEXP_CREATOR_HPP
#define SEXP_CREATOR_HPP

#include <string>
#include "naosoccer_sim_interfaces/msg/joint_speed_command.hpp"

namespace SexpCreator
{
    std::string createCreateMessage();
    std::string createInitMessage(std::string const &team_name, int player_number);
    std::string createJointMessage(naosoccer_sim_interfaces::msg::JointSpeedCommand::SharedPtr cmd);
}


#endif // SEXP_CREATOR_HPP