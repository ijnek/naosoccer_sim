#ifndef SEXP_CREATOR_HPP
#define SEXP_CREATOR_HPP

#include <string>
#include <vector>

namespace SexpCreator
{
    std::string createCreateMessage();
    std::string createInitMessage(std::string const &team_name, int player_number);
    std::string createJointMessage(std::vector<std::pair<std::string, float>> cmd);
}


#endif // SEXP_CREATOR_HPP