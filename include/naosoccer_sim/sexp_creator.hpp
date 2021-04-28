#ifndef SEXP_CREATOR_HPP
#define SEXP_CREATOR_HPP

#include <string>

namespace SexpCreator
{
    std::string createCreateMessage();
    std::string createInitMessage(std::string const &team_name, int player_number);
    
}


#endif // SEXP_CREATOR_HPP