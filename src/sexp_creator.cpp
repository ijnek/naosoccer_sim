#include "naosoccer_sim/sexp_creator.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

namespace SexpCreator
{
    std::string createMessage(sexpresso::Sexp sexp, bool wrap = true)
    {
        auto root = sexpresso::Sexp{};
        if (wrap)
        {
            root.addChild(std::move(sexp));
        }
        else
        {
            root = std::move(sexp);
        }

        auto msg = root.toString();
        return msg;
    }

    std::string createCreateMessage()
    {
        auto sceneSexp = sexpresso::Sexp{"scene"};
        sceneSexp.addChild("rsg/agent/nao/nao.rsg");
        return createMessage(sceneSexp);
    }

    std::string createInitMessage(std::string const &team_name, int player_number)
    {
        auto initSexp = sexpresso::Sexp{"init"};

        auto unumSexp = sexpresso::Sexp{"unum"};
        unumSexp.addChild(std::to_string(player_number));
        initSexp.addChild(std::move(unumSexp));

        auto teamSexp = sexpresso::Sexp{"teamname"};
        teamSexp.addChild(team_name);
        initSexp.addChild(std::move(teamSexp));

        return createMessage(initSexp);
    }
}