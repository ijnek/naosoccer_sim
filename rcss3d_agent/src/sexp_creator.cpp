#include "rcss3d_agent/sexp_creator.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

static const float DEG_OVER_RAD = 180 / 3.1415926;
inline static float RAD2DEG(const float x) {
   return ((x) * DEG_OVER_RAD);
}

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

    std::string createJointMessage(std::vector<std::pair<std::string, float>> cmd)
    {
        auto sexp = sexpresso::Sexp{};
        for (auto const &[name, speed] : cmd)
        {
            auto jointSexp = sexpresso::Sexp{name};
            jointSexp.addChild(std::to_string(speed));
            sexp.addChild(std::move(jointSexp));
        }
        return createMessage(sexp, false);
    }

    std::string createBeamMessage(double x, double y, double theta)
    {
        auto sexp = sexpresso::Sexp{"beam"};
        sexp.addChild(std::to_string(x));
        sexp.addChild(std::to_string(y));
        sexp.addChild(std::to_string(RAD2DEG(theta)));
        return createMessage(sexp);
    }
}