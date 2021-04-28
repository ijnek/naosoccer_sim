#include "naosoccer_sim/sexp_parser.hpp"

static constexpr double deg2rad(double rad) { return rad * 3.141592654 / 180.0; }

naosoccer_sim_interfaces::msg::Joints SexpParser::getJoints()
{
    auto joints = naosoccer_sim_interfaces::msg::Joints{};
    for (auto const &arg : sexp.arguments())
    {
        // Joint expressions have form: (HJ (n llj2) (ax -0.00))
        auto const &s = arg.value.sexp;
        if (s[0].value.str == "HJ")
        {
            joints.name.push_back(s[1].value.sexp[1].value.str);
            joints.position.push_back(std::stod(s[2].value.sexp[1].value.str));
        }
    }
    return joints;
}

std::tuple<bool, nao_interfaces::msg::Accelerometer> SexpParser::getAccelerometer()
{
    nao_interfaces::msg::Accelerometer accelerometerMsg = nao_interfaces::msg::Accelerometer{};

    auto const *accSexp = sexp.getChildByPath("ACC/a");
    bool found = (accSexp != nullptr);
    if (found) {
        RCLCPP_DEBUG(logger, "Found accelerometer information");

        auto const &aSexp = accSexp->value.sexp;
        accelerometerMsg.x = std::stod(aSexp[2].value.str);
        accelerometerMsg.y = -std::stod(aSexp[1].value.str);
        accelerometerMsg.z = std::stod(aSexp[3].value.str);
    } else {
        RCLCPP_DEBUG(logger, "No accelerometer information found");
    }

    return {found, accelerometerMsg};
}

std::tuple<bool, nao_interfaces::msg::Gyroscope> SexpParser::getGyroscope()
{
    nao_interfaces::msg::Gyroscope gyroscopeMsg = nao_interfaces::msg::Gyroscope{};

    auto const *gyrSexp = sexp.getChildByPath("GYR/rt");
    bool found = (gyrSexp != nullptr);
    if (found) {
        RCLCPP_DEBUG(logger, "Found gyroscope information");

        auto const &rateSexp = gyrSexp->value.sexp;
        gyroscopeMsg.x = deg2rad(std::stod(rateSexp[2].value.str));
        gyroscopeMsg.y = -deg2rad(std::stod(rateSexp[1].value.str));
        gyroscopeMsg.z = deg2rad(std::stod(rateSexp[3].value.str));
    } else {
        RCLCPP_DEBUG(logger, "No gyroscope information found");
    }

    return {found, gyroscopeMsg};
}
