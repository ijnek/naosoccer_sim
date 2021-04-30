#include "naosoccer_sim/sexp_parser.hpp"

static constexpr double deg2rad(double rad) { return rad * 3.141592654 / 180.0; }

std::vector<std::pair<std::string, float>> SexpParser::getJoints()
{
    std::vector<std::pair<std::string, float>> joints;
    for (auto const &arg : sexp.arguments())
    {
        // Joint expressions have form: (HJ (n llj2) (ax -0.00))
        auto const &s = arg.value.sexp;
        if (s.at(0).value.str == "HJ")
        {
            std::string name = s.at(1).value.sexp.at(1).value.str;
            float position = deg2rad(std::stod(s.at(2).value.sexp.at(1).value.str));
            joints.push_back(std::make_pair(name, position));
        }
    }
    return joints;
}

nao_interfaces::msg::Accelerometer SexpParser::getAccelerometer()
{
    nao_interfaces::msg::Accelerometer accelerometerMsg;

    auto const *accSexp = sexp.getChildByPath("ACC/a");
    bool found = (accSexp != nullptr);
    if (found)
    {
        RCLCPP_DEBUG(logger, "Found accelerometer information");

        auto const &aSexp = accSexp->value.sexp;
        accelerometerMsg.x = std::stod(aSexp.at(2).value.str);
        accelerometerMsg.y = -std::stod(aSexp.at(1).value.str);
        accelerometerMsg.z = std::stod(aSexp.at(3).value.str);
    }
    else
    {
        RCLCPP_ERROR(logger, "No accelerometer information found");
    }

    return accelerometerMsg;
}

nao_interfaces::msg::Gyroscope SexpParser::getGyroscope()
{
    nao_interfaces::msg::Gyroscope gyroscopeMsg;

    auto const *gyrSexp = sexp.getChildByPath("GYR/rt");
    bool found = (gyrSexp != nullptr);
    if (found)
    {
        RCLCPP_DEBUG(logger, "Found gyroscope information");

        auto const &rateSexp = gyrSexp->value.sexp;
        gyroscopeMsg.x = deg2rad(std::stod(rateSexp.at(2).value.str));
        gyroscopeMsg.y = -deg2rad(std::stod(rateSexp.at(1).value.str));
        gyroscopeMsg.z = deg2rad(std::stod(rateSexp.at(3).value.str));
    }
    else
    {
        RCLCPP_ERROR(logger, "No gyroscope information found");
    }

    return gyroscopeMsg;
}


// WARNING!!!!
// THE SIMULATOR USES DIFFERENT SENSORS TO THE REAL ROBOT, AND THE CONVERSION
// IS NOT COMPLETED YET. WOULD RECOMMEND NOT USING THIS MSG
nao_interfaces::msg::FSR SexpParser::getFSR()
{
    nao_interfaces::msg::FSR fsrMsg;

    for (auto const &arg : sexp.arguments())
    {
        // Joint expressions have form: (FRP (n lf) (c -0.14 0.08 -0.05) (f 1.12 -0.26 13.07))
        auto const &s = arg.value.sexp;
        if (s.at(0).value.str == "FRP")
        {
            std::string frp_name = s.at(1).value.sexp.at(1).value.str;
            if (frp_name == "lf")
            {
                float lf_val = std::stof(s.at(3).value.sexp.at(3).value.str);
                fsrMsg.l_foot_front_left = lf_val;
                fsrMsg.l_foot_front_right = lf_val;
                fsrMsg.l_foot_back_left = lf_val;
                fsrMsg.l_foot_back_right = lf_val;
            }
            else if (frp_name == "rf")
            {
                float rf_val = std::stof(s.at(3).value.sexp.at(3).value.str);
                fsrMsg.r_foot_front_left = rf_val;
                fsrMsg.r_foot_front_right = rf_val;
                fsrMsg.r_foot_back_left = rf_val;
                fsrMsg.r_foot_back_right = rf_val;
            }
            else
            {
                RCLCPP_ERROR(logger, "Received Unknown FRP with name: " + frp_name);
            }
        }
    }

    return fsrMsg;
}
