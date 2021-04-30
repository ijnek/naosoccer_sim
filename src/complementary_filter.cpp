#include "naosoccer_sim/complementary_filter.hpp"

#include <cmath>

#include <iostream>


#define SIM_DT 1.0 / 50.0

#define ACC_WEIGHT 0.01
#define GYR_WEIGHT (1.0 - ACC_WEIGHT)

// Using implementation provided here: https://www.youtube.com/watch?v=whSw42XddsU&ab_channel=BrianDouglas
// Axes of NAO are explained here: http://doc.aldebaran.com/2-1/family/robots/inertial_robot.html

void ComplementaryFilter::addMeasurement(const nao_interfaces::msg::Accelerometer &acc,
                                         const nao_interfaces::msg::Gyroscope &gyr)
{
    float angle_x_from_gyr = x_ + gyr.x * SIM_DT;
    float angle_y_from_gyr = y_ + gyr.y * SIM_DT;

    float angle_x_from_acc = atan2(acc.y, acc.z);
    float angle_y_from_acc = atan2(-acc.x, acc.z);

    x_ = GYR_WEIGHT * angle_x_from_gyr + ACC_WEIGHT * angle_x_from_acc;
    y_ = GYR_WEIGHT * angle_y_from_gyr + ACC_WEIGHT * angle_y_from_acc;
}

nao_interfaces::msg::Angle ComplementaryFilter::getAngle()
{
    nao_interfaces::msg::Angle angles;
    angles.x = x_;
    angles.y = y_;
    return angles;
}
