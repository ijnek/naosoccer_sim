#ifndef COMPLEMENTARY_FILTER_HPP
#define COMPLEMENTARY_FILTER_HPP

#include "nao_interfaces/msg/angle.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"

/*
 *  The simulator does not provide an AngleX or AngleY sensor. Luckily,
 *  we can calculate this information ourselves using the gyroscope and
 *  accelerometer information. This class does this to provide an estimation
 *  of such a sensor.
 */

class ComplementaryFilter
{
public:
    ComplementaryFilter(float initial_x, float initial_y)
        : x_(initial_x), y_(initial_y)
    {
    }

    ComplementaryFilter()
        : x_(0.0f), y_(0.0f)
    {
    }

    void addMeasurement(const nao_interfaces::msg::Accelerometer &acc,
                        const nao_interfaces::msg::Gyroscope &gyr);

    nao_interfaces::msg::Angle getAngle();

private:
    float x_;
    float y_;
};

#endif // COMPLEMENTARY_FILTER_HPP