// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

    nao_interfaces::msg::Angle update(const nao_interfaces::msg::Accelerometer &acc,
                                      const nao_interfaces::msg::Gyroscope &gyr);

private:
    float x_;
    float y_;

    nao_interfaces::msg::Angle getAngle();
};

#endif // COMPLEMENTARY_FILTER_HPP