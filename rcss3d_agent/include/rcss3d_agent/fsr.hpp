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

#ifndef RCSS3D_AGENT__FSR_HPP_
#define RCSS3D_AGENT__FSR_HPP_

namespace FSR
{

enum Foot
{
  Left,
  Right,
  NUM_FOOT
};

enum Location
{
  FrontLeft,
  FrontRight,
  BackLeft,
  BackRight,
  NUM_LOCATION
};

enum Coordinate
{
  X,
  Y,
  NUM_COORDINATE
};

#define FOOT_CENTRE_DIFFERENCE 0.030  // in the simulated nao robot, FRP

// http://doc.aldebaran.com/2-1/family/robots/fsr_robot.html
const float FSR_POS[NUM_FOOT][NUM_LOCATION][NUM_COORDINATE] = {
  {{0.07025 - FOOT_CENTRE_DIFFERENCE, 0.0299}, {0.07025 - FOOT_CENTRE_DIFFERENCE, -0.0231},
    {-0.03025 - FOOT_CENTRE_DIFFERENCE, 0.0299}, {-0.02965 - FOOT_CENTRE_DIFFERENCE, -0.0191}},
  {{0.07025 - FOOT_CENTRE_DIFFERENCE, 0.0231}, {0.07025 - FOOT_CENTRE_DIFFERENCE, -0.0299},
    {-0.03025 - FOOT_CENTRE_DIFFERENCE, 0.0191}, {-0.02965 - FOOT_CENTRE_DIFFERENCE, -0.0299}}
};

}  // namespace FSR

#endif  // RCSS3D_AGENT__FSR_HPP_
