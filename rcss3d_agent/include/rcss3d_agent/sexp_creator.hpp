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

#ifndef RCSS3D_AGENT__SEXP_CREATOR_HPP_
#define RCSS3D_AGENT__SEXP_CREATOR_HPP_

#include <utility>
#include <string>
#include <vector>

namespace SexpCreator
{
std::string createCreateMessage();
std::string createInitMessage(std::string const & team_name, int number);
std::string createJointMessage(std::vector<std::pair<std::string, float>> cmd);
std::string createBeamMessage(double x, double y, double theta);
}

#endif  // RCSS3D_AGENT__SEXP_CREATOR_HPP_
