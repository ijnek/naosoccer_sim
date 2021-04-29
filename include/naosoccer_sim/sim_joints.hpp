#ifndef SIM_JOINTS_HPP
#define SIM_JOINTS_HPP

#include <array>
#include <vector>
#include <algorithm>

enum SimJointIndex
{
    he1,
    he2,
    lae1,
    lae2,
    lae3,
    lae4,
    lle1,
    lle2,
    lle3,
    lle4,
    lle5,
    lle6,
    rle1,
    rle2,
    rle3,
    rle4,
    rle5,
    rle6,
    rae1,
    rae2,
    rae3,
    rae4,
    NUM_SIM_JOINTS
};

const std::array<std::string, NUM_SIM_JOINTS> jointNames = {
    "he1",
    "he2",
    "lae1",
    "lae2",
    "lae3",
    "lae4",
    "lle1",
    "lle2",
    "lle3",
    "lle4",
    "lle5",
    "lle6",
    "rle1",
    "rle2",
    "rle3",
    "rle4",
    "rle5",
    "rle6",
    "rae1",
    "rae2",
    "rae3",
    "rae4",
};

const std::array<std::string, NUM_SIM_JOINTS> jointPerceptorNames = {
    "hj1",
    "hj2",
    "laj1",
    "laj2",
    "laj3",
    "laj4",
    "llj1",
    "llj2",
    "llj3",
    "llj4",
    "llj5",
    "llj6",
    "rlj1",
    "rlj2",
    "rlj3",
    "rlj4",
    "rlj5",
    "rlj6",
    "raj1",
    "raj2",
    "raj3",
    "raj4",
};

typedef std::array<float, NUM_SIM_JOINTS> SimJoints;

SimJoints toSimJoints(std::vector<std::pair<std::string, float>> in);
std::vector<std::pair<std::string, float>> fromSimJoints(SimJoints &in);

#endif // SIM_JOINTS_HPP