#include <gtest/gtest.h>
#include "naosoccer_sim/joint_pid.hpp"

TEST(TestJointPid, TestDifferentTypes)
{
    JointPid<float, 1> pid(2, 0, 0);
    EXPECT_EQ(8, pid.update(std::array<float, 1>{1}, std::array<float, 1>{5})[0]);

    JointPid<int, 1> pid2(2, 0, 0);
    EXPECT_EQ(8, pid2.update(std::array<int, 1>{1}, std::array<int, 1>{5})[0]);
}