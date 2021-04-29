#include "naosoccer_sim/nao_joints_pid.hpp"

#define P 28.0
#define I 0.0
#define D 0.0

NaoJointsPid::NaoJointsPid()
    : jointPid(P, I, D)
{}

void NaoJointsPid::setTarget(SimJoints target)
{
    this->target = target;
}

SimJoints NaoJointsPid::update(SimJoints current)
{
    SimJoints out = jointPid.update(current, target);
    return out;
}

