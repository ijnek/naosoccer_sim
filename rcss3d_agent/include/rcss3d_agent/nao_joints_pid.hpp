#ifndef NAO_JOINTS_PID
#define NAO_JOINTS_PID

#include "rcss3d_agent/joint_pid.hpp"
#include "rcss3d_agent/sim_joints.hpp"
#include <vector>

class NaoJointsPid
{
public:
    NaoJointsPid();
    void setTarget(SimJoints target);

    SimJoints update(SimJoints nextJoints);

private:
    JointPid<float, NUM_SIM_JOINTS> jointPid;
    SimJoints target;
};

#endif // NAO_JOINTS_PID