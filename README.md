# naosoccer_sim
ROS2 package for interacting with the SimSpark simulation used in the RoboCup 3D simulation league, from a nao api.

## Package Summary

## Overview

## naosoccer_sim node

### Subscribed Topics

`effectors/joints` (nao_interfaces/msg/Joints)

    Joint effector commands to be sent to the robot

### Published Topics

`sensors/joints` (nao_interfaces/msg/Joints)

    Joint sensor information

`sensors/buttons` (nao_interfaces/msg/Buttons)

    Button sensor information

`sensors/accelerometer` (nao_interfaces/msg/Accelerometer)

    Accelerometer sensor information

`sensors/gyroscope` (nao_interfaces/msg/Gyroscope)

    Gyroscope sensor information

`sensors/angle` (nao_interfaces/msg/Angle)

    Angle sensor information

`sensors/sonar` (nao_interfaces/msg/Sonar)

    Sonar sensor information

`sensors/fsr` (nao_interfaces/msg/FSR)

    FSR sensor information

`sensors/touch` (nao_interfaces/msg/Touch)

    Touch sensor information

`vision/ball` (geometry_msgs/msg/PointStamped)

    Ball vision information, relative to 'CameraTop_frame'

### Parameters

`host` (string)

    Host IP Address that simulation server is running on
    
`port` (int)

    Port number that simulation server is communicating on
    
`team` (string)

    Team name of robot, to be sent to simulation server
    
`player_number` (int)

    Player number of robot, to be sent to simulation server

`initial_pose_x` (double)

    Initial position of robot along the x-axis of the field in metres, where 0.0 is the centre, and positive x is towards the opponent goal.
    
`initial_pose_y` (double)

    Initial position of robot along the y-axis of the field in metres, where 0.0 is the centre, and positive y is left when facing the opponent goal.
    
`initial_pose_theta` (double)

    Initial heading of robot along the x-axs of the field in radians, where 0.0 is facing the opponent goal, and positive theta is anti-clockwise.


# Others

Dependencies:
* nao_interfaces
* nao_description

Clone using (https or ssh, as the following)
`git clone --recursive https://github.com/ijnek/naosoccer_sim.git`
to ensure submodules are cloned too.

If you forget to use `--recursive`, run `git submodule update --init` in the repository once cloned.

To run,

1. Run `rcsoccersim3d` in a terminal, graphical monitor should show up
1. In another terminal, run `. install/local_setup.bash` and then run `ros2 launch naosoccer_sim everything_launch.py`
