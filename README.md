# naosoccer_sim
ROS2 package for nao in robocup SPL

Depends on this package too
`https://gitlab.com/ijnek/ros2_rcss3d

Run rcsoccersim3d separately, then run

`ros2 launch naosoccer_sim everything_launch.py`

And in a new terminal, publish messages to move a joint using nao joint names, eg.
```
ros2 topic pub /nao_joint_positions nao_interfaces/msg/JointPositionCommand '{name: {'HeadYaw', 'LShoulderPitch'}, position: {1.0, -1.0}}'
```
