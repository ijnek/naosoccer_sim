# naosoccer_sim
ROS2 package for nao in robocup SPL

Clone using
`git clone --recursive <URL>`
to ensure submodules are cloned too.

If you forget to use `--recursive`, run `git submodule update --init` in the repository once cloned.

Depends on this package too
`https://gitlab.com/ijnek/ros2_rcss3d

Run rcsoccersim3d separately, then run

`ros2 launch naosoccer_sim everything_launch.py`

And in a new terminal, publish messages to move a joint using nao joint names, eg.
```
ros2 topic pub /joint_commands nao_interfaces/msg/JointCommands '{name: {'HeadYaw', 'LShoulderPitch'}, position: {1.0, -1.0}}'
```
