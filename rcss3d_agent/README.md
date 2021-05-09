# rcss3d_agent
ROS2 package for interacting with the SimSpark simulation used in the RoboCup 3D simulation league, from a nao api.

Documentation can be found on our [ReadTheDocs page](https://naosoccer-sim.readthedocs.io/)

Dependencies:
* nao_interfaces
* nao_description

Clone using (https or ssh, as the following)
`git clone --recursive https://github.com/ijnek/naosoccer_sim.git`
to ensure submodules are cloned too.

If you forget to use `--recursive`, run `git submodule update --init` in the repository once cloned.

To run,

1. Run `rcsoccersim3d` in a terminal, graphical monitor should show up
1. In another terminal, run `. install/local_setup.bash` and then run `ros2 launch rcss3d_agent everything_launch.py`