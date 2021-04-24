#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest, GTest
from launch_testing.legacy import LaunchTestService
from launch import LaunchService
import sys
import os

def main(argv=sys.argv[1:]):
    testExecutable = os.getenv('TEST_EXECUTABLE')

    ld = LaunchDescription([])

    nao_to_sim_node = Node(
        package='naosoccer_sim',
        executable='nao_to_sim_node'
    )
    ld.add_action(nao_to_sim_node)

    test1_action = ExecuteProcess(
            cmd=[testExecutable],
            name='test_nao_to_sim',
            output='screen',
        )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)
    
if __name__ == '__main__':
    sys.exit(main())