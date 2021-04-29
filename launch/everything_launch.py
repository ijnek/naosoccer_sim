import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        declare_launch_argument(),
        robot_state_publisher(),
        Node(
            package='rcss3d_controller',
            executable='rcss3d_controller_node',
            namespace='sim',
            remappings=[
                ('/joint_commands', '/sim/joint_commands'),
                ('/joint_states', '/sim/joint_states')
            ]
        ),
        Node(
            package='rcss3d_controller',
            executable='rcss3d_joint_controller_node',
            namespace='sim',
            remappings=[
                ('/joint_positions', '/sim/joint_positions'),
                ('/joint_commands', '/sim/joint_commands'),
                ('/joint_states', '/sim/joint_states')
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                "-d", os.path.join(get_package_share_directory('naosoccer_sim'), 'nao.rviz')
            ]
        ),
        Node(
            package='naosoccer_pos_action',
            executable='linear',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"file": os.path.join(get_package_share_directory('naosoccer_pos_action'), 'pos', 'test_arms.pos')}
            ],
            namespace='robot'
        )
    ])


def declare_launch_argument():
    return DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

def robot_state_publisher():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'nao.urdf'
    urdf = os.path.join(
        get_package_share_directory('naosoccer_sim'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf],
            remappings=[('/joint_states', "/sim/joint_states")]
        )