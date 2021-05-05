import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('team', default_value='Anonymous'),
        DeclareLaunchArgument('player_number', default_value='2'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_theta', default_value='0.0'),

        Node(
            package='naosoccer_sim',
            executable='naosoccer_sim',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'team': LaunchConfiguration('team'),
                'player_number': LaunchConfiguration('player_number'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_theta': LaunchConfiguration('initial_pose_theta')
            }]
        ),
        Node(
            package='naosoccer_sim',
            executable='joints_to_joint_state',
            namespace=LaunchConfiguration('namespace')
        ),
        robot_state_publisher(),
    ])


def robot_state_publisher():

    urdf_file_name = 'nao.urdf'
    urdf = os.path.join(
        get_package_share_directory('nao_description'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf],
        )
