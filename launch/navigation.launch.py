from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'jetbot_nav'


def generate_launch_description():

    nav2_params_arg = DeclareLaunchArgument(
        name='params_file', 
        default_value=PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'config', 'nav2_params.yaml'])
    )

    nav2_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        )
    )

    nav2_group = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/jetbot/cmd_vel'),
            SetRemap(src='/jetbot/odom', dst='/visual_slam/tracking/odometry'),
            nav2_bringup_include
        ],
    )


    rviz2_arg = DeclareLaunchArgument(
        name='use_rviz', 
        default_value='True',
        description="Set this flag to launch rviz visualization"
    )

    rviz2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'launch', 'rviz.launch.py']),
        )
    )

    rviz2_group = GroupAction(
        actions=[
            rviz2_include
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )


    return LaunchDescription([
        nav2_params_arg,
        nav2_group,
        rviz2_arg,
        rviz2_group
    ])