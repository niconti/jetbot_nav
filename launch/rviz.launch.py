from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'jetbot_nav'


def generate_launch_description():

    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config', 
        default_value=PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'rviz', 'navigation.rviz'])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '--display-config', LaunchConfiguration('rviz_config')
        ]
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node
    ])