import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap, SetParameter, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'jetbot_nav'


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        name='mode',
        default_value='mapping',
        description="Select either mapping or navigation mode"
    )

    map_file_arg = DeclareLaunchArgument(
        name='map_file',
        default_value=os.path.join(os.getcwd(), 'map'),
        description="The nvblox map to load for navigation"
    )


    nvblox_params_arg = DeclareLaunchArgument(
        name='nvblox_params',
        default_value=PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'config', 'nav2_nvblox.yaml']),
        description="Parameters file specialization for the nvblox node"
    )

    nvblox_include = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nvblox_examples_bringup'), 'launch', 'perception', 'nvblox.launch.py']),
        launch_arguments={ 
            'mode': 'static',
            'camera': 'realsense',
            'run_standalone': 'True'
        }.items()
    )


    nvblox_mapping_group = GroupAction(
        actions=[
            SetRemap(src='/camera/realsense_splitter_node/output/depth', dst='/camera/depth/image_rect_raw'),
            nvblox_include
        ],
        condition=LaunchConfigurationEquals('mode', 'mapping')
    )

    nvblox_navigation_group = GroupAction(
        actions=[
            # SetParametersFromFile(LaunchConfiguration('nvblox_params')),
            # SetRemap(src='/camera/color/image_raw', dst='/camera/color/image_raw/offline'),
            # SetRemap(src='/camera/realsense_splitter_node/output/depth', dst='/camera/depth/image_rect_raw/offline'),
            nvblox_include,
            ExecuteProcess(cmd=['ros2', 'param', 'load', '/nvblox_node', LaunchConfiguration('nvblox_params')]),
            ExecuteProcess(cmd=['ros2', 'service', 'call', '/nvblox_node/load_map', 'nvblox_msgs/srv/FilePath', "{{ file_path: {0} }}".format(os.path.join(os.getcwd(), 'map')) ])
        ],
        condition=LaunchConfigurationEquals('mode', 'navigation')
    )


    return LaunchDescription([
        mode_arg,
        map_file_arg,
        nvblox_params_arg,
        nvblox_mapping_group,
        nvblox_navigation_group
    ])