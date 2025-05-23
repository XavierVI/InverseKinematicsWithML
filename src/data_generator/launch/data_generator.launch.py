from typing import List

from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    #-------------------------------------------
    # Launch Arguments
    #-------------------------------------------
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')


    data_generator = Node(
        package='data_generator',
        executable='data_generator',
        namespace=robot_name_launch_arg,
        parameters=[{
            'robot_model': robot_model_launch_arg,
        }],
        arguments=[
            '--robot_model', robot_model_launch_arg.perform(context),
            '--robot_name', robot_name_launch_arg.perform(context),
        ],
    )

    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'use_sim': use_sim_launch_arg,
            'robot_description': robot_description_launch_arg,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg)
    )

    return [
        xsarm_control_launch,
        data_generator
    ]


def generate_launch_description():
    declared_arguments: List[DeclareLaunchArgument] = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='px100',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if xsarm_control should be launched - set to `false` if you would like to '
                'run your own version of this file separately.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            choices=('true', 'false'),
            description=(
                "if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's"
                ' motion; if `false`, the real DYNAMIXEL driver node is run.'
            ),
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments()
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
