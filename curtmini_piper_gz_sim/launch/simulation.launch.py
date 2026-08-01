import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


ARM_PREFIX = 'piper_'


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _three_values(context, name):
    value = LaunchConfiguration(name).perform(context)
    values = value.replace(',', ' ').split()
    if len(values) != 3:
        raise RuntimeError(
            f'Launch argument {name} must contain exactly three values'
        )
    return ' '.join(values)


def _build_descriptions(context, sim_share):
    description_share = Path(
        get_package_share_directory('curtmini_piper_description')
    )
    mappings = {
        'simulation': 'True',
        'use_sim_time': 'True',
        'use_simplified_collision': 'True',
        'gazebo_controllers': str(
            sim_share / 'config' / 'simulation_controllers.yaml'
        ),
        'arm_prefix': ARM_PREFIX,
        'arm_mount_xyz': _three_values(context, 'arm_mount_xyz'),
        'arm_mount_rpy': _three_values(context, 'arm_mount_rpy'),
        'tcp_offset_xyz': _three_values(context, 'tcp_offset_xyz'),
        'tcp_offset_rpy': _three_values(context, 'tcp_offset_rpy'),
    }

    full_description = xacro.process_file(
        str(
            description_share
            / 'urdf'
            / 'curtmini_piper.urdf.xacro'
        ),
        mappings=mappings,
    ).toxml()

    moveit_mappings = dict(mappings)
    moveit_mappings['simulation'] = 'False'
    moveit_config = (
        MoveItConfigsBuilder(
            'curtmini_piper',
            package_name='curtmini_piper_bringup',
        )
        .robot_description(
            file_path=str(
                description_share
                / 'urdf'
                / 'curtmini_piper_moveit.urdf.xacro'
            ),
            mappings=moveit_mappings,
        )
        .robot_description_semantic(
            file_path='config/curtmini_piper.srdf'
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(
            file_path=str(
                sim_share / 'config' / 'moveit_controllers.yaml'
            )
        )
        .to_moveit_configs()
    )
    moveit_config.sensors_3d = {}
    return full_description, moveit_config


def _controller_spawner(name):
    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            name,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
    )


def _launch_setup(context):
    sim_share = Path(
        get_package_share_directory('curtmini_piper_gz_sim')
    )
    bringup_share = Path(
        get_package_share_directory('curtmini_piper_bringup')
    )
    curt_share = Path(get_package_share_directory('curt_mini'))
    agx_urdf_share = Path(
        get_package_share_directory('agx_arm_urdf')
    )
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))

    full_description, moveit_config = _build_descriptions(
        context, sim_share
    )

    world = LaunchConfiguration('world').perform(context)
    gz_args = []
    if not _as_bool(context, 'paused'):
        gz_args.append('-r')
    if not _as_bool(context, 'gui'):
        gz_args.append('-s')
    gz_args.append(world)

    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        'use_sim_time': True,
    }

    actions = [
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(curt_share.parent),
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(agx_urdf_share.parent),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_share / 'launch' / 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ' '.join(gz_args),
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': full_description,
                    'use_sim_time': True,
                }
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[
                {
                    'config_file': str(
                        sim_share / 'config' / 'gz_bridge.yaml'
                    ),
                    'use_sim_time': True,
                }
            ],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name',
                'curtmini_piper',
                '-topic',
                'robot_description',
                '-x',
                LaunchConfiguration('spawn_x'),
                '-y',
                LaunchConfiguration('spawn_y'),
                '-z',
                LaunchConfiguration('spawn_z'),
                '-Y',
                LaunchConfiguration('spawn_yaw'),
            ],
        ),
        _controller_spawner('joint_state_broadcaster'),
        _controller_spawner('base_controller'),
        _controller_spawner('arm_controller'),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                move_group_configuration,
            ],
            remappings=[('joint_states', '/joint_states')],
            additional_env={
                'DISPLAY': os.environ.get('DISPLAY', '')
            },
        ),
    ]

    if _as_bool(context, 'start_joystick'):
        actions.append(
            GroupAction(
                [
                    SetRemap(
                        src='/joy_teleop/cmd_vel',
                        dst='/base_controller/cmd_vel',
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(
                                curt_share
                                / 'bringup'
                                / 'joystick.launch.py'
                            )
                        )
                    ),
                ]
            )
        )

    if _as_bool(context, 'use_rviz'):
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                output='log',
                arguments=[
                    '-d',
                    str(bringup_share / 'config' / 'moveit.rviz'),
                ],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {'use_sim_time': True},
                ],
                remappings=[('joint_states', '/joint_states')],
            )
        )

    return actions


def generate_launch_description():
    sim_share = Path(
        get_package_share_directory('curtmini_piper_gz_sim')
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=str(
                    sim_share / 'worlds' / 'curtmini_piper.sdf'
                ),
                description='Gazebo world file.',
            ),
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                choices=['true', 'false'],
                description='Start the Gazebo graphical client.',
            ),
            DeclareLaunchArgument(
                'paused',
                default_value='false',
                choices=['true', 'false'],
                description='Start Gazebo paused.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz with the MoveIt panel.',
            ),
            DeclareLaunchArgument(
                'start_joystick',
                default_value='false',
                choices=['true', 'false'],
                description='Start Curt Mini joystick teleoperation.',
            ),
            DeclareLaunchArgument(
                'spawn_x',
                default_value='0.0',
                description='Initial robot x position.',
            ),
            DeclareLaunchArgument(
                'spawn_y',
                default_value='0.0',
                description='Initial robot y position.',
            ),
            DeclareLaunchArgument(
                'spawn_z',
                default_value='0.16',
                description='Initial robot z position.',
            ),
            DeclareLaunchArgument(
                'spawn_yaw',
                default_value='0.0',
                description='Initial robot yaw.',
            ),
            DeclareLaunchArgument(
                'arm_mount_xyz',
                default_value='0 0 0.18',
                description='Piper mount translation from Curt Mini chassis.',
            ),
            DeclareLaunchArgument(
                'arm_mount_rpy',
                default_value='0 0 0',
                description='Piper mount rotation from Curt Mini chassis.',
            ),
            DeclareLaunchArgument(
                'tcp_offset_xyz',
                default_value='0 0 0',
                description='TCP translation from piper_link6.',
            ),
            DeclareLaunchArgument(
                'tcp_offset_rpy',
                default_value='0 0 0',
                description='TCP rotation from piper_link6.',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
