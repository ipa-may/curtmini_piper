import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERAS = ('base_camera', 'tool_camera')


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f'{label} must be a number')
    if not math.isfinite(value):
        raise RuntimeError(f'{label} must be finite')
    return str(float(value))


def _launch_setup(context):
    camera = LaunchConfiguration('camera').perform(context)
    serial = LaunchConfiguration('serial_no').perform(context).strip()
    if not serial:
        raise RuntimeError(f'Set serial_no for {camera}; do not select a random USB camera')
    if serial.startswith('_'):
        serial = serial[1:]
    if not serial.isdigit():
        raise RuntimeError(f'{camera} serial_no must contain only digits')

    mounts_path = Path(LaunchConfiguration('mounts_file').perform(context))
    with mounts_path.open(encoding='utf-8') as stream:
        mounts = yaml.safe_load(stream)
    mount = mounts[f'{camera}_mount']
    position = mount['position']
    rotation = mount['rotation']

    return [
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=camera,
            namespace='/',
            output='screen',
            parameters=[{
                'camera_name': camera,
                'camera_namespace': '',
                'serial_no': f'_{serial}',
                'enable_color': True,
                'enable_depth': True,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 1,
                'publish_tf': True,
                'pointcloud.enable': False,
                'rgb_camera.color_profile': '640x480x30',
                'depth_module.depth_profile': '640x480x30',
                'use_sim_time': False,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{camera}_mount_tf',
            arguments=[
                '--x', _number(position['x'], f'{camera}.x'),
                '--y', _number(position['y'], f'{camera}.y'),
                '--z', _number(position['z'], f'{camera}.z'),
                '--roll', _number(rotation['r'], f'{camera}.r'),
                '--pitch', _number(rotation['p'], f'{camera}.p'),
                '--yaw', _number(rotation['y'], f'{camera}.y'),
                '--frame-id', mount['parent'],
                '--child-frame-id', f'{camera}_link',
            ],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('curtmini_piper_sensors'))
    return LaunchDescription([
        DeclareLaunchArgument('camera', choices=CAMERAS,
                              description='Physical camera mount to start.'),
        DeclareLaunchArgument('serial_no', default_value='',
                              description='Required RealSense serial number.'),
        DeclareLaunchArgument('mounts_file',
                              default_value=str(share / 'config' / 'camera_mounts.yaml'),
                              description='Measured camera mount offsets.'),
        OpaqueFunction(function=_launch_setup),
    ])
