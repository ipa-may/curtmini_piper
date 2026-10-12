"""Localize on a saved map and serve NavigateToPose goals in simulation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_navigation(context):
    map_file = Path(LaunchConfiguration("map").perform(context))
    if not map_file.is_file():
        raise RuntimeError(
            f"Map does not exist: {map_file}. Save a map with mapping-sim first; "
            "see curtmini_piper_navigation/README.md."
        )
    share = Path(get_package_share_directory("curtmini_piper_navigation"))
    parameters = [str(share / "config/nav2.yaml"), {"use_sim_time": True}]
    servers = [
        ("nav2_amcl", "amcl"),
        ("nav2_controller", "controller_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ]
    return [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=parameters + [{"yaml_filename": str(map_file)}]),
        *[
            Node(package=package, executable=name, name=name, output="screen",
                 parameters=parameters,
                 remappings=[("cmd_vel", "/base_controller/cmd_vel")])
            for package, name in servers
        ],
        *[
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name=f"lifecycle_manager_{group}", output="screen", parameters=[{
                     "use_sim_time": True,
                     "autostart": True,
                     "node_names": names,
                 }])
            for group, names in (
                ("localization", ["map_server", "amcl"]),
                ("navigation", [name for _, name in servers if name != "amcl"]),
            )
        ],
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="/maps/map.yaml",
                              description="Absolute path to the saved occupancy map YAML."),
        OpaqueFunction(function=launch_navigation),
    ])
