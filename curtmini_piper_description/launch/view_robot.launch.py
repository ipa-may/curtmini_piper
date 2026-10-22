"""Preview the assembled robot with joint sliders in RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("curtmini_piper_description")
    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "curtmini_piper.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "view_robot.rviz"]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " simulation:=False use_hokuyo:=",
                LaunchConfiguration("use_hokuyo"),
                " arm_model:=",
                LaunchConfiguration("arm_model"),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_hokuyo", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "arm_model", default_value="piper_x", choices=["piper", "piper_h", "piper_l", "piper_x"]
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
