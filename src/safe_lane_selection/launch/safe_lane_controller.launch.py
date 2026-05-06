from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("safe_lane_selection")
    config_file = PathJoinSubstitution([package_share, "config", "safe_lane_selection.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="rm0"),
            DeclareLaunchArgument("use_lateral_velocity", default_value="false"),
            DeclareLaunchArgument("enable_object_detection", default_value="true"),
            DeclareLaunchArgument("enable_safe_lane_selection", default_value="false"),
            Node(
                package="safe_lane_selection",
                executable="safe_lane_node",
                name="safe_lane_node",
                namespace=LaunchConfiguration("name"),
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_lateral_velocity": ParameterValue(
                            LaunchConfiguration("use_lateral_velocity"),
                            value_type=bool,
                        ),
                        "enable_object_detection": ParameterValue(
                            LaunchConfiguration("enable_object_detection"),
                            value_type=bool,
                        ),
                        "enable_safe_lane_selection": ParameterValue(
                            LaunchConfiguration("enable_safe_lane_selection"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
