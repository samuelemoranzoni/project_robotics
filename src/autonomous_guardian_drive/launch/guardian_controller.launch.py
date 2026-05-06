from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("autonomous_guardian_drive")
    config_file = PathJoinSubstitution(
        [package_share, "config", "guardian_drive.yaml"]
    )

    namespace = LaunchConfiguration("name")
    scenario_level = LaunchConfiguration("scenario_level")
    metrics_path = LaunchConfiguration("metrics_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="rm0"),
            DeclareLaunchArgument("scenario_level", default_value="manual"),
            DeclareLaunchArgument(
                "metrics_path",
                default_value="logs/guardian_drive_metrics.csv",
            ),
            Node(
                package="autonomous_guardian_drive",
                executable="guardian_drive",
                name="guardian_drive",
                namespace=namespace,
                output="screen",
                parameters=[
                    config_file,
                    {
                        "scenario_level": scenario_level,
                        "metrics_path": metrics_path,
                    },
                ],
            ),
            Node(
                package="autonomous_guardian_drive",
                executable="scenario_report",
                name="scenario_report",
                namespace=namespace,
                output="screen",
                parameters=[config_file],
            ),
        ]
    )

