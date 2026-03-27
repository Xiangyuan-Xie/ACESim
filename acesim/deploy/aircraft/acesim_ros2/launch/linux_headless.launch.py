from acesim_ros2.launch_common import build_linux_launch_entities, load_px4_repo_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    override = LaunchConfiguration("px4_repo").perform(context)
    return build_linux_launch_entities(load_px4_repo_path(override), play_executable="acesim_play_headless")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_repo",
                default_value="",
                description="PX4-Autopilot repository path; if empty, auto-detect from ACESim",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
