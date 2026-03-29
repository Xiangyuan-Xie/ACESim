from acesim_ros2.launch_common import build_launch_entities, load_px4_repo_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    override = LaunchConfiguration("px4_repo").perform(context)
    return build_launch_entities(
        load_px4_repo_path(override),
        bridge_mode="linux",
        play_executable="acesim_play_headless",
        enable_px4_post_start_setup=True,
    )


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
