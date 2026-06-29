from acesim_ros2.launch_common import build_launch_entities, load_px4_repo_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    override = LaunchConfiguration("px4_repo").perform(context)
    ace_follower = LaunchConfiguration("ace_follower").perform(context)
    return build_launch_entities(
        load_px4_repo_path(override),
        bridge_mode="linux",
        play_executable="acesim_play",
        enable_px4_post_start_setup=True,
        ace_follower=ace_follower,
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_repo",
                default_value="",
                description="PX4-Autopilot repository path; if empty, auto-detect from ACESim",
            ),
            DeclareLaunchArgument(
                "ace_follower",
                default_value="auto",
                description="Start the ACETele-compatible simulated ACEFollower shim: auto, true, or false",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
