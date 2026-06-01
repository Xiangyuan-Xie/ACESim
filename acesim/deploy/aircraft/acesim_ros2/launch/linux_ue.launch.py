from acesim_ros2.launch_common import build_launch_entities, load_px4_repo_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    override = LaunchConfiguration("px4_repo").perform(context)
    ue_mode = LaunchConfiguration("ue_mode").perform(context)
    ue_executable = LaunchConfiguration("ue_executable").perform(context)
    unreal_editor = LaunchConfiguration("unreal_editor").perform(context)
    ue_project = LaunchConfiguration("ue_project").perform(context)
    return build_launch_entities(
        load_px4_repo_path(override),
        bridge_mode="linux",
        play_executable="acesim_play_ue",
        enable_px4_post_start_setup=True,
        additional_play_env={
            "ACESIM_UE_MODE": ue_mode,
            "ACESIM_UE_EXECUTABLE": ue_executable,
            "ACESIM_UNREAL_EDITOR": unreal_editor,
            "ACESIM_UE_PROJECT": ue_project,
        },
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
                "ue_mode",
                default_value="package",
                description=(
                    "UE launch mode: package for daily ROS2 use, editor for explicit development checks. "
                    "Editor mode runs UnrealEditor <uproject> -game and may compile shader/DDC data."
                ),
            ),
            DeclareLaunchArgument(
                "ue_executable",
                default_value="auto",
                description=(
                    "Packaged ACESimUE executable path. In package mode, auto means "
                    "/home/xxy/ACESim-unreal/packages/ACESimUE-Linux/Linux/ACESimUE/Binaries/Linux/ACESimUE"
                ),
            ),
            DeclareLaunchArgument(
                "unreal_editor",
                default_value="/home/xxy/ACESim-unreal/UnrealEngine/Engine/Binaries/Linux/UnrealEditor",
                description="UnrealEditor path used only when ue_mode:=editor",
            ),
            DeclareLaunchArgument(
                "ue_project",
                default_value="/home/xxy/ACESim/acesim/third_party/unreal/ACESimUE/ACESimUE.uproject",
                description="ACESimUE .uproject path used only when ue_mode:=editor",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
