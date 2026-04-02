"""
launch/full_system.launch.py
─────────────────────────────
Convenience launcher: Nav2 stack + RViz2.

Usage:
    ros2 launch /workspace/launch/full_system.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORKSPACE = "/workspace"
NAV2_LAUNCH = os.path.join(WORKSPACE, "launch", "nav2_bringup.launch.py")


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(NAV2_LAUNCH),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz":     "true",   # override default to show RViz2
        }.items(),
    )

    return LaunchDescription([use_sim_time_arg, nav2])
