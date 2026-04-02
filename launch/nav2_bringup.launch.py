"""
launch/nav2_bringup.launch.py
─────────────────────────────
Starts the full Nav2 navigation stack for the TurtleBot-like robot:
  - robot_state_publisher  (URDF → /tf)
  - map_server             (loads simple_map.yaml)
  - amcl                   (localisation)
  - nav2 navigation nodes  (planner, controller, behaviour tree …)
  - lifecycle managers

Usage (inside the ROS2 Docker container):
    ros2 launch /workspace/launch/nav2_bringup.launch.py

Optional arguments:
    use_sim_time:=true|false   (default: true)
    map:=<path>                (default: /workspace/maps/simple_map.yaml)
    params_file:=<path>        (default: /workspace/config/nav2_params.yaml)
    use_rviz:=true|false       (default: false)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

WORKSPACE = "/workspace"


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use /clock topic from Isaac Sim")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(WORKSPACE, "maps", "simple_map.yaml"),
        description="Path to the 2-D occupancy map YAML")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(WORKSPACE, "config", "nav2_params.yaml"),
        description="Path to the Nav2 params YAML")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Launch RViz2 for visualisation")

    # ── robot_state_publisher ─────────────────────────────────────────────────
    urdf_path = os.path.join(WORKSPACE, "urdf", "turtlebot_like.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_description": robot_description,
        }],
    )

    # ── Nav2 bringup (includes map_server, amcl, all navigation nodes) ────────
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time":  LaunchConfiguration("use_sim_time"),
            "map":           LaunchConfiguration("map"),
            "params_file":   LaunchConfiguration("params_file"),
            "use_lifecycle_mgr": "true",
            "use_composition": "false",
        }.items(),
    )

    # ── RViz2 (optional) ──────────────────────────────────────────────────────
    rviz_config = os.path.join(WORKSPACE, "config", "rviz2_nav2.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_arg,
        use_rviz_arg,
        rsp_node,
        nav2_launch,
        rviz_node,
    ])
