#!/usr/bin/env bash
# docker/entrypoint.sh
# Sources ROS2 setup and exec's the CMD.
set -e
source /opt/ros/humble/setup.bash
exec "$@"
