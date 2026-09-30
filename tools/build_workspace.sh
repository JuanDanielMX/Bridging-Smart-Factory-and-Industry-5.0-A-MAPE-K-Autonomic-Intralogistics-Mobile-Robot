#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash

chmod +x "$PROJECT_ROOT"/catkin_ws/src/autonomic_turtlebot3/scripts/*.py
chmod +x "$PROJECT_ROOT"/tools/*.sh "$PROJECT_ROOT"/tools/*.py

cd "$PROJECT_ROOT/catkin_ws"
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release

echo "Workspace built. Run: source $PROJECT_ROOT/catkin_ws/devel/setup.bash"
