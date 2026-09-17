#!/usr/bin/env bash
set -e
C920_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
if [[ ! -f "$C920_PROJECT_ROOT/ros2_ws/install/c920_recorder/share/c920_recorder/package.bash" ]]; then
  echo 'Build c920_recorder in ros2_ws first.' >&2
  exit 1
fi
source "$C920_PROJECT_ROOT/ros2_ws/install/setup.bash"
exec ros2 run c920_recorder record "$@"
