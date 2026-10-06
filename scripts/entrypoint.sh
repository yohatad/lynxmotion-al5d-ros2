#!/bin/bash
# Source ROS 2 and the workspace, then run the requested command.
#   entrypoint.sh test        run the full test suite (see run_tests.sh)
#   entrypoint.sh sim         start the headless simulation
#   entrypoint.sh <command>   anything else is executed as given
set -e
source /opt/ros/jazzy/setup.bash
source /home/rpp/workspace/install/setup.bash

case "$1" in
  test)
    shift
    exec /home/rpp/run_tests.sh "$@"
    ;;
  sim)
    shift
    exec ros2 launch lynxmotion_al5d_description sim.launch.py headless:=true "$@"
    ;;
  *)
    exec "$@"
    ;;
esac
