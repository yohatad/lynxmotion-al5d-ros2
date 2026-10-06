#!/bin/bash
# Entry point of the CoraPlex (module 11) image.
#   entrypoint_cram.sh test        run the module 11 tests
#   entrypoint_cram.sh tutorial    pick and place in CoraPlex (add --simulator for Gazebo)
#   entrypoint_cram.sh sim         start the headless Gazebo simulation
#   entrypoint_cram.sh <command>   anything else is executed as given
set -e
source /opt/ros/jazzy/setup.bash
source /home/rpp/ros2_ws/install/setup.bash
source /home/rpp/workspace/install/setup.bash
source /home/rpp/cram-env/bin/activate

case "$1" in
  test)
    shift
    cd /home/rpp/module11
    exec python -m pytest tests -v "$@"
    ;;
  tutorial)
    shift
    exec python -m module11.pick_and_place "$@"
    ;;
  sim)
    shift
    exec ros2 launch lynxmotion_al5d_description sim.launch.py headless:=true "$@"
    ;;
  *)
    exec "$@"
    ;;
esac
