#!/bin/bash
# Build and test the whole workspace and print a summary. Exit status is non-zero on any failure.
#
#   run_tests.sh                       run everything (unit, integration, Gazebo simulation, lint)
#   run_tests.sh --no-sim              skip the tests that start Gazebo
#   run_tests.sh module3 module5       only these packages
#
WORKSPACE="${WORKSPACE:-/home/rpp/workspace}"
cd "$WORKSPACE"
source /opt/ros/jazzy/setup.bash

sim=ON
packages=()
for arg in "$@"; do
  case "$arg" in
    --no-sim) sim=OFF ;;
    -*) echo "unknown option $arg" >&2; exit 2 ;;
    *) packages+=("$arg") ;;
  esac
done

select=()
if [ ${#packages[@]} -gt 0 ]; then
  select=(--packages-select "${packages[@]}")
fi

colcon build "${select[@]}" --symlink-install --event-handlers console_cohesion- \
  --cmake-args -DBUILD_TESTING=ON -DAL5D_ENABLE_SIM_TESTS=$sim -DMODULE4_ENABLE_SIM_TESTS=$sim || exit 1
source install/setup.bash

# One package at a time, so that the simulations of different packages never overlap.
colcon test "${select[@]}" --packages-skip-regex "^create_" --executor sequential --event-handlers console_cohesion- \
  --return-code-on-test-failure
status=$?

colcon test-result --verbose
exit $status
