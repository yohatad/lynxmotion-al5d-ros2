# Migration notes: ROS 1 Noetic to ROS 2 Jazzy

Source: the course install guide (Ubuntu 20.04, ROS Noetic, Gazebo Classic) with
`lynxmotion_al5d_description` and `coro_examples` from the `cognitive-robotics-course` organisation.
Target: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz sim 8), in Docker.

## 1. Command and API mapping

| ROS 1 | ROS 2 here |
|---|---|
| `roscore` | not needed |
| `rosrun pkg node` / `roslaunch pkg f.launch` | `ros2 run pkg node` / `ros2 launch pkg f.launch.py` |
| `rostopic`, `rosservice`, `rosparam` | `ros2 topic`, `ros2 service`, `ros2 param` |
| `catkin_make`, `devel/setup.bash` | `colcon build`, `install/setup.bash` |
| `roscpp` (`ros::NodeHandle`, `Rate`, `spinOnce`) | `rclcpp` (`Node`, `Rate`, `coro_common::spinOnce`) |
| `ros::package::getPath` | `coro_common::packagePath` / `dataDirectory` (ament index) |
| `ServiceClient::call` / `waitForService` | `coro_common::callService` / `waitForService` |
| `.launch` XML, `rosparam`, `robot_description` param | Python launch files, `robot_state_publisher` |
| Gazebo Classic + `gazebo_ros_control` + mimic plugin | gz sim + `gz_ros2_control` + `command_relay` |
| `/gazebo/spawn_sdf_model`, `delete_model`, `model_states` | `/world/al5d/create`, `remove`, `set_pose` (bridged) and `gz_pose_relay` |

### Interfaces that changed

| ROS 1 | ROS 2 |
|---|---|
| `/lynxmotion_al5d/joints_positions/command` | `/lynxmotion_al5d/joints_positions/commands` (same six values) |
| `lynxmotion_al5d_description/SpawnBrick` response `{name}` | adds `success`, `message` (failures are reported instead of dropping the call) |
| `TeleportAbsolute/Relative` empty response | `success`, `message` |
| joint states: gripper first, then joints | joints are looked up **by name** everywhere |
| brick poses from `/gazebo/model_states` | published at 10 Hz on `/lynxmotion_al5d/<name>/pose`, as before |
| `reset` / `clear` | unchanged (`std_srvs/Empty`), `reset` now waits at most 5 s for the controller |

## 2. New components

* **`command_relay`**: Gazebo has no mimic joints (DART lacks mimic constraints, the mimic plugin of
  Classic does not exist, bullet-featherstone cannot load a model with two root trees). The relay turns the six-value
  command into the eight values of the ros2_control controller (adds both finger joints, using the same relations
  as the URDF mimic tags, clamped to the joint limits). The `<mimic>` tags stay in the URDF for RViz and are removed
  only for the simulator (`use_sim:=true`).
* **`gz_pose_relay`**: `ros_gz_bridge` converts `Pose_V` to `TFMessage` but leaves every `child_frame_id` empty
  for this topic, so brick poses could not be matched to bricks. The relay subscribes with `gz-transport` and
  fills in the entity names.
* **`coro_common`**: the ROS 1 idioms the course code relies on, with the behaviour of the originals (see 3a).
* `CORO_DATA_DIR_<PACKAGE>` environment variable to point a program at its own data directory.

## 3. Defects found while migrating (each has a test)

a. **`spin_some` is not `spinOnce`.** ROS 1 `spinOnce()` runs every queued callback; `rclcpp::spin_some()` handles about
   one message per call. A 50 Hz control loop on a 62.5 Hz pose topic fell behind without bound and steered on stale
   poses (turtle oscillating, never arriving). `coro_common::spinOnce` drains the queue (`test_spin`).
b. **Maximum velocity never enforced** in `goToPoseMIMO1` (module 3): the clamp was stored in `msg.linear.x` and then
   overwritten with the unclamped value. Fixed; a simulated Create 2 checks the 0.2 m/s limit.
c. **`robot_support` inertia was not positive definite** (off-diagonal terms larger than the diagonal); Gazebo
   Harmonic refuses such a model. Replaced by the inertia of a box of the same mass (`test_description`, `gz sdf`).
d. **Brick pose subscription per brick** in `manager.cpp` (one subscription per brick, each calling `spinOnce`
   inside its own callback) and an infinite busy-wait for subscribers. Now one subscription, bounded waits, locks.
e. **Brick colours** used `Gazebo/Blue` material scripts that gz sim ignores (bricks rendered grey); now explicit colours.
f. **Unbounded busy-wait** `while (getNumSubscribers() < 1)` in module 4 and 5 arm publishing; now a 10 s bounded wait.
g. `signnum()` fell off the end of a non-void function; `main` without a return type; `%s` into a 13 byte buffer left as is.
h. **`COM4` file in the brick model directory**: a file named like a Windows device made the repository impossible to
   check out on Windows. It only held a stray servo command string and is not shipped.
i. `Media/bricks.png` is named in `cameraInvPerspectiveMonocularInput.txt` but missing from the repository (recorded
   as a known gap in the module 5 test).
j. Programs crashed on Ctrl-C or shutdown (globals destroyed after the ROS context); all mains now release ROS objects
   in order and treat an interrupt as a normal exit.

## 4. Simulated grasping (known limitation)

The pick-and-place programs drive the arm to the brick correctly (joint tracking error below 0.01 rad; the commanded
and measured poses agree), but the simulated gripper does not reliably lift the brick:

* In the ROS 1 model the finger plates overlap at the 15 mm closing distance the programs use for a 15.8 mm brick
  (gap = `Gripper` - 13.5 mm). The finger joint origins were therefore moved outward (`finger_origin_x`, 5 mm to 18.3 mm)
  so that the gap equals the `Gripper` value, as the programs assume, and the finger force limit was reduced from 100 N to 1 N
  (the brick was otherwise thrown away at 4 m/s).
* The fingertips of the modelled gripper hang about 25 mm higher than the 105 mm effector length in the shipped
  configurations assumes, so a pick needs an `EFFECTOR` value around 60 to 75 mm for the simulator. A sweep of that value
  did not give a reliable grasp, so the shipped `robot_simulator_config.txt` keeps the real-robot value.

Suspected causes, none of them confirmed: finger and brick not aligned when the gripper closes; the effector length
mismatch above; contact physics of a small brick between two thin fingers; too little finger friction; the gripper
closing or the arm lifting before the arm has settled. A diagnostic test (close, lift, log brick pose and finger
positions) would tell them apart.

This needs tuning of the gripper model (collision shapes, contact parameters) against a known-good reference, which was
not available. The tests therefore check what is verified: the program runs, the arm moves to the brick, the brick is
spawned at the pick pose and removed afterwards.

## 5. Not migrated

* The Lisp `module11` (CRAM tutorials) was not ported line by line: it was rewritten for CoraPlex (section 7).
* The ROS 1 `rviz` configuration was kept and loads in RViz 2; the dashboard-style display was not redesigned.
* `ros1_bridge` does not exist for Jazzy, so ROS 1 nodes cannot be mixed with this workspace.

## 6. Test isolation (found by running the whole suite in a clean image)

Gazebo discovers its peers by `GZ_PARTITION`, not by `ROS_DOMAIN_ID`. Gazebo takes longer to exit than the launch
system waits (5 s), so a server from one simulation test could still be alive when the next test started and mix
its bricks into the new world (one test saw a brick that another test had spawned). The simulation tests now run
with their own `GZ_PARTITION` and remove leftover servers afterwards. If you run the simulator by hand and see a
brick you did not spawn, stop the old simulator first: `pkill -f "gz sim"`.

## 7. Module 11: rewritten for CoraPlex (the Lisp CRAM is archived)

The ROS 1 `module11` was a Common Lisp package for the original CRAM (`cram2/cram`, archived April 2026, ROS 1
and `roslisp`). Its successor is the Python monorepo `cram2/cognitive_robot_abstract_machine`, whose planning
framework is CoraPlex. It targets Python 3.12 and ROS 2 Jazzy (its workspace script hardcodes
`/opt/ros/jazzy`), so it fits this migration, but the API is new and nothing is source-compatible with the Lisp.

`src/coro_examples/module11` is a new Python tutorial built on CoraPlex (pinned: CRAM 26.07.0, commit `0c5a90f`):
designators and plans for the AL5D (`MoveWristAction`, `PickAction`, `PlaceAction`, `PickAndPlaceAction`,
`DemoAction`), a custom motion (`PreciseMoveJointsMotion`), the AL5D kinematics ported from module 4, and a bridge
that replays the steps on the Gazebo simulator. Verified: 37 tests; the plan's joint values match what the C++
`pickAndPlace` sends; the mirrored plan moved the Gazebo arm through all 9 steps of a pick and place.

Findings that shaped it:

* CoraPlex' URDF parser needs an explicit `<axis>` on every moving joint. `right_finger_joint` relied on the URDF
  default, so the axis is now written out in `lynxmotion_al5d.xacro` (same behaviour).
* `MoveJointsMotion` stops when a joint is within 0.01 (rad or m); the gripper travels only 0.032 m, so a 15 mm
  request stopped near 7 mm. `PreciseMoveJointsMotion` takes a tolerance (0.001).
* The module 4 inverse kinematics does not check joint limits. For a gripper pointing down the wrist pitch
  exceeds its 90 degree limit above a wrist height of about 200 mm; the Python version raises `UnreachablePose`.
* A gripper pointing down is pitch -180 degrees in the C++ code, and the gripper roll is the negative of the
  brick orientation.
* CoraPlex builds a whole plan before it runs the motions, so code inside a plan cannot observe intermediate
  robot states; the bridge therefore runs the steps one at a time.
* The CoraPlex world is kinematic: it has no bricks, so grasping is not simulated there.

## 8. Sliders for the Gazebo arm (new, no ROS 1 counterpart)

`ros2 launch lynxmotion_al5d_description sliders.launch.py` (simulator running) opens a `joint_state_publisher_gui`
window, remapped to `/arm_sliders/joint_states`, plus the node `slider_command`, which turns a slider change into a
six-value command on `/lynxmotion_al5d/joints_positions/commands`. (`display.launch.py` still shows the model in RViz
only; it does not move the Gazebo arm.)

* The window publishes its start values at once and then repeats them every 0.1 s, which would throw the arm to the
  window's zero pose. `SliderMapper` (`slider_mapping.hpp`) treats the first valid message as a baseline and publishes
  only when a joint differs from the previous message.
* Joints are matched by name, values are clamped to the joint ranges, and incomplete or non-finite messages are
  ignored without resetting the baseline.
* The sliders show what was set, not the arm's current pose; after `reset` or a command from another program they
  no longer match the arm.
* Tests: 9 unit tests (`test_slider_mapping`) and `test_10_slider_changes_move_the_arm` in the simulation test. The
  slider window itself was not opened in any test. The simulation test failed once in a run under load and passed on
  two reruns of the same code; the cause of that failure was not found.
