# Module 11: Task-level robot programming with CoraPlex

This tutorial controls the Lynxmotion AL5D arm with **CoraPlex**, the planning framework of the
[cognitive_robot_abstract_machine](https://github.com/cram2/cognitive_robot_abstract_machine)
monorepo. It replaces the ROS 1 tutorial written in Common Lisp for the original CRAM
(`cram_lynxmotion_al5d_tutorials`): the Lisp CRAM repository is archived, and its successor is a
Python rewrite for ROS 2 Jazzy. The ideas are the same (designators, plans, process modules), the
language and the API are not.

Version used: CRAM 26.07.0, commit `0c5a90f` (pinned in `docker/Dockerfile.cram`).

## What you learn

1. **Designators**: describe *what* the robot should do (`PickAction(x, y, z)`), and let the
   framework decide *how* at run time.
2. **Motions and actions**: a motion is the lowest level and builds one goal for the motion
   controller; an action builds a plan out of motions and other actions.
3. **Plans**: combine actions with `sequential`, `parallel`, `repeat`, ...; the plan can be inspected
   before it runs.
4. **A world model**: plans run on a model of the robot (loaded from the course's URDF), so you can
   develop without any simulator, then show the same steps in Gazebo.
5. **Kinematics matter**: a pose is only a goal if the arm can reach it within its joint limits.

## From Lisp CRAM to CoraPlex

| Lisp tutorial | This module |
|---|---|
| `(a motion (type moving) (destination ?pose))` | `MoveWristAction(x, y, z, pitch, roll)`, built on `PreciseMoveJointsMotion` |
| `(a motion (type grasping) (distance ?d))` | `SetGripperDistanceAction(distance)` |
| `(an action (type approaching) (at ?pose))` | `ApproachAction(x, y, z, phi)` |
| `(an action (type picking) (from ?pose))` | `PickAction(x, y, z, phi)` |
| `(an action (type placing) (to ?pose))` | `PlaceAction(x, y, z, phi)` |
| `(an action (type picking-and-placing) ...)` | `PickAndPlaceAction(...)` |
| `(an action (type demoing))` | `DemoAction()` |
| process module `lynxmotion_al5d_navigation` publishing to ROS 1 | the world model runs the plan; `ros2_bridge.ArmMirror` forwards it to Gazebo |
| joint-state fluents | `al5d.joint_positions(world)` |

## Files

```
module11/kinematics.py   inverse and forward kinematics of the arm (Python port of module 4)
module11/al5d.py         load the AL5D into a CoraPlex world, make the robot and the plan context
module11/motions.py      a custom motion designator: PreciseMoveJointsMotion
module11/actions.py      the action designators and their plans
module11/ros2_bridge.py  send the steps to the Gazebo simulator and wait for the arm
module11/pick_and_place.py  command-line program
tests/                   kinematics, actions and bridge tests
```

## Running it

CoraPlex is large, so it has its own image (Section "Installation" of the manual lists the
manual alternative):

```bash
docker compose build rpp
docker compose --profile cram build cram          # about 30 minutes, several GB
docker compose --profile cram run --rm cram test       # module 11 tests
docker compose --profile cram run --rm cram tutorial   # pick and place in the CoraPlex world
```

With the Gazebo simulator running (in another terminal:
`docker compose --profile cram run --rm cram sim`), show the same plan on the simulated arm:

```bash
docker compose --profile cram run --rm cram tutorial --simulator
```

Options: `--source X Y Z`, `--destination X Y Z` (millimetres), `--source-phi`, `--destination-phi`
(brick orientation, degrees).

## Walk-through

### 1. A world with the AL5D

```python
from module11.al5d import make_context, joint_positions

context, world = make_context()          # parses the URDF of lynxmotion_al5d_description
print(joint_positions(world))            # all zeros: the arm in its model's zero configuration
```

`make_context()` returns the *context* every plan needs (world and robot) and the world itself.

### 2. Run a motion

```python
from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import execute_single
from module11.actions import MoveWristAction

with simulated_robot:
    execute_single(MoveWristAction(0, 200, 150), context=context).perform()
print(joint_positions(world))
```

`MoveWristAction` converts the wrist position into joint angles with the inverse kinematics and
asks the motion controller to move the five joints there.

### 3. Compose a plan

```python
from coraplex.plans.factories import sequential
from module11.actions import PickAction, PlaceAction

plan = sequential([PickAction(-60, 175, 0, phi=90), PlaceAction(60, 175, 0)], context=context)
with simulated_robot:
    plan.perform()
```

`PickAction` is itself a plan: open the gripper, approach, lower, close, rise. Look at
`PickAction(...).steps()` to see the five steps it consists of.

### 4. Write your own motion

`motions.py` is only 20 lines of code. The built-in `MoveJointsMotion` considers a joint "reached"
within 0.01 (rad or m). For the gripper, whose whole range is 0.032 m, that is far too coarse: a
request for 15 mm stops after 7 mm. `PreciseMoveJointsMotion` is the same motion with a tolerance you
choose. Change `DEFAULT_TOLERANCE` and watch the gripper test fail.

### 5. Show it in Gazebo

`ros2_bridge.run_mirrored` runs the steps one at a time and, after each, sends the joint values to
`/lynxmotion_al5d/joints_positions/commands`, then waits for `/lynxmotion_al5d/joint_states` to
report the arm there. This is what the process module of the Lisp tutorial did.

## Exercises

1. Add `ReturnHomeAction`, which moves the arm to the pose of the home configuration of module 4.
2. Make `PickAction` fail with a clear message when the brick is outside the working envelope, and
   write the test.
3. Stack two bricks: a `StackAction` that picks from two places and puts the second on the first
   (grasp height 5 mm, brick height 9.6 mm).
4. Use `parallel` to open the gripper while the arm approaches. Does the order of the steps still
   hold? Why does `steps()` not describe a parallel plan?
5. Replace the inverse kinematics of `kinematics.py` by one computed numerically (for example
   with the world model's Jacobian) and compare the joint angles with the closed-form ones.

## Things to know

* **Joint limits.** The gripper cannot point straight down when the wrist is higher than about 200 mm:
  the wrist pitch would exceed its limit of 90 degrees. `inverse_kinematics` raises `UnreachablePose`
  with the joint and the angle it would need. (The C++ inverse kinematics of module 4 does not check
  this; the simulator silently stops at the limit.)
* **Brick orientation.** The gripper roll is the negative of the brick orientation `phi`, as in the
  commands the module 4 program sends.
* **Plan building and execution are separate.** CoraPlex builds the whole plan first, then runs the
  motions. A step in the middle of a plan cannot read the robot state to decide what to do next; run
  the steps separately (as `run_mirrored` does) when you need that.
* **No grasp physics.** The CoraPlex world is a kinematic model: it has no bricks. Whether the
  simulated Gazebo gripper holds a brick is a separate (known) limitation described in `MIGRATION.md`.
* **Parser requirement.** CoraPlex requires an explicit `<axis>` on every moving joint. The URDF of
  `lynxmotion_al5d_description` now states the axis of `right_finger_joint`, which previously
  relied on the URDF default.
