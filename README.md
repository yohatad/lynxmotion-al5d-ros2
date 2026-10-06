# Robotics: Principles and Practice on ROS 2 Jazzy

The course software (iRobot Create 2, Lynxmotion AL5D arm, example programs for modules 2 to 5),
migrated from ROS 1 Noetic / Gazebo Classic to **ROS 2 Jazzy / Gazebo Harmonic** and packaged in
one Docker image. See [MIGRATION.md](MIGRATION.md) for what changed and why.

| Package | Contents |
|---|---|
| `lynxmotion_al5d_description` | AL5D model (xacro), Gazebo world, `ros2_control` setup, Lego-brick services, launch files |
| `coro_common` | small header-only helpers shared by the examples (`packagePath`, `dataDirectory`, `callService`, `spinOnce`) |
| `module2` | publisher, subscriber and service examples for `turtlesim` |
| `module3` | go-to-position (turtlesim) and go-to-pose (iRobot Create 2) |
| `module4` | AL5D inverse kinematics, robot programming and pick-and-place |
| `module5` | OpenCV vision examples, camera calibration, simulator camera |
| `module11` | Python tutorial on CoraPlex (successor of CRAM) controlling the AL5D; replaces the Lisp tutorial; optional image |
| `third_party/create_robot` | the Create 2 driver (cloned by the Dockerfile, `humble` branch, builds on Jazzy) |

## Quick start

```bash
docker compose build
docker compose run --rm rpp                 # shell as user rpp (password rpp)
docker compose run --rm test                # build + run the whole test suite
```

Inside the container (every command is `ros2 ...`; there is no `roscore`):

```bash
ros2 launch lynxmotion_al5d_description sim.launch.py                  # Gazebo simulation (GUI)
ros2 launch lynxmotion_al5d_description sim.launch.py headless:=true   # no GUI
ros2 launch lynxmotion_al5d_description display.launch.py              # model in RViz with sliders
ros2 launch lynxmotion_al5d_description sliders.launch.py              # sliders that move the Gazebo arm (sim running)

ros2 run lynxmotion_al5d_description spawn_brick -c red -x 0.1 -y 0.15
ros2 run lynxmotion_al5d_description kill_brick brick1
ros2 service call /lynxmotion_al5d/reset std_srvs/srv/Empty

ros2 run turtlesim turtlesim_node           # module 2 and 3
ros2 run module2 pubvel
ros2 run module3 goToPosition
ros2 run module4 pickAndPlace               # needs the simulator; see below
```

### Driving the simulated arm

The arm takes six values, `[Joint1 .. Joint5 (rad), Gripper (m)]`:

```bash
ros2 topic pub --once /lynxmotion_al5d/joints_positions/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 1.2, -1.2, 0.3, 0.2, 0.03]}"
```

Joint states are on `/lynxmotion_al5d/joint_states`, the overhead camera on
`/lynxmotion_al5d/external_vision/image_raw`, and every brick publishes its pose on
`/lynxmotion_al5d/<name>/pose`.

`module4` programs run on the real robot unless the configuration file says `SIMULATOR TRUE`.
`robot_simulator_config.txt` is the simulator variant of `robot_1_config.txt`. To use your own files
without editing the installed ones, point `CORO_DATA_DIR_MODULE4` at a directory that holds
`pickAndPlaceInput.txt` and the configuration it names.

## Graphical programs on Windows

The Gazebo GUI, RViz and the OpenCV windows need an X server. Install VcXsrv, start XLaunch
(multiple windows, display 0, **disable access control**), allow it in the Windows firewall and run
the container with the default `DISPLAY=host.docker.internal:0.0`. Rendering is in software and slow;
for speed run Docker inside WSL2 and use WSLg (`DISPLAY=:0`, mount `/tmp/.X11-unix` and `/mnt/wslg`).
On Linux use `DISPLAY=$DISPLAY` and mount `/tmp/.X11-unix`. All simulation tests run without a display.

## Physical robots and cameras

* Create 2: `ros2 launch create_bringup create_2.launch` (publishes `odom`, listens on `cmd_vel`;
  module 3 `goToPoseCreate` uses exactly these). The user `rpp` is in the `dialout` group.
* AL5D: `echo "#0P1610S250" > /dev/ttyUSB0` moves joint 0, as in the course instructions.
* Linux hosts: pass the devices with `--device /dev/ttyUSB0` / `--device /dev/video0`.
* Windows hosts: Docker Desktop cannot see USB devices. Use `usbipd-win`
  (`usbipd bind --busid <id>`, `usbipd attach --wsl --busid <id>`) and a WSL2 kernel that has the
  FTDI (`ftdi_sio`) and UVC drivers. This was **not** tested here: no robot was available.

## Tests

`scripts/run_tests.sh` builds and tests everything (`--no-sim` skips Gazebo; package names select).
Same through `docker compose run --rm test`.

| Level | Where |
|---|---|
| Unit | pose maths, CLI parsing, gripper mapping, Gazebo-pose conversion, kinematics, servo mapping, helpers |
| Component | brick manager against a fake simulator (gtest) and against fake Gazebo bridge services (launch test) |
| Description | xacro expands, URDF/SDF accepted by `gz sdf`, inertias physical, mesh files exist, controller names match |
| Integration | headless Gazebo: controllers activate, arm/fingers follow commands, camera streams, bricks spawn, fall, teleport, are removed; `pickAndPlace` runs end to end |
| Programs | module 2 against real turtlesim, module 3 against turtlesim and a simulated Create 2, module 5 programs under a virtual display |
| Static | flake8, pep257, cppcheck, cmake and XML lint on the new code |

## Known limitations

* **Simulated grasping**: the arm reaches the brick, but the AL5D model's fingers do not reliably
  pick it up (see MIGRATION.md, "Simulated grasping"). Pick-and-place tests assert that the program
  runs and moves the arm, not that the brick arrives.
* Physical robot access from Docker on Windows is untested (see above).
* `module11` is a rewrite, not a port: the Lisp CRAM it was written for is archived. It is now a Python tutorial on CoraPlex (see `src/coro_examples/module11/README.md`), in its own optional image (`docker compose --profile cram ...`), because CoraPlex adds several GB of dependencies.
* `robotCameraModelDataSimulator` of module 5 was already disabled in the original build and is not shipped.

## GPU rendering (NVIDIA, optional)

The default service renders in software (`LIBGL_ALWAYS_SOFTWARE=1`), which is slow for Gazebo. The `rpp-gpu`
service uses an NVIDIA GPU. On WSL 2 (Windows 11, NVIDIA RTX 4070 laptop GPU) the renderer was confirmed to be the
NVIDIA GPU through D3D12 (`glxinfo`); the speed of the Gazebo GUI was not measured. The Linux host variant was written
from the Docker documentation and **not tested**. Docker Desktop on Windows with VcXsrv cannot use the GPU for
rendering; use WSL 2 (WSLg) or Linux.

```bash
# Linux host (NVIDIA driver + NVIDIA Container Toolkit)
xhost +local:docker
DISPLAY=$DISPLAY docker compose --profile gpu run --rm rpp-gpu
# WSL 2 on Windows 11 (NVIDIA driver on Windows), run from the WSL terminal.
# Mesa must be told to use the Windows GPU driver (confirmed to give "D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)"):
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
docker compose --profile gpu run --rm rpp-gpu
# inside: check the renderer, then start the simulation with its GUI
glxinfo -B | grep -i renderer      # must not say llvmpipe
ros2 launch lynxmotion_al5d_description sim.launch.py
```

### More terminals in a running container
`docker compose run` gives the container a generated name, so find it first:
```bash
docker exec -it $(docker ps -q --filter ancestor=rpp-jazzy | head -1) bash
```
