# Example code for Module 5 of the Cognitive Robotics course (ROS 2 Jazzy)

## Table of contents
1. [External dependencies](#external-dependencies)
2. [Installation](#installation)
3. [Running the examples](#running-the-examples)


## External dependencies

### Build dependencies

This module uses OpenCV 4 (`libopencv-dev` from Ubuntu 24.04, together with the ROS 2 `cv_bridge` and
`image_transport` packages) and ncurses. They are declared in `package.xml`, so `rosdep` installs them:

```
rosdep install --from-paths src --ignore-src -r -y
```

or by hand:

```
sudo apt-get install libopencv-dev libncurses-dev
```

### Run-time dependencies

The `imageAcquisitionFromSimulatorCamera` and `moveRobot` programs need the simulator, so the
[Lynxmotion AL5D robot description](https://github.com/cognitive-robotics-course/lynxmotion_al5d_description)
package (`lynxmotion_al5d_description` in this workspace) must be built and its simulation running.
The programs open OpenCV windows and take key presses in the terminal, so they need a display.

## Installation
This package is part of the `coro_examples` workspace. In the Docker image it is already built. To build it
yourself, from the root of the workspace:
```
colcon build --packages-select module5
source install/setup.bash
```

To pick up later changes, pull the repository and run the same two commands again.

## Running the examples
Every program is started with `ros2 run`, for example:
```
ros2 run module5 binaryThresholding
ros2 run module5 cannyEdgeDetection
ros2 launch lynxmotion_al5d_description sim.launch.py      # terminal 1, for the simulator programs
ros2 run module5 imageAcquisitionFromSimulatorCamera       # terminal 2
```
Press a key in the terminal to advance to the next image. The input and image files are found in the package
data directory; set `CORO_DATA_DIR_MODULE5` to use your own copy.

`robotCameraModelDataSimulator` was already disabled in the original build and is not shipped.
