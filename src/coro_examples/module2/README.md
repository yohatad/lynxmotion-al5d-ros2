# C++ package to support Module 2 on writing ROS software in C++: publishers, subscribers, services.
This package implements the four examples in Module 2, Lecture 2, which involve the creation of a new package, agitr, with four ROS 2 nodes:

- hello
- pubvel
- subpose
- useservices

Please refer to Lecture 2 for details on the functionality of each of these nodes.

## Running the example code

ROS 2 needs no master process: there is no `roscore`. Every terminal needs ROS 2 and the workspace sourced (done automatically inside the Docker image).

Open a terminal and enter

`ros2 run module2 hello`

to see the Hello World message.

Then enter

`ros2 run turtlesim turtlesim_node`

Open a second terminal and enter

`ros2 run module2 pubvel`

to publish random linear and angular command velocities and see the turtle wander about the simulator environment.

Open a third terminal and enter

`ros2 run module2 subpose`

to see the pose values published on the turtleX/pose topic, where X stands for the turtle number.

Enter <ctrl>-c to stop the pubvel and subpose nodes.

Enter

`ros2 run module2 useservices`

to use the example services to clear the simulator and teleport the turtle.

The turtlesim window needs a display; see the main README for how to set one up in Docker.
