# hector_ros_controllers

Simple controllers for the [ros2_control](https://control.ros.org/jazzy/index.html) framework.

## Controllers

1. Safety Forward Controller: A simple forward controller that resets the command value to zero if no command is
   received within a specified timeout period.
2. Self Collision Avoidance Controller: A controller that prevents self-collision by checking the robot's state and if a
   nearby collision is detected, it makes sure to prevent the robot from moving into that state.
3. Safety Position Controller: Unwraps joint values for continuous joint such that the command position is in (-inf, inf)
   and closest to the current value. Normal joint positions are clamped to their limits.

# hector_controller_spawner

A lightweight ROS2 node that boots an entire *ros2_control* setup in a single shot, managing hardware interfaces and
controllers efficiently.
See [README.md](hector_controller_spawner/README.md) for details.
