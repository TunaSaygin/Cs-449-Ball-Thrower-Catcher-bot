# Ball Thrower and Catcher Robot Project

## Overview

This project involves the development of a ball-throwing and catching robotic system. The system utilizes the Robotic library (`ry`) for simulation and control, and includes a thrower robot that calculates optimal trajectories and velocities to throw a ball, and a catcher robot that dynamically adjusts its position to catch the thrown ball.

The project demonstrates advanced robotics concepts such as:
- Trajectory optimization
- Kinematics and dynamics simulation
- Multi-robot coordination
- Object tracking and manipulation

## Features

### **Thrower Robot**
- Calculates optimal throwing velocity using kinematic and dynamic constraints.
- Simulates ball trajectories based on gravity and initial release parameters.
- Handles inversion scenarios where the bin position requires trajectory flipping.

### **Catcher Robot**
- Uses a vision-based system to capture ball trajectories.
- Dynamically updates its position to catch the thrown ball.

### **Path Finding**
- Check if thrower is too far/close to bin or catcher.
- Use RRT to move the thrower robot to optimal

### **Simulation Environment**
- Built with the Robotic library (`ry`).
- Supports visualization of trajectories and robot movements.
- Provides debugging markers to track robot states and ball positions.

## Project Structure

### **Core Components**

#### `thrower_generic.py`
Contains functions for the thrower robot, including:
- `find_velocity`: Calculates the velocity required to reach the bin.
- `pick_last_object_if_valid`: Determines the initial position of the ball for a valid throw.
- `grasp_object`: Simulates the robot grasping the ball.
- `throw_object`: Simulates the throwing action and updates the trajectory.
- `check_in_the_bin`: Verifies if the ball lands in the target bin.

#### `catcher_robot.py`
Defines the catcher robot's logic, including:
- Vision-based point cloud capture.
- Position adjustment to intercept the ball's trajectory.

#### `thrower_catcher_integration.py`
Integrates the thrower and catcher robots, simulating their interaction in a shared environment.
- Coordinates throwing and catching actions using threading.
- Visualizes the simulation in real-time.

#### Utility Scripts
- `my_utils.py`: Contains helper functions for quaternion and rotation calculations.
- `velocity_finder.py`: Implements trajectory optimization and velocity calculation algorithms.

### **Data Files**
- `catching.g`: Simulation configuration for the catcher robot.
- `throwing_bare.g`: Simulation configuration for the thrower robot.


### Documentation 
- To see the documentation open branch doc and initialize index.html.
