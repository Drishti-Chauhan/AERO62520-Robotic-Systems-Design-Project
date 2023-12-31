# AERO62520: LEO ROBOTICS SYSTEMS DESIGN PROJECT

**Maintainer:** Team 2: [Marcell](mailto:marcell.westsik@postgrad.manchester.ac.uk),[Maggie](ruyi.zhang-3@student.manchester.ac.uk),[Yining](mailto:yining.wang-17@postgrad.manchester.ac.uk),and[Drishti](mailto:drishtichauhanpvt@gmail.com)



The Leo Rover with PincherX-150 ROS package integrates the Leo Rover mobile robot and PincherX-150 robotic arm, providing a comprehensive set of nodes for controlling and managing both the robot's mobility and the arm's manipulation capabilities. This package facilitates seamless communication and coordination between different nodes for a unified robotic system.

# Table of Contents

[Features](#features)

[Node](#node)

[Install](#install)

[Folder Structure](#folder-structure)

[Contributing](#contributing)

[License](#license)

[Contact](#Contact)

# Features

* **Localisation and Mapping**: Simultaneous Localization and Mapping (SLAM) capabilities and integration with popular SLAM algorithms.

* **Navigation**: Path planning algorithms and obstacle avoidance.

* **Sensors and Noise Models**: Generate sensor data, optionally with noise,
from LIDAR sensor, heat cameras, depth sensors.


* **Communication**: ROS node communication for seamless integration with other ROS packages.


* **Simulation models**: Integration with ROS Gazebo for simulation and use launch files for simulation environments.

* **Safety**: Collision detection and prevention and
Emergency stop functionality.

## Nodes

### 1. leo_ros_controller

This node is responsible for controlling the Leo Rover's chassis motion, including wheel speed and direction.

### 2. leo_sensors

The leo_sensors node processes and publishes data from a variety of sensors mounted on the Leo Rover, such as cameras, LiDAR, depth cameras, and heat cameras.

### 3. leo_navigation

The leo_navigation node serves as the navigation system, incorporating SLAM algorithms and path planning functionalities.

### 4. communication_interface

The communication_interface node manages the communication interface between the robot and different nodes for the mechanical arm and payload.

### 5. payload

The Payload node is responsible for the carrying and releasing of objects.

### 6. manipulator_controller

This node handles the motion control of the robotic arm, accepting motion commands and driving joints or actuators.

### 7. manipulator_state_publisher

The manipulator_state_publisher node publishes information about the robotic arm's state, such as joint angles and positions.

### 8. end_effector_controller

The end_effector_controller node controls the end effector of the robotic arm, such as a gripper or other tools.

### RQT Graph

![RQT](img/rqt.jpeg "RQT_graph") 

# Install

This branch supports ROS Humble. See this [INSTALL.md](INSTALL.md) for more details of install.


# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
leo_robotic
├── img                          This folder include some design imgs about this project.
├── ros_leo_description          This folder include some basic code for robot model.
├── ros_leo_sensor               This folder include the code and example pics for analysising the sensor date.
├── ros_leo_demos                This folder include how to navigate the robotic in world.
├── test                         This is the test file for code.
├── INSTALL.md                   This file include the package needed to be downloaded for this project.
└── README.md                    This is the readme file.
```

# Contributing

Team 2: [Marcell](mailto:marcell.westsik@postgrad.manchester.ac.uk),[Maggie](ruyi.zhang-3@student.manchester.ac.uk),[Yining](mailto:yining.wang-17@postgrad.manchester.ac.uk),and[Drishti](mailto:drishtichauhanpvt@gmail.com).


# License

This library is licensed under [MIT LICENSE](https://opensource.org/license/mit/). 

# Contact
For maintenance inquiries, please contact [Marcell](mailto:marcell.westsik@postgrad.manchester.ac.uk),[Maggie](ruyi.zhang-3@student.manchester.ac.uk),[Yining](mailto:yining.wang-17@postgrad.manchester.ac.uk),and[Drishti](mailto:drishtichauhanpvt@gmail.com).
