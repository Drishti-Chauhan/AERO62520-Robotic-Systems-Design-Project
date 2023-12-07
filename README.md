# AERO62520: LEO ROBOTICS SYSTEMS DESIGN PROJECT

**Maintainer:** team 2



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

## Install

This branch supports ROS Rolling. See above for other ROS versions.



### Binaries

Rolling binaries are available for Fortress.
They are hosted at https://packages.ros.org.

1. Add https://packages.ros.org

        sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        sudo apt-get update

1. Install `ros_gz`

        sudo apt install ros-rolling-ros-gz

### From source

#### ROS

Be sure you've installed
[ROS Rolling](https://index.ros.org/doc/ros2/Installation/)
(at least ROS-Base). More ROS dependencies will be installed below.

#### Gazebo

Install either [Edifice, Fortress, or Garden](https://gazebosim.org/docs).

Set the `GZ_VERSION` environment variable to the Gazebo version you'd
like to compile against. For example:

    export GZ_VERSION=edifice

> You only need to set this variable when compiling, not when running.

#### Compile ros_gz

The following steps are for Linux and OSX.

1. Create a colcon workspace:

    ```
    # Setup the workspace
    mkdir -p ~/ws/src
    cd ~/ws/src

    # Download needed software
    git clone https://github.com/Drishti-Chauhan/AERO62520-Robotic-Systems-Design-Project.git
    ```

1. Install dependencies (this may also install Gazebo):

    ```
    cd ~/ws
    rosdep install -r --from-paths src -i -y --rosdistro humble
    ```

    > If `rosdep` fails to install Gazebo libraries and you have not installed them before, please follow [Gazebo installation instructions](https://gazebosim.org/docs/latest/install).

1. Build the workspace:

    ```
    # Source ROS distro's setup.bash
    source /opt/ros/<distro>/setup.bash

    # Build and install into workspace
    cd ~/ws
    colcon build
    ```


# Folder Structure

Refer to the following table for information about important directories and files in this repository.

```
leo_robotic
├── img
├── ros_leo        
├── ros_leo_bridge
├── ros_leo_pointcloud
├── ros_leo_sim
├── ros_leo_sim_demos
├── test                         Test for code.
└── README.md                    This readme.
```

# Contributing

Please see
[CONTRIBUTING.md](https://github.com/gazebosim/gz-sim/blob/main/CONTRIBUTING.md).



# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/gazebosim/gz-sim/blob/main/LICENSE) file.

# Contact
For inquiries and support, contact [maintainer's email]