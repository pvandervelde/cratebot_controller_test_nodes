# zinger_controller_test_nodes

This package contains test nodes to verify that the ROS2 controllers for zinger are working. There
are test nodes for:

- The steering controllers for all four wheels of the zinger robot: `steering_controller.py`. This
  verifies that the steering angle for the four drive modules can be changed.
- The drive controller for all four wheels of the zinger robot: `drive_controller.py`. This verifies
  that the wheel rotation speed for the four drive modules can be changed.

## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
device on which you want to run this code. That device might be an Ubuntu machine or a physical
robot using Raspberry Pi OS.

- [zinger_description](https://github.com/pvandervelde/zinger_description) - Contains the geometric
  description of the zinger robot for ROS to work with.

## Contents

This repository contains different folders for the different parts of the robot description.

- The `config` files that provide the configurations for the ROS control actuators and the test publishers
  - [config/steering_controller.yaml](config/steering_controller.yaml) defines the
    settings for the test publisher that changes the steering angle of all the wheel modules.
  - [config/drive_controller.yaml](config/drive_controller.yaml) defines the settings for
    the test publisher that changes the wheel velocity for all the wheel modules.
- The `launch` directory contains the launch files
  - [launch/steering_controller.launch.py](launch/steering_controller.launch.py) is the launch
    file used to launch the steering controller.
  - [launch/drive_controller.launch.py](launch/drive_controller.launch.py) is the launch file
    used to launch the drive controller.
- The source code for the controllers can be found in the `zinger_controller_test_nodes` directory
  - The `steering_controller.py` defines a ROS2 controller that controls the steering angle of
    the drive modules using the [JointGroupPositionController](https://github.com/ros-controls/ros2_controllers/blob/master/position_controllers/doc/userdoc.rst).
  - The `drive_controller.py` defines a ROS2 controller that controls the wheel rotation velocity
    of the drive modules using the [JointGroupVelocityController](https://github.com/ros-controls/ros2_controllers/blob/master/velocity_controllers/doc/userdoc.rst).

## Usage

To start the steering controller use the following command line

    ros2 launch zinger_controller_test_nodes steering_controller.launch.py

To start the drive controller use the following command line

    ros2 launch zinger_controller_test_nodes drive_controller.launch.py

When simulating the zinger robot in Gazebo add the `use_sim_time:=true` parameter to the command lines.
