# tblaze_controller_test_nodes

Contains test nodes to verify that the ROS2 controllers for tblaze are working.

## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
device on which you want to run this code. That device might be an Ubuntu machine or a physical
robot using Raspberry Pi OS.

* [tblaze_description](https://github.com/pvandervelde/tblaze_description) - Contains the geometric
  description of the tblaze robot for ROS to work with.

## Contents

This repository contains two different ROS nodes which publish [ROS control](https://control.ros.org/master/index.html)
commands. The `publisher_joint_trajectory_controller` node publishes a set of positions over a given
timespan. The `publisher_velocity_controller` node publishes a set of velocities over a given timespan.

## Usage

In general the `tblaze_description` package will not directly be launched. It is designed to be
referenced by other SCUTTLE ROS packages.
