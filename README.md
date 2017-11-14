# EL2425 - Drone Search & Rescue

This repo contains the code used in the Drone project for the course EL2425 - Automatic Control, Project Course, Smaller Course given at KTH during fall 2017.

## Installation

Clone this repository in your source folder in your catkin workspace (_~/catkin\_ws/src_ if you followed the standard ROS tutorials.) Then, build the package by running _catkin\_make_ in your catkin workspace folder. 

### Dependencies

This ROS package was build and tested on [ROS Kinetic Kame](http://wiki.ros.org/kinetic) running on [Ubuntu 16.04](http://releases.ubuntu.com/16.04/).

The code depends on a couple of other ROS packages:
 - [The Crazyflie ROS package](https://github.com/whoenig/crazyflie_ros)
 - [The KTH-SML Qualisys ROS package](https://github.com/KTH-SML/qualisys)

The code also depends on a couple of Python packages:
 - [cflib](https://github.com/bitcraze/crazyflie-lib-python)
 - pygame

 Note that cflib requires access to the USB radio to work. Follow the udev permission tutorial in [this repo](https://github.com/bitcraze/crazyflie-lib-python) to make it work on Linux.
