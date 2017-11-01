# beginner_tutorials
ROS Beginner Tutorials
## Overview
This project contains a ROS package named "beginner_tutorials".<br />
This project covers a tutorial on how to write a simple publisher and subscriber node in C++.<br />
The package and code is taken directly from [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials).<br />
## Dependencies
### ROS
ROS should be installed on the system. This package is tested on Ubuntu 16.04 LTS with [ROS Kinetic Distribution](http://wiki.ros.org/kinetic).<br />
Installation Instructions can be found [here](http://wiki.ros.org/kinetic/Installation).
### catkin
catkin is a Low-level build system macros and infrastructure for ROS.<br />
catkin is included by default when ROS is installed. It can also be installed with apt-get
```
sudo apt-get install ros-kinetic-catkin
```
## Build Instructions
### Creating a catkin workspace
Create a catkin workspace using following instructions:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Running catkin_make command the first time in your workspace will create a CMakeLists.txt link in your 'src' folder. Before continuing source your new setup.*sh file:
```
$ source devel/setup.bash
```
### Building the Package inside catkin workspace
Clone the package in src folder of catkin workspace using following commands:
```
$ cd ~/catkin_ws/src/
$ https://github.com/VBot2410/beginner_tutorials.git
```
Then build the package using following commands:
```
$ cd ~/catkin_ws/
$ catkin_make
```
## Running The Demo
Make sure that a roscore is up and running:
```
$ roscore
```
make sure you have sourced your workspace's setup.sh file after calling catkin_make but before trying to use your applications:
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```
### Running the Publisher
Open a new terminal and run following commands:
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker
```
### Running the Subscriber
Open a new terminal and run following commands:
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
```
