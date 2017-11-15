# beginner_tutorials
ROS Beginner Tutorials<br />
<br />
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Overview
This project contains a ROS package named "beginner_tutorials".<br />
This project covers a tutorial on how to write a simple publisher and subscriber node in C++.<br />
This section covers rosbag, tf, gtest/rostest.<br />
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
$ git clone --branch Week11_HW https://github.com/VBot2410/beginner_tutorials.git
```
Then build the package using following commands:
```
$ cd ~/catkin_ws/
$ catkin_make
```
## Running The Demo
Open a terminal and run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials Tutorial.launch record:=false
```
You may also specify custom message and Frequency by using **arg:=value**. For example, to launch with Message "Hi!" & Frequency 100Hz,
```
roslaunch beginner_tutorials Tutorial.launch Message:="Hi!" Frequency:=100 record:=false
```
For the custom message to take effect, open a new terminal & follow these instructions:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials modify
```
### Using rosservice to call the service
Open a new terminal after following the above instructions for running the Demo
```
$ cd ~/catkin_ws
$ source devel/setup.bash
rosservice call /String_Modify "Modified String"
```
The message broadcasted by talker will be changed to "Modified String". You may use any custom string while using above command.
### Inspecting TF Frames
Talker node braodcasts tf frame /talk with parent /world.<br />
To view the transforms, run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials Tutorial.launch record:=false
```
To view the output of /talk frame with reference frame /world, launch a new terminal and run following command:
```
rosrun tf tf_echo /world /talk
```
The output shown will be similar to the following:
```
At time 1510702945.420
- Translation: [-0.569, -1.917, 3.000]
- Rotation: in Quaternion [-0.626, 0.091, 0.654, 0.415]
            in RPY (radian) [-1.106, 1.106, 1.283]
            in RPY (degree) [-63.380, 63.380, 73.484]
At time 1510702945.420
- Translation: [-0.569, -1.917, 3.000]
- Rotation: in Quaternion [-0.626, 0.091, 0.654, 0.415]
            in RPY (radian) [-1.106, 1.106, 1.283]
            in RPY (degree) [-63.380, 63.380, 73.484]
At time 1510702946.420
- Translation: [0.575, 1.915, 3.000]
- Rotation: in Quaternion [0.092, 0.625, -0.416, 0.654]
            in RPY (radian) [-1.106, 1.106, -1.863]
            in RPY (degree) [-63.380, 63.380, -106.716]
```
To generate output of view_frames tool, run following command while talker node is broadcasting
```
rosrun tf view_frames
```
## Testing
Tests for talker node are written using rostest and gtest. To build the tests, run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make run_tests_beginner_tutorials
```
To run the tests after building them in previous step, use following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rostest beginner_tutorials Test_Talker.launch
```
This will give an output similar to following:
```
... logging to /home/viki/.ros/log/rostest-ubuntu-11189.log
[ROSUNIT] Outputting test results to /home/viki/.ros/test_results/beginner_tutorials/rostest-test_Test_Talker.xml
testTest_Talker ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-Test_Talker/Service_Existance_Test][passed]
[beginner_tutorials.rosunit-Test_Talker/String_Modification_Test][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/viki/.ros/log/rostest-ubuntu-11189.log
```
## Recording bag files with the launch file
So far we've been keeping the launch file argument *record:=false*. To record the bag file, simply make the record argument *true*. Follow these commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch beginner_tutorials Tutorial.launch record:=true
```
This will record all topics and a bag file with name *rosbag_output.bag* will be generated in the results directory.
### Enable/Disable rosbag recording
The argument *record* in Tutorial.launch launch file is a falg that enables or disables the recording.<br />
Use *record:=true* to enable the recording and *record:=false* to disable it.

### Inspecting the bag file
For inspecting the recorded rosbag file, run following commands:
```
$ cd ~/catkin_ws/src/beginner_tutorials/results
$ rosbag info rosbag_output.bag
```
This will produce an output similar to following:
```
path:        rosbag_output.bag
version:     2.0
duration:    1:27s (87s)
start:       Nov 14 2017 16:53:44.49 (1510696424.49)
end:         Nov 14 2017 16:55:11.55 (1510696511.55)
size:        501.5 KB
messages:    2592
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      431 msgs    : std_msgs/String   
             /rosout       870 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   860 msgs    : rosgraph_msgs/Log 
             /tf           431 msgs    : tf2_msgs/TFMessage
```

### Playing back the bag file with Listener Node
After generating the bag file, follow these commands:<br />
Open a new terminal and run following commands:
```
$ roscore
```
Open another terminal and run following commands:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```
Open a new terminal and run following commands:
```
$ cd ~/catkin_ws/src/beginner_tutorials/results
$ rosbag play rosbag_output.bag
```
Listener node will print the messages that were sent by talker node while recording the bag file *rosbag_output.bag*
