## Overview

#### ROS Gazebo Tutorial:

This project provides an example of a simple walker algorithm much 
like a Roomba robot vacuum cleaner by creating a ROS package for a 
TurtleBot simulated in a Gazebo environment.


## Dependencies
```
1. ROS melodic
2. Catkin
3. Ubuntu 10.04 LTS
4. TurtleBot Gazebo
```
Install ROS melodic and setup catkin workspace by following this tutrial:
1. [Link to ROS tutorial!](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/kartikv97/TurtleBot_Gazebo_Walker.git
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make
```
## Run launch file
Open a new terminal and run the commands given below:

1. Open terminal 1 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  roscore
```  
2. Open terminal 2 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  roslaunch turtlebot_gazebo_walker turtlebot_gazebo_walker.launch
```

To stop the program press Ctrl + c in each terminal 


## Record bag file
Open a new terminal and run the commands given below:

1. Open terminal 1 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  roscore
```  
2. Open terminal 2 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  roslaunch turtlebot_gazebo_walker turtlebot_gazebo_walker.launch startRecording:=true
```
## Play bag file
Open a new terminal and run the commands given below:

1. Open terminal 1 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  roscore
```  
2. Open terminal 2 and run (to view wheel velocities being published):
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rostopic echo /cmd_vel
```
3. Open terminal 3 and run:
```
  cd ~/catkin_ws/src/turtlebot_gazebo_walker/results
  source ./devel/setup.bash
  rosbag play turtlebot_gazebo_walker.bag 
```


