# Turtlebot Walker
<a href='https://github.com/Arjung27/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
---

## Project Overview
Implementation of a simle walker algorithm using turtlebot like a Roomba robot vaccum cleaner.

## Dependencies
Following dependencies needs to be installed before running this project:

```
1. ROS Kinetic, [click here](http://wiki.ros.org/kinetic/Installation) to install it.
2. Ubuntu 16.04
3. Turtlebot packages:
      sudo apt-get install ros-kinetic-turtlebot-gazebo
      ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
## How to build your project
```
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash (For every new terminal this command needs to be run before running any 
      ROS command. You could add it to your bashrc so that every time a terminal opens it would run by itself)
cd src/
git clone https://github.com/Arjung27/turtlebot_walker.git
cd ..
catkin_make
```
## Running the demo
### Using roslaunch
Type the following in a new terminal:
```
roslaunch turtlebot_walker walkerAlgorithm.launch
```
### Using rosrun
If you want to run the each node manually and in separate terminal then you can do it 
by following the steps below:
```
roscore
roslaunch turtlebot_gazebo turtlebot_world.launch (in the new terminal)
rosrun turtlebot_walker turtlebot_walker (in the new terminal)
```
## Recording the bag files using rosbag
Record the bag files using the launch file by using the following commands:
```
 roslaunch turtlebot_walker walkerAlgorithm.launch record:=true record_time:=30
 ```
By default the record value is set as false (i.e. the bag files are not recorded). Here we set it to true. Also, the record_time argument can be set to record the bag files for fixed number of seconds. The bag file is stored in the results folder of the parent directory.

## Playing the bag file
First cd to the results folder using:
 ```
cd ~/catkin_ws/src/turtlebot_walker/results
```
Open roscore in separate terminal and play the bag file using:
```
rosbag play turtlebot_walker.bag
```
You can verify the published topic by echoing the following topic:
```
rostopic echo /cmd_vel_mux/input/navi
```