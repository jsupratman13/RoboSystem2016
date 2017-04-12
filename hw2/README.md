# Homework 2
* create a demonstration with raspberry pi using ROS
* show video on youtube
* upload source code on github

## Description
* controlling roomba using joystick controller via Raspberry pi and ROS <br />
https://youtu.be/xJhtPAgWQ4Q

```
roscore &
rosrun joy joy_node &
rosrun roomba_500_series roomba500_light_node &
rosrun hw2 JoyControlRoomba
```

## Prerequisite
* Ubuntu 16.04 on raspberry pi 3
* ROS installed in raspberry pi 3
* install joystick controller driver ROS package
```
sudo apt update
sudo apt install ros-kinetic-joy
```
* install roomba driver ROS package
```
git clone https://github.com/NetBUG/cereal_port
git clone https://github.com/NetBUG/roomba_500_series
rosdep install cereal_port
rosdep install roomba_500_series
catkin_make -j1
catkin_make install
 ```
