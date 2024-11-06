# IRED Move to Goal
## Install Package
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/aims-lab-kmitl/ired_movetogoal.git
$ cd ~/catkin_ws
$ catkin_make
```
## How to use package
- IRED
```sh
$ roslaunch ired_bringup bringup.launch
$ roslaunch ired_navigation navigation.launch
$ roslaunch ired_movetogoal movetogoal.launch
```
- PC
```sh
$ roslaunch ired_rviz navigation.launch
``` 