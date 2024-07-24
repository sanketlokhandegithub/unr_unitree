#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/sanket/workspace_catkin/unitree_ws/devel/setup.bash
cd /home/sanket/workspace_catkin/unitree_ws
roslaunch unitree_legged_real real.launch rname:=a1 ctrl_level:=highlevel firmwork:=3_2
