#!/bin/bash
./home/nvidia/jetson_clocks.sh
set ROS_IP='10.20.53.42'
source /home/nvidia/catkin_ws/devel/setup.bash
roslaunch frc_robot main_robot.launch

