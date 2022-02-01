#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/k354jn1/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=192.168.1.146 
export ROS_MASTER_URI=http://192.168.1.146:11311 
export PYTHONPATH=$PYTHONPATH:/usr/local/lib::/usr/local/lib/python3.6

# this allows to display GUI on the host
export DISPLAY=:0 

exec "$@"
