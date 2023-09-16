#!/bin/bash

gnome-terminal \
 --tab -e "ros2 run multiple_orca mission_runner.py --ros-args -r __ns:=/rov1 -r /tf:=tf -r /tf_static:=tf_static -p use_sim_time:=false" \
 --tab -e "ros2 run multiple_orca mission_runner.py --ros-args -r __ns:=/rov2 -r /tf:=tf -r /tf_static:=tf_static -p use_sim_time:=false"  \
#  --tab -e "ros2 run multiple_orca mission_runner.py --ros-args -r __ns:=/tur3 -r /tf:=tf -r /tf_static:=tf_static -p use_sim_time:=false"  
