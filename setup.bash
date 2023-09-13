#!/usr/bin/env bash

# Modify this for your environment

# Add results of ArduSub build
export PATH=$HOME/QGC/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$HOME/QGC/ardupilot/Tools/autotest:$PATH

# Add results of colcon build
source $HOME/workspaces/tur_ws_cont/install/setup.bash

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros2_ws/build/ardupilot_gazebo:$Z_SIM_SYSTEM_PLUGIN_PATH

# Optional: add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/ardupilot_gazebo/models:$HOME/ros2_ws/src//ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add bluerov2_ignition models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/workspaces/tur_ws_cont/src/bluerov2_ignition/models:$HOME/workspaces/tur_ws_cont/src/bluerov2_ignition/worlds:$GZ_SIM_RESOURCE_PATH

# Add orca4 models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/workspaces/tur_ws_cont/src/multiple_orca/models:$HOME/workspaces/tur_ws_cont/src/multiple_orca/worlds:$GZ_SIM_RESOURCE_PATH

# Build ros_gz on the humble branch for Gazebo Garden
export GZ_VERSION=garden
