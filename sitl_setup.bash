#!/bin/bash

THIS_SCRIPT_PATH=${BASH_SOURCE[0]}
PX4_PATH="/home/mantovanim/PX4-Autopilot"

if [ "$#" != 1 ]; then
    echo -e "You can suppress this printout with \n >> source setup_ifo_gazebo.bash suppress \n"
    SUPPRESS_OUTPUT=false 
else
    SUPPRESS_OUTPUT=true
fi

export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${PX4_PATH}/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${PX4_PATH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PX4_PATH}/build/px4_sitl_default/build_gazebo-classic


export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/simulation/gazebo-classic

if [ "$SUPPRESS_OUTPUT" = false ]; then
    echo -e "GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
    echo -e "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
    echo -e "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    echo -e "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
fi