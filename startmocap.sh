#!/bin/bash

export LD_LIBRARY_PATH=./
source /opt/ros/melodic/setup.bash
source ~/.bashrc
source ./devel/setup.bash

rosrun nokov_ros nokov_ros_node

