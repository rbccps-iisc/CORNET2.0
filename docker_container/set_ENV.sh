#!/bin/bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SKIP_DEFAULT_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/newprofile.xml
#export RMW_FASTRTPS_USE_QOS_FROM_XML=1
source /opt/ros/foxy/setup.bash

#ros2 run demo_nodes_cpp talker
