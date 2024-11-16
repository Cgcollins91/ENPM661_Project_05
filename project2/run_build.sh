#!/bin/bash

# Project: ENPM662-Project1-Group1
# License: MIT
# The code in this file represents the collective work of Group 1.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

BASE_PATH=/mnt/enpm662p2/project2
cd ${BASE_PATH} || exit
source /opt/ros/humble/setup.bash
rm -rf ${BASE_PATH}/build
rm -rf ${BASE_PATH}/install
rm -rf ${BASE_PATH}/log

#arena models
mkdir -p ~/.gazebo/models/arena
rm -rf ~/.gazebo/models/arena
cp -R ${BASE_PATH}/src/terp1/models/arena ~/.gazebo/models/

colcon build
source ${BASE_PATH}/install/setup.bash


