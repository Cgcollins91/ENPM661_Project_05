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
source ${BASE_PATH}/install/setup.bash
ros2 launch terp1 gazebo.launch.py


