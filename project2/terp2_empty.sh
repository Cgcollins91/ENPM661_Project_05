#!/bin/bash

# Project: ENPM661-Project5-Group2
# License: MIT
# The code in this file represents the collective work of Group 2.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

BASE_PATH=/mnt/ENPM661_Project_05/project2
cd ${BASE_PATH} || exit
source ${BASE_PATH}/install/setup.bash
# ros2 service call /slam_toolbox/reset std_srvs/srv/Empty {}
ros2 launch terp2 gazebo.launch.py


