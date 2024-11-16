#!/bin/bash

# Project: ENPM662-Project1-Group1
# License: MIT
# The code in this file represents the collective work of Group 1.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

#
# example:  ./set_goal.sh 5 5
#    will cause controller to navigate terp1 to (5,5)

source install/setup.bash
ros2 param set /terp1_controller goal "[${1}.0,${2}.0]"
