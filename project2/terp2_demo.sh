#!/bin/bash

# Project: ENPM662-Project2-Group1
# License: MIT
# The code in this file represents the collective work of Group 1.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.


./terp2_set_goal.sh 5 0
./terp2_arm_demo.sh


./terp2_set_goal.sh 5 5
./terp2_arm_demo.sh

./terp2_set_goal.sh 0 0
./terp2_arm_demo.sh

