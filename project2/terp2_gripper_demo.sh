#!/bin/bash

# Project: ENPM661-Project2-Group1
# License: MIT
# The code in this file represents the collective work of Group 2.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.


./terp2_set_gripper.sh 0.0 0.0 0.0 0.0 
sleep 2

./terp2_set_gripper.sh 1.5 0.0 0.0 0.0 
sleep 2

./terp2_set_gripper.sh 1.5 0.0 1.0 1.0 
sleep 2

./terp2_set_gripper.sh 1.5 0.0 0.0 0.0 
sleep 2

./terp2_set_gripper.sh 0.0 0.0 0.0 0.0 
sleep 2

./terp2_set_gripper.sh 0.0 0.0 1.0 1.0 
sleep 2

./terp2_set_gripper.sh 0.0 0.0 0.0 0.0 

