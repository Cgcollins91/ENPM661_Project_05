#!/bin/bash

# Project: ENPM662-Project2-Group1
# License: MIT
# The code in this file represents the collective work of Group 1.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.


# go to table
./terp2_set_goal.sh 9 0
./terp2_set_arm.sh 3.0 0.0 0.0 0.0 0.0
./terp2_set_gripper.sh 1.5 0.0 0.0 0.0
sleep 12

# grab 1x book
./terp2_set_arm.sh 3.0 0.5 1.25 -0.8 0.0
sleep 2
./terp2_set_gripper.sh 1.5 0.0 0.03 0.03
sleep 1

# store 1x book
./terp2_set_arm.sh 3.0 0.0 -0.8 0.0 0.0
sleep 1
./terp2_set_arm.sh 3.0 -1.1 -1.1 0.0 -0.1
sleep 1
./terp2_set_arm.sh 3.0 -1.25 -0.9 -0.3 -0.1
sleep 1
./terp2_set_gripper.sh 1.5 0.0 0.0 0.0
sleep 1
./terp2_set_arm.sh 3.0 -1.1 -1.1 0.0 -0.1
sleep 1
./terp2_set_arm.sh 3.0 0.0 -0.8 0.0 0.0
sleep 1

# go to shelf
./terp2_set_goal.sh 3 0
./terp2_set_arm.sh 3.0 0.0 0.0 0.0 0.0
sleep 10
./terp2_set_goal.sh 0 6
sleep 15

# remove 1x book from storage
./terp2_set_arm.sh 3.0 0.0 -0.8 0.0 0.0
sleep 1
./terp2_set_arm.sh 3.0 -1.1 -1.1 0.0 -0.1
sleep 1
./terp2_set_arm.sh 3.0 -1.25 -0.9 -0.3 -0.1
sleep 1
./terp2_set_gripper.sh 1.5 0.0 0.03 0.03
sleep 1
./terp2_set_arm.sh 3.0 -1.1 -1.1 0.0 -0.1
sleep 1
./terp2_set_arm.sh 3.0 0.0 -0.8 0.0 0.0
sleep 1
./terp2_set_arm.sh 3.0 0.0 0.0 0.0 0.0
sleep 1

# place 1x book on shelf
./terp2_set_arm.sh 2.7 0.0 0.0 0.0 0.0
sleep 2
./terp2_set_arm.sh 2.7 0.0 1.0 2.1 3.0
sleep 2
./terp2_set_arm.sh 2.7 0.2 1.2 2.2 3.0
sleep 1
./terp2_set_gripper.sh 1.5 0.0 0.0 0.0
sleep 1
./terp2_set_arm.sh 2.7 0.2 1.2 2.2 3.0
sleep 2
./terp2_set_arm.sh 2.7 0.0 0.0 0.0 0.0
sleep 2

# # go home
./terp2_set_goal.sh 0 0
sleep 10



