# Group 2, Library Robot

# Team Members

Chris Collins 
  - UID: 110697305
  - Directory ID: ccollin5@umd.edu

Kyle Demmerle 
  - UID: 121383341
  - Directory ID: kdemmerl@umd.edu

Dan Zinobile 
  - UID: 121354464
  -  Directory ID: zinobile@umd.edu

# Clone the repository in home folder

git clone https://github.com/Cgcollins91/ENPM661_Project_05.git

# Dependencies
## Python

import pygame

import pygame.gfxdraw

import time

import math

import heapq

import numpy as np

import csv

import os

from pathlib import Path

from __future__ import annotations

from random import random, uniform

from math   import hypot

## Building Docker Image
The docker folder holds a image definition for ROS Humble
Build the docker image by:

    cd ENPM661_Project_05/docker/humblebot
    ./build.sh
    ./launch.sh
    
Alternatively, all of these steps can be completed with
    
    cd ENPM661_Project_05
    ./init.sh

## Running a Container
The easiest way to run a container is to use

    cd ENPM661_Project_05
    ./init.sh

which will run a container and attach it, or build one if one does not already exist. If a container is already running, another terminal can be opened with the same container using

    cd ENPM661_Project_05
    ./attach.sh

Launching can also be done manually. Assuming the image was built without issue, you can repeatedly launch as many containers as you need.

    cd ENPM661_Project_05/docker/humblebot
    ./launch.sh 99

Replace '99' with any number you like.  This helps identify unique containers.  (e.g., humblebot1, humblebot2, etc.).

The launch script will try to mount your code directory to the container.  This lets you edit the code from your host/laptop and see the code changes in the container. The launch script will try to find your code directory in one of a few expected locations.  You may have to edit launch.sh if you have a unique project folder.

If this is unsucessful, it is recommended that you edit <your/path/to/the/project> below and then run the following commands in a terminal to launch the docker container after being built:

    sudo docker run --rm --net=host -e DISPLAY=$DISPLAY \
               -v /tmp/.X11-unix:/tmp/.X11-unix \
               -v <your/path/to/the/project>:/mnt/ENPM661_Project_05 \
               -v /dev/shm:/dev/shm \
               -it humblebot
               
    cd mnt/ENPM661_Project_05/project2

## Running the Gazebo Demo
If there are changes to the Dockerfile, you will need to update the image and start a fresh container.  Please see the instructions in the previous sections.

After launching a new container, to open the library world in Gazebo (which automatically spawns the robot):

    cd /mnt/ENPM661_Project_05/project2
    ./terp2_empty

The script `terp2_build.sh` performs a clean build using `colcon` and sources the install files for you.  Alternatively, you can build and run it yourself:

    cd /mnt/ENPM661_Project_05/project2
    colcon build
    source install/setup.bash

If you make changes to the project and want to see what happens, you can rerun `terp2_build.sh` or manually run the commands listed above.  

## Running the Teleop Demo
After starting up the Gazebo Demo (with the robot spawned), you can control the robot using the teleop controller.

After connecting to the existing container and spawning the library world, you can start the teleop controller:

    cd /mnt/ENPM661_Project_05/project2
    ./terp2_teleop.sh

# Path Planning
Path planning is executed outside of the gazebo simulation environment and stores a csv of waypoints and goal points in terp2_controller_py/path. The gazebo simulation reads in the csv file generated and follows the path. To run A* path planning the following commands can be run in terminal, alternatively the scripts can be run in cell mode. Note: You will need to rebuild with ./terp2_build.sh each time you run the path planning to refresh the path/goals use in the path_follower. Use Test Case of 2 Books, and input 100 200.

    cd project2/src/terp2_controller_py/terp2_controller_py
    python3 multi_point_planner.py

To run RRT* Path Planning

    cd project2/src/terp2_controller_py/terp2_controller_py
    python3 multi_point_planner_rrt.py


## Running the Path Planning, Mapping, and Book Shelving Demo
Before starting the mapping and path planning demo you  need to startup the Gazebo simulator.  Once the robot is loaded in Gazebo, without error, start another session to the same container, or start a new container.  For instructions, see above as with the teleop demo. The robot will follow the path in path/path.csv and shelf a book at each goal point in path/goals.csv

Once you have a session ready in one terminal, Start Gazebo Simulation loading robot into library world:

    cd /mnt/ENPM661_Project_05/project2
    ./terp2_empty.sh 

In a seperate terminal within the same container, launch mapping node and view in RVIZ:

    cd /mnt/ENPM661_Project_05/project2
    ./terp2_slam.sh

In a seperate terminal within the same container, launch path follower:

    cd /mnt/ENPM661_Project_05/project2
    ros2 run terp2_controller_py path_follower

The controller will log telemetry data to the screen.  Use this to monitor what's happening.  This is useful even when using the teleop demo.

## Manually Controlling Robot
To set parameters manually, start a new session (or new container) and run as described previously. Then, run the desired script for setting parameters. Below are examples:

    ./terp2_set_goal.sh 10 10 #move the robot to x=10, y=10
    ./terp2_set_arm.sh 3.0 0.5 1.25 -0.8 0.0 #set joints 1 to 5 to the positions of 3.0, 0.5, 1.25, -0.8, and 0.0
    ./terp2_set_gripper.sh 1.5 0.0 0.0 0.0 #set the "gripper joints" to positions 1.5, 0.0, 0.0, and 0.0
    

## Build Map of New World 
The mapping node can be used to generate a map of any world the robot is launched into, the user could edit the world used in terp2/launch/gazebo.py to any world of their choosing, then the robot could be navigated via teleop or by sending goals(see above) in the terminal to explore the map. Once the map has been sufficiently explore it can be saved with:

    ros2 run terp2_slam save_grid
   
This map can then be used for path planning

