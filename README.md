# Group 1, Library Robot

## Building Docker Image
The docker folder holds a variety of image definitions: humblebot, jazzybot, ubuntu2204, ubuntu2404, etc.  
Building each docker image works the same way.  Using humblebot as an example:

    cd enpm662p2/docker/humblebot
    ./build.sh
    ./launch.sh
    
Alternatively, all of these steps can be completed with
    
    cd enpm662p2
    ./init.sh

## Running a Container
The easiest way to run a container is to use

    cd enpm662p2
    ./init.sh

which will run a container and attach it, or build one if one does not already exist. If a container is already running, another terminal can be opened with the same container using

    cd enpm662p2
    ./attach.sh

Launching can also be done manually. Assuming the image was built without issue, you can repeatedly launch as many containers as you need.

    cd enpm662p2/docker/humblebot
    ./launch.sh 99

Replace '99' with any number you like.  This helps identify unique containers.  (e.g., humblebot1, humblebot2, etc.).

The launch script will try to mount your code directory to the container.  This lets you edit the code from your host/laptop and see the code changes in the container.
The launch script will try to find your code directory in one of a few expected locations.  You may have to edit launch.sh if you have a unique project folder.

## Running the Gazebo Demo
If there are changes to the Dockerfile, you will need to update the image and start a fresh container.  Please see the instructions in the previous sections.

After launching a new container, to open the library world in Gazebo (which automatically spawns the robot):

    cd /mnt/enpm662p2/project2
    ./terp2_empty

The script `terp2_build.sh` performs a clean build using `colcon` and sources the install files for you.  Alternatively, you can build and run it yourself:

    cd /mnt/enpm662p2/project2
    colcon build
    source install/setup.bash

    ros2 launch terp2 gazebo.launch.py

If you make changes to the project and want to see what happens, you can rerun `terp2_build.sh` or manually run the commands listed above.  

## Running the Teleop Demo
After starting up the Gazebo Demo (with the robot spawned), you can control the robot using the teleop controller.

After connecting to the existing container and spawning the library world, you can start the teleop controller:

    cd /mnt/enpm662p2/project2
    ./terp2_teleop.sh

## Running the Controller Demo

Before starting the controller, you will need to startup the Gazebo simulator.  Once the robot is loaded in Gazebo, without error, start another session to the same container, or start a new container.  For instructions, see above as with the teleop demo.

Once you have a session ready:

    cd /mnt/enpm662p2/project2
    ./terp2_controller.sh

That bash script is basically running:

    ros2 run terp2_controller terp2_controller

The controller will log telemetry data to the screen.  Use this to monitor what's happening.  This is useful even when using the teleop demo.

To run the library shelving demo within the library world in Gazebo, run

    ./terp2_library_nav.sh

This script is a sequence of actions defined by setting parameters with `terp2_set_goal.sh`, `terp2_set_arm.sh` and `terp2_set_gripper.sh`

To set parameters manually, start a new session (or new container) and run as described previously. Then, run the desired script for setting parameters. Below are examples:

    ./terp2_set_goal.sh 10 10 #move the robot to x=10, y=10
    ./terp2_set_arm.sh 3.0 0.5 1.25 -0.8 0.0 #set joints 1 to 5 to the positions of 3.0, 0.5, 1.25, -0.8, and 0.0
    ./terp2_set_gripper.sh 1.5 0.0 0.0 0.0 #set the "gripper joints" to positions 1.5, 0.0, 0.0, and 0.0
    
