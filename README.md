# enpm662p1

## Building Docker Image
The docker folder holds a variety of image definitions: humblebot, jazzybot, ubuntu2204, ubuntu2404, etc.  
Building each docker image works the same way.  Using humblebot as an example:

    cd enpm662p1/docker/humblebot
    ./build.sh

Once you build the images, you can create containers with it.

## Running a Container
The docker folder (see above) has a script to help you launch a container.  
Assuming the image was built without issue, you can repeatedly launch as many containers as you need.

    cd enpm662p1/docker/humblebot
    ./launch.sh 99

Replace '99' with any number you like.  This helps identify unique containers.  (e.g., humblebot1, humblebot2, etc.).

The launch script will try to mount your code directory to the container.  This lets you edit the code from your host/laptop and see the code changes in the container.
The launch script will try to find your code directory in one of a few expected locations.  You may have to edit launch.sh if you have a unique project folder.

## Running the Gazebo Demo
If there are changes to the Dockerfile, you will need to update the image and start a fresh container.  Please see the instructions in the previous sections.

After launching a new container, to run the Gazebo demo (which automatically spawns the robot):

    cd /mnt/enpm662p1/project1
    ./run_empty.sh

The script `run_gazebo.sh` performs a clean build using `colcon` and sources the install files for you.  Alternatively, you can build and run it yourself:

    cd /mnt/enpm662p1/project1
    colcon build
    source install/setup.bash

    ros2 launch terp1 gazebo.launch.py

If you make changes to the project and want to see what happens, you can rerun `run_gazebo.sh` or manually run the commands listed above.  

## Running the Teleop Demo
After starting up the Gazebo Demo (with the robot spawned), you can control the robot using the teleop controller.  

First, attach to the existing container or start a new container.  Either will work.

### Running the Teleop Demo in an Existing Container
Attach to the container running the Gazebo simulation:

    # connecting to an existing container named `humblebot1`
    docker exec -it humblebot1 bash

After connecting to the existing container, you can start the teleop controller:

    cd /mnt/enpm662p1/project1
    ./run_teleop.sh

Like the Gazebo Demo, you can also run it manually:
    
    cd /mnt/enpm662p1/project1
    source install/setup.bash

    ros2 run terp1 teleop.py

### Running the Teleop Demo in a New Container

Create a new container:

    # creating a new container named `humblebot99`
    cd project_dir/docker/humblebot
    ./launch.sh 99 

Since we are using a new container, we need to build the project first.  Only do this step when running the teleop demo in another container:

    cd /mnt/enpm662p1/project1
    colcon build
    source install/setup.bash

    ros2 run terp1 teleop.py

## Running the Controller Demo

Before starting the controller, you will need to startup the Gazebo simulator.  Once the robot is loaded in Gazebo, without error, start another session to the same container, or start a new container.  For instructions, see above as with the teleop demo.

Once you have a session ready:

    cd /mnt/enpm662p1/project1
    ./run_controller.sh

That bash script is basically running:

    ros2 run terp1_controller terp1_controller

The controller will log telemetry data to the screen.  Use this to monitor what's happening.  This is useful even when using the teleop demo.

The controller is waiting for a parameter named `goal` to set the x and y coordinate of the target destination for the robot. There is an example of how to set the parameter in `set_goal.sh`.  

To set the parameter, start a new session (or new container) as described previously and run the following:

    ros2 param set /terp1_controller goal "[10.0, 10.0]"  #to move robot to x=10.0, y=10.0

## Running the Arena, LIDAR, and RViz Demo
Before running this demo, ensure you have an updated docker image and are not running other simulations at the same time.  This demo will automatically start Gazebo and RViz.  

    cd /mnt/enpm662p1/project1
    ./run_arena.sh

That bash script is running:

    ros2 launch terp1 competition.launch.py

The automated controller is currently not compatible with the arena.  However, you can start a teleop controller (see above) and direct the robot around the track.  RViz should show realtime LIDAR and joint state data.


## Additional Details on Project 1
The project includes mix of changes that got it from a collection of URDF files into a working project.  Please see project1/README.md for more details.
