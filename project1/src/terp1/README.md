## Starting Point

NOTE: This file captures the significant steps that were taken to get the gazebo simulator to load the robot.  However, this file does not have a complete record of all the steps that we took.  For instructions on how to run the project, please see the README.md in the repository root folder (../README.md). 

For this project, I am using "project1" as the ROS2 workspace and "terp1" as the robot name.  
```
#workspace = project1
#robot/package = terp1
#working directory = /mnt/enpm662p1 (which is how it's mounted in the container)
```

## Create Package
First step is to create a new ROS2 package and source it.  
```
#create terp1 package
cd /mnt/enpm662p1
mkdir -p project1/src
cd /mnt/enpm662p1/project1/src
ros2 pkg create --build-type ament_cmake terp1

#build project1
cd /mnt/enpm662p1/project1
colcon build 
source install/setup.bash  
```

## Add and Fix the URDF and Mesh Files
There was some cleaning required to get the URDF files to work.  Some of these steps could be removed by changing the export process in SolidWorks.  
```
# goto the directory with the meshes and URDF files
cd /whatever/car_assem3

cp -R meshes /mnt/enpm662p1/project1/src/terp1/
cp -R urdf   /mnt/enpm662p1/project1/src/terp1/
```

### Apply Any Required Fixes to URDF and Mesh Files
Some of this is personal preference, like down-casing the mesh files.  Some of this is REQUIRED, like fixing the file name, and the data in the URDF file.
```
#fix urdf file
cd /mnt/enpm662p1/project1/src/terp1/urdf
mv car_assem3.urdf terp1.urdf

#delete car_assem3.csv???

# fix xml header 
<?xml version="1.0"?>

# fix robot tag 
<robot name="terp1" xmlns:xacro="http://ros.org/wiki/xacro">

# replace car_assem3 with terp1  (vim command follows)
:%s/car_assem3/terp1/g
:%s/\.STL/.stl/g

# Manually, change joint names so they do not conflict with link names

# TODO: probably should go back and clean up the crazy float values

# fixup stl files in meshes directory, if needed (bash command follows)
for file in *; do mv "$file" "$(echo $file |tr '[:upper:]' '[:lower:]')"; done
```

# Add Launch Files
These can be written from scratch or yanked from other projects.  Here, I used templates that were from a former student or TA.  (linked in the Project Instructions)

NOTE: The Launch Files have changed ALOT in this project since originally pulling them from the repo noted below.  

```
# create package directories for launch files and world
mkdir /mnt/enpm662p1/project1/src/terp1/launch
mkdir /mnt/enpm662p1/project1/src/terp1/worlds

#bring down templates in safe space
cd /whatever/temp
git clone https://github.com/shantanuparabumd/ENPM-662-Introduction-to-Robot-Modelling.git
# cd into cloned directory and copy files
cp templates/launch/*.py /mnt/enpm662p1/project1/src/terp1/launch/
cp templates/worlds/empty_world.world /mnt/enpm662p1/project1/src/terp1/worlds/
```

## CMakeLists.txt
In the terp1 package, add install instructions for special directories, and any dependencies.

NOTE: This file has changed since adding the Teleop Demo.  
    
```
cd /mnt/enpm662p1/project1/src/terp1/
vim CMakeLists.txt
------
install(DIRECTORY urdf meshes launch worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/terp1 DESTINATION include)
install(PROGRAMS launch/robot_description_publisher.py DESTINATION lib/${PROJECT_NAME})

find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
```

## package.xml
In the terp1 package, fixup the header bits (email, version, license, etc.).  Note: email must be a properly formatted email address.
```
cd /mnt/enpm662p1/project1/src/terp1/
vim package.xml
------ 
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>terp1</name>
  <version>1.0.0</version>
  <description>Project1 Robot</description>
  <maintainer email="group1@enpm662">jfehrmann</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- begin jsf add -->
  <build_depend>rosidl_default_generators</build_depend>
  <depend>gazebo_ros_pkgs</depend>
  <depend>robot_state_publisher</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <!-- end jsf add -->

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <!-- jsf added next line -->
    <gazebo_ros gazebo_model_path="${prefix}/.." />
    
  </export>
</package>
```

## Modify Launch Files
There are multiple edits in multiple places to get the launch files working

NOTE:  There have been many changes to these files.  Too many to enumerate here.  

```
cd /mnt/enpm662p1/project1/src/terp1/launch
chmod 644 *.py
------
vim gazebo.launch.py
# change 'package_name' and 'test_package' to 'terp1'
------
vim spawn_robot_ros2.launch.py
# change 'package_name' and 'test_package' to 'terp1'
# note: two places inn "DATA INPUT" section, and one place lower
# note: file should be terp1.urdf, not terp1.urdf.xacro
------
```

## Build and Run
Run from .../project1 directory, not from .../project1/src/terp1.  This will keep you package directory from being polluted with build files.  
```
cd /mnt/enpm662p1/project1
colcon build
source install/setup.bash
```
