# ENPM662 Project 2 Final Report
## The Library Robot
Class: ENPM662 - Introduction to Robot Modeling

Due: November 8, 2024

# Team Members
- Anne-Michelle Lieberson (aliebers@umd.edu)
	- ROS Development
	- Demo Orchestration
- Christopher Collins (ccollin5@umd.edu)
	- ROS Development
	- World Building
- Daniel Zinobile (zinobile@umd.edu)
	- Robot Design
	- SolidWorks Modeling
- James Fehrmann (jsf@umd.edu)
	- DevOps
	- ROS Development

# Introduction
The objective of this project was to design, model, and demonstrate a mobile robot that can retrieve and return books to and from library shelves. The simulation will demonstrate the robot's ability to move to a specified location, retrieve a book from the robot's storage system, and place the book on a library shelf.

There are two motivations for this robot: 
- The robot can provide assistance with book retrieval tasks, such as if an individual is physically unable to reach a book off a shelf.  
- The robot can automate tasks  normally handled by library staff, such as reshelving returned booked.  

# Application
The design of the robot is inspired by the UJI librarian robot described here:
https://www.researchgate.net/publication/225367956_The_UJI_librarian_robot#fullTextFileContent

![image](https://github.com/user-attachments/assets/887e41de-fb06-4eb6-ad77-a863a20a5dfb)

_An image of the UJI as pictured in the original paper_

The UJI robot was able to autonomously locate and retrieve a book in an ordinary library, provided only with the book code and a library map. The UJI robot employed stereo vision, visual tracking, and other techniques to locate and retrieve the book. 

Group 1's robot simulation will focus on kinematics and will not include path planning and environmental response capabilities such as those found on the UJI librarian robot. 

# Robot Type

![image](https://github.com/user-attachments/assets/fc5d0e8f-c646-4e03-b373-34a51cc6c7b5)

The Library robot is effectively two robots in a single package. The base is a mobile robot consisting 2 drive wheels, 2 wheels for directional control, and 6 slots of various sizes for storing books. The lower section will have a cavity capable of hold large batteries for extended runtime and additional ballast.  

Mounted to the top of the base robot is a manipulator arm with 5 links.  The links and joints have a compact design with a low center of gravity to facilitate locomotion without risk of tipping.  

The dimensions of the arm are based on those of the UR3 but the CAD model itself was designed from scratch. The manipulator arm has a custom gripper capable of rotating 360 degrees and independently controlled pads. The base and storage slots are also custom.

--- ARE STATMENTS ON MODEL CORRECT?

# DOFs and Dimensions
The mobile base has three degrees of freedom that allow it to move across a large, flat workspace to a desired x-y coordinate and 2D orientation. The arm and gripper assembly combine for six degrees of freedom, allowing the gripper to be placed at any x-y-z coordinate and 3D orientation inside the robot's effective workspace.  

The robot has a circular footprint with a 600mm diameter, and the base/storage system sits approximately 720mm above the floor. The arm, when collapsed has a height of 900mm (above the base), and a maximum extension of 1850mm (above the base). The total overall height of the robot, at full extension is 2750mm, and has a sweeping radius (with arm extended fully perpendicular) of approximately 1950mm.

# CAD Model
The robot and additional assets were fully modelled in SolidWorks and exported in URDF format.  Model files are included with this report.

![image](https://github.com/user-attachments/assets/b2300898-2e23-4363-b82e-846c66ebfebf)
![image](https://github.com/user-attachments/assets/eda76730-fc9d-4556-8520-5cb1004f0701)

The CAD models for the library environment were sourced from 

# Frame Assignment

-- INSERT FRAME DRAWING
-- INSERT FRAME PICTURE

# DH Parameters 

|     | θ,z  | α,x  | d     | r     |
| --- | ---- | ---- | ----- | ----- |
| J1  | -90° | -90° | 50mm  | 0mm   |
| J2  | -90° | 180° | 50mm  | 850mm |
| J3  | 180° | 0°   | 100mm | 750mm |
| J4  | 90°  | -90° | 50mm  | 0mm   |
| J5  | 90°  | 90°  | 200mm | 0mm   |
| J6  | 0°   | 0°   | 50mm  | 0mm   |

# Forward Kinematics
*Provide the final transformation matrix, verifying your end effector coordinates. You can copy paste the code output here or provide a screenshot of the output.*

-- INCOMPLETE

# Inverse Kinematics
*Explain how to derive the inverse kinematics using Jacobian matrix.*  
*Provide the Jacobian matrix. You can copy paste the code output here or provide the screenshot of the output.*

The Jacobian matrix was determined via the first method described in class (for code, see `enpm662p2/project2/src/terp2_arm_controller/terp2_arm_controller/hw_3_project_2.py`). All of the robot arm joints are revolute, and so the following applies for all joints:

![image](https://github.com/user-attachments/assets/143912ce-7a53-4dbe-8b1b-bfd1347cdcc9)

The first step in this method is to define the DH table, from which sequential transformation matricies can be extracted (transform from frame 1 to 2 called _T12_, from 2 to 3 for _T23_, etc.). These matricies can then be used to form the transformation matricies from frame 0 to i (_T0i_), which are needed for the calculation of each _Z_ and _O_ matrix in the Jacobian. _Z_ at _i-1_ is composed of the first three elements of the 3rd column of _T0i_, while _O_ at _i_ is the first three elements of the 4th column of _T0i_. The final Jacobian can be created by considering each _J_ at _i_ as a column in the Jacobian.

-- INCOMPLETE, PERHAPS CAN BE FORMATTED SOMEHOW BETTER AS WELL?

# Forward Kinematics Validation
*Either geometrically validate the FK i.e. start with some known joint angles and verify the end effector position using the final transformation matrix. Show diagrammatic representations of the robot at the known joint values and validate the transformation matrix you get.*  
*Show representations and validations for at least 3 configurations*

--INCOMPLETE

# Inverse Kinematics Validation
*Validate the IK either manually i.e. start with some known end effector position and find out the joint angles to validate OR create a python script just like HW1/2 to validate the IK. Make sure to include the figure of the plot output of that script (just like in HW1/2).*  
*Note: You can plot a figure of your choice (straight line or arc or anything depending on your application). You can start with a configuration of your choice and choose the plane you want to draw the figure on.*  
*Make sure you specify the required trajectory, and how you compute the tool velocity for the required trajectory.*

-- INCOMPLETE

# Additional Validations

-- NOT A REQUIRED section, but it should add some meat to our report

In addition to the validations performed above, a workspace study of the arm was conducted. This was acheived by limiting the base joint to -90 and +90 while placing no limits on the other joints, and then joint positions were input at random and a point cloud of the end effector was generated:

![image](https://github.com/user-attachments/assets/6d2d1a43-ac03-4d8d-b422-df57ae80aab8)

-- IS THIS CORRECT? Also, is there code for this that we can include in the repo?

# Assumptions
The following assumptions were made for this project:
- Robot navigation will be simulated by following a pre-determined path.  The robot will not determine a path based on identifying the home location of an arbitrary book.
- The robot will not use environmental feedback as part of its path planning.  Course changes will be limited to adjustments made by the PID controller to compensate for trajectory errors.  The robot will not make adjustments for path obstructions.
- The act of picking up a book will be simulated.  Handing the dynamics associated with picking up an object in Gazebo are beyond the scope of this project.  

# Control Method
The robot has two control methods, a Teleop controller and an autonomous controller.  The Teleop Controller provides forward, reverse and turning controls for the robot base, as well as individual controls for each joint and gripper.  The Teleop controller reports joint positions that can be recorded for use in the autonomous controller.  

The autonomous controller works by replaying recorded joint angles for the arm and using a PID controller to assist in steering the robot to a destination position.  The autonomous controller receives its input in the form of ROS parameters.

- **goal {x, y}** parameter to send robot to specific location.
- **arm_goal {a, b, c, d, e}** parameter to recall position of each arm joint.
- **gripper_goal {a, b, c, d}** parameter to orient and manipulate the gripper.

# Gazebo / Rviz Visualization
A Gazebo demo using the autonomous controller can be found on YouTube, at the following link.  The video includes demonstrating how to spawn the robot and bootstrap the controllers.   

https://www.youtube.com/watch?v=lB9bhnnsFkw

0:00 - Spawn Gazebo World
0:30 - Boot Autonomous Controller
0:40 - Run Autonomous Demo Script
0:48 - DEMO

RViz was not used for this project.

# Problems Faced
This project had an ambitious scope.  Many topics were new to every team member.  Lots of “hacking and slashing" was required to get things working in Gazebo, without fully understanding each software component.   This resulted in no time for refinement or major changes.  

The simulation had systemic fragility.  There were too many sources for issues: tedious math, ROS and its ecosystem of plugins, multiple programming languages. This made it hard to narrow down problems and find root causes.

Time management was also challenging for this project.  It was hard to balance time between learning mathematics modeling topics and spending time debugging ROS issues.  Ultimately, the team was forced to accept best efforts over best outcomes.  

Lastly, time pressures forced team members into fixed roles.  There was no time or capacity for team members to switch roles.  This unfortunately left some team members with no opportunity to step outside comfort zone.

# Conclusion
Overall, the Library Robot project was successful;  many of the goals were met and the team was able to present a viable design for the intended application.  The team was exposed to the challenges associated with using Gazebo's physics engine and gained experience in developing nodes for ROS.  There were also auxiliary benefits for the team such as experience working with Docker and GitHub.  

If there was an opportunity to rework the project, the team would like to have better synchronization between the DH-based model and the Solidworks model.  The team would also like to have better integration between the kinematics modeling and ROS modeling.  

Some the team's most notable successes included:
- **DevOps:**  Codified  a portable runtime and dev environment which will be tremendously useful for future projects.  
- **ROS:**  Learned basic ROS concepts such as spawning a robot, creating nodes, topics, parameters, etc.  
- **PID Controller:**  Implemented a PID Controller.
- **Quaternion Math:**  Worked with quaternion representations for robot orientation, and converting to roll-pitch-yaw, Euler angles, and Transform matrices.  
- **Gazebo:**  Gained experience working in Gazebo and parsing Gazebo state data.

Ultimately, given the team's lack of experience with this subject matter, and the compressed schedule of the project, having a working Gazebo simulation can be considered a successful outcome.

# Future Work
The following concepts could lead to future work or extensions of this project:
- Use LiDAR to adjust the robots position relative to the surrounding bookshelves when putting a book into its home location.
- Use cameras or environmental sensors to identify bookshelf capacity for the book in the gripper.
- Add waypoints to the environment and let the robot determine the best path from its current location to its intended destination.
- Develop a "reload" scenario for simulation.  In this scenario the robot would load or reload books from its starting area and then proceed to put the additional book away in the library.

# GitHub Link
The code for this project can be found at: https://github.com/jamesfehrmann/enpm662p2

Please contact James Fehrmann at jsf@umd.edu or 202-285-8025 if the repository is not accessible.  

The following steps can be used to run the demo:
- Clone the repository to ~/enpm662p2, ~/cpp/enpm66p2, or ~/projects/enpm662p2.  Cloning to another location will result in the repo not being mounted to the container.
- Run `init.sh` to build and launch the container.   If the container is running but you are not connected, you can run `attach.sh` to connect.  Reviewing the code of these bash scripts will reveal clues on how the project is configured to run in Docker.
- Once in the Docker container (and assuming the project files mounted correctly, as noted in the first step), run the following scripts:
	- `project2/terp2_build.sh` = build the project files
	- `project2/terp2_empty.sh` = run the gazebo simulation
	- `project2/terp2_teleop.sh` = run the teleop controller
	- `project2/terp2_controller.sh` = run the autonomous controller
	- `project2/terp2_library_nav.sh` = run the library demo (requires controller online)
- The `project2` directory includes other interesting scripts (for example, `set_goal.sh` and `set_arm.sh`) that work with the autonomous controller.  

# Package Submission
See instructions under "GitHub Link" for how to run the demo, or see the video in the Gazebo Simulation section.

# References 
Annin, Chris. 6 Axis Robot...with the AR4-MK2. https://www.youtube.com/watch?v=FNuiNmoqaZM. Accessed Nov. 2024

cppreference.com. C++ Reference. https://en.cppreference.com/w/cpp. Accessed Oct. 2024.

Docker. Dockerfile Reference. https://docs.docker.com/reference/dockerfile/. Accessed Oct. 2024.

Eater. Quaternions. https://eater.net/quaternions. Accessed Oct. 2024.

Kreyszig, Erwin. Advanced Engineering Mathematics. 9th ed., Wiley, 2006.

Lay, Steven R., et al. Linear Algebra and Its Applications. 6th ed., Pearson, 2020.

Lynch, Kevin M., and Frank C. Park. Modern Robotics: Mechanics, Planning, and Control. Cambridge University Press, 2017.

Open Robotics. ROS 2 Documentation: Humble. https://docs.ros.org/en/humble/. Accessed Oct. 2024.

Parab, Shantanu. ENPM662 Introduction to Robot Modeling. https://enpm-662introduction-to-robotmodelling.readthedocs.io/en/latest/. Accessed Oct. 2024.

Press, William H., et al. Numerical Recipes, The Art of Scientific Computing. 3rd ed., Cambridge University Press, 2007

Prats, M., Martínez, E., Sanz, P.J. _et al._ The UJI librarian robot. _Intel Serv Robotics_ **1**, 321–335 (2008). 
https://doi.org/10.1007/s11370-008-0028-1

Python Software Foundation. The Python Language Reference. https://docs.python.org. Accessed Nov. 2024.

ros2_control Development Team. ros2_control documentation. https://control.ros.org/rolling/index.html. Accessed Oct. 2024.

Spong, Mark W., et al. Robot Modeling and Control. 2nd ed., Wiley, 2020.

Stewart, James. Calculus: Early Transcendentals. 9th ed., Cengage Learning, 2021.

SymPy Development Team. SymPy: Python Library for Symbolic Mathematics. https://www.sympy.org. Accessed Nov. 2024.

∎
