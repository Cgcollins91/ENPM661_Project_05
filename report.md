## The Library Robot
Class: ENPM662 - Introduction to Robot Modeling
Due: November 8, 2024
![[Pasted image 20241208091358.png]]
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
The objective of this project was to design, model, and demonstrate a mobile robot that can retrieve and return books to and from library shelves.  The simulation will demonstrate the robot's ability to move to a specified location, retrieve a book from the robot's storage system, and place the book on a library shelf.

There are two motivations for this robot: 
- The robot can provide assistance with book retrieval tasks, such as if an individual is physically unable to reach a book off a shelf.  
- The robot can automate tasks  normally handled by library staff, such as reshelving returned booked.  
# Application
The design of the robot is inspired by the UJI librarian robot described here:
https://www.researchgate.net/publication/225367956_The_UJI_librarian_robot#fullTextFileContent

![[Pasted image 20241208090845.png]]

The UJI robot was able to autonomously locate and retrieve a book in an ordinary library, provided only with the book code and a library map. The UJI robot employed stereo vision, visual tracking, and other techniques to locate and retrieve the book. 

Group 1's robot simulation will focus on kinematics and will not include path planning and environmental response capabilities such as those found on the UJI librarian robot. 
# Robot Type

![[Pasted image 20241208090803.png]]

The Library robot is effectively two robots in a single package.  The base is a mobile robot consisting two drive wheels, two wheels for directional control, and six slots for storing books of various sizes.  The lower section will have a cavity capable of hold large batteries for extended runtime and additional ballast.  

Mounted to the top of the base robot is a manipulator arm with five links.  The links and joints have a compact design with a low center of gravity to facilitate locomotion without risk of tipping.  

The dimensions of the arm are based on those of the UR3 but the CAD model  was designed from scratch. The manipulator arm has a custom gripper capable of rotating 360 degrees and independently controlled pads. The base and storage slots are also custom.
# DOFs and Dimensions
![[Pasted image 20241208090934.png]]
The mobile base has three degrees of freedom that allow it to move across a large, flat workspace to a desired x-y coordinate and 2D orientation. The arm and gripper assembly combine for six degrees of freedom, allowing the gripper to be placed at any x-y-z coordinate and 3D orientation inside the robot's effective workspace.  

The robot has a circular footprint with a 600mm diameter, and the base/storage system sits approximately 720mm above the floor. The arm, when collapsed has a height of 900mm (above the base), and a maximum extension of 1850mm (above the base). The total overall height of the robot, at full extension is 2750mm, and has a sweeping radius (with arm extended fully perpendicular) of approximately 1950mm.
# CAD Model
The robot and additional asserts were fully modelled in SolidWorks and exported in URDF format.  Model files are included with this report.

![[Pasted image 20241208091343.png]]

# Frame Assignment

![[Pasted image 20241208091150.png]]
![[Pasted image 20241208091322.png]]
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
*Note 1: The code in this section was based on a team member's work product from an individual HW2/3 submission.  To protect the integrity of the homework assignments, code was not shared with other team members until after the due date of the HW3.*

*Note 2:  The code in this section focuses on the robot arm/manipulator.  Similar approach is used for the movement of the robot base.*  

The following code creates a single parametric transformation matrix for a given joint identifier (e.g., "θ1").  This code is used multiple times to create parametric matrices for each joint.
![[Pasted image 20241208092416.png]]
The following code performs matrix multiplication on the parametric transform matrices and returns a list of each joint's complete transform.  For example, the third element (second if considering base 0 indexing) in this list will have the complete transform of joint 3.
![[Pasted image 20241208092812.png]]
Lastly, the following code uses the transform for joint 6 (e.g., the 6th element of the transform list from the previous step) to return the forward kinematic equation for the arm's end effector.  Please note, this position is relative to the robot's mobile base since we consider the base and arm to act as two independent robots.  It also does not account for additional transformations performed by the gripper assembly. 
![[Pasted image 20241208092523.png]]
Below is the output from the above `forward_position_kinematics` solution.  It is a parameterized version of `{x,y,z,Rx,Ry,Rz}`.  The values for α, d, and r have been resolved since those are constant values in the kinematic chain.  
![[Pasted image 20241208093712.png]]

# Inverse Kinematics
*Note 1: The code in this section was based on a team member's work product from an individual HW2/3 submission.  To protect the integrity of the homework assignments, code was not shared with other team members until after the due date of the HW3.*

*Note 2:  The code in this section focuses on the robot arm/manipulator.  Similar approach is used for the movement of the robot base.*  

The following code derives the linear elements of the Jacobian matrix using the partial derivatives method.  The output of this function is a 3x6 matrix, three rows representing the x, y, and z functions, and six columns representing partial derivatives with respect to joints θ1 through θ6.
![[Pasted image 20241208100126.png]]

The  following is the output of the 3x6 linear matrix representing the top half of the Jacobian Matrix produced by the `jacobian_linear_functions` code above.
![[Pasted image 20241208100523.png]]

Using the transforms list (produced by `prebuild_transforms`) in the Forward Kinematics section, we can substitute theta values and extract the angular rows of the Jacobian.  The output from this function is a 3x6 matrix, with each column representing the incremental z values from the transformation matrices.  
![[Pasted image 20241208100946.png]]

The complete and evaluated Jacobian can be resolved by stacking the linear and angular submatrices together and substituting the current joint values.  
![[Pasted image 20241208101711.png]]

# Forward Kinematics Validation
Geometric validation of the forward kinematics chain was performed using transformation matrices in an Excel spread sheet.  This methodology was adapted from a YouTube video by Chris Annin of Annin Robotics (see references).  A copy of the excel file is included with this report.
![[Pasted image 20241208102737.png]]
Using this "spreadsheet model" we can change joint values in isolation and confirm the output is as expected.

Rotating J1 by 90 degrees should rotate the robot's arm around the Z axis.  As we can see in the table below, the z value of the output does not change, the Rz value rotates by 90 degrees, and the absolute x/y values swap values.  Ry and Rx do not change.  Next, we rotated joints J2 - J5 individually by 90 degrees to confirm our DH table is correct. The plots below show the Home position vs. 90 degree rotation of each joint.

![[Pasted image 20241208103306.png]]
![[Pasted image report_images/Case1_project2.png]]
![[Pasted image report_images/Case2_project2.png]]
![[Pasted image report_images/Case3_project2.png]]
![[Pasted image report_images/Case4_project2.png]]
![[Pasted image report_images/Case5_project2.png]]


For the next validation, we can rotate J2 and J3 by the same angle.  With the design of this manipulate, if J2 and J3 are rotated by the same value, we should see the end effector move away and down from the arm's origin, without any rotational changes.  As shown in the table below, compared with the at-rest position, there is an absolute value increase in y (outward motion), a decrease in z (downward motion), and no change in x (lateral motion).  Also, there was no change in orientation of the end effector (Rz, Ry, Rx).
![[Pasted image 20241208104024.png]]

Joints J3 and J4 can be rotated together for a similar validation as J2 and J3.  However, this will cause the end effector to raise above the origin instead of lower.  Below, we can see the output is as expected:  No change in x or in orientation.  The z value has increased (upward motion), and the absolute y value has also increase (outward motion).  
![[Pasted image 20241208104646.png]]

Below, J5 is rotated 45 degrees, which is reflected in the Rz rotation, and in changes in the x/y position of the end effector.  No change to z or to other rotational axes is as expected.
![[Pasted image 20241208104949.png]]

Finally, J6 is rotated which, as expected, only affects the Ry value when the other joints are in the home position.  
![[Pasted image 20241208105031.png]]

# Robot Workspace 
We assessed the robot workspace by limiting joint movement to +/- 90 degrees and running every combination of 4 linearly spaced joint angles from -90 to +90 degrees. The constraint on +/-90 degrees is due to the possibility of joint combinations where the end effector would collide with the floor. This constraint could later be relaxed leveraging logic to limit specific joint based on the position of other joints to avoid floor collisions. The below plot shows the end effector position at each point run and the shaded blue indicates possible positions for the end effector.

![[Pasted image report_images/Robot_Workspace_Project2.png]]

# Inverse Kinematics Validation
*Note 1: The code in this section was based on a team member's work product from an individual HW2/3 submission.  To protect the integrity of the homework assignments, code was not shared with other team members until after the due date of the HW3.*

*Note 2:  The code in this section focuses on the robot arm/manipulator.  Similar approach is used for the movement of the robot base.*  

For inverse kinematics validation, the team leveraged HW2/3's requirement to draw the rounded rectangle or "keystone" shape.  Using this code was as simple as replacing the DH table values with values that represented the manipulator used in this project.  

To account for singularities and other oddities when certain joints were near orthogonal positions, the team employed a Dampened Least Squares model.  The code for this was derived from exercises and examples provided by 
Kreyszig (Advanced Engineering Mathematics), Lay (Linear Algebra and Its Applications), and Press (Numerical Recipes).  It is important to note that this algorithm may not be implemented perfectly.  However, it generally worked as expected, correcting for singularities, and was thus viable for this project.  
![[Pasted image 20241208105738.png]]

Using the Dampened Least Squares algorithm and a scaling lambda factor, the fully computed (i.e., numerical) inverse Jacobian can be used to validate inverse kinematics.  Since this is a numerical inverse (due to computational limitations on deriving a parametric inverse), it must be computed iteratively for specific joint values.
![[Pasted image 20241208105628.png]]

New joint values can be calculated by numerically integrating the theta velocities with current joint values.  
![[Pasted image 20241208112531.png]]

Use the methods described above, the path of the end effector and the projection of the arm can be plotted in 3D space.  These can be visually compared to the model in Gazebo by nudging the joints until the model's joint values map to those in the 3D plots.  Below are samples showing the full 3D plot versus the Gazebo model in its starting position.  

Please note, the 3D plot and the Gazebo model will not perfectly align.  This is due to two limitations.  First, the two visualizations do not share synchronized camera positions.  Therefore, parallax error and perspective distortion will cause the images to look different.  Second, the 3D plot is limited to drawing lines from the origin of one link to the origin of the next link.  Since in DH, the modeled origin does not always align with the actual origin, there are deviations for the intermediate links.  This is notably visible for the offset from link1 to link2.  In the model, there is clearly an offset in the join.  However, in the plot, this offset is proportionately spread over the entire link.  

The 3D plot of the end effector and the projection of the arm. 
![[Pasted image 20241208105359.png]]

The Gazebo model in the same starting position as used for the plot above.  Starting joint values are approximately (0, 1, 1, 0, 0, 0) radians. 
![[Pasted image 20241208105415.png]]

Side view of the plot of the starting joint values, (0, 1, 1, 0, 0, 0)radians.  (This is just the starting position, not the entire keystone plot shown previously)
![[Pasted image 20241208105444.png]]

The Gazebo model in the same starting position as used for the plot above.  
![[Pasted image 20241208105433.png]]

Forward view of the plot of the starting joint values, (0, 1, 1, 0, 0, 0)radians.  (This is just the starting position, not the entire keystone plot shown previously)
![[Pasted image 20241208105500.png]]

The Gazebo model in the same starting position as used for the plot above.  
![[Pasted image 20241208105524.png]]
# Assumptions
The following assumptions were made for this project:
- Robot navigation will be simulated by following a pre-determined path.  The robot will not determine a path based on identifying the home location of an arbitrary book.
- The robot will not use environmental feedback as part of its path planning.  Course changes will be limited to adjustments made by the PID controller to compensate for trajectory errors.  The robot will not make adjustments for path obstructions.
- The act of picking up a book will be simulated.  Handing the dynamics associated with picking up an object in Gazebo are beyond the scope of this project.  
# Control Method
The robot has two control methods, a Teleop controller and an autonomous controller.  The Teleop Controller provides forward, reverse and turning controls for the robot base, as well as individual controls for each joint and gripper.  The Teleop controller reports joint positions that can be recorded for use in the autonomous controller.  

The autonomous controller works by replaying recorded joint angles for the arm and using a PID controller to assist in steering the robot to a destination position.  The autonomous controller receives its input in the form of ROS parameters.

- **`goal {x, y}`** parameter to send robot to specific location.
- **`arm_goal {a, b, c, d, e}`** parameter to recall position of each arm joint.
- **`gripper_goal {a, b, c, d}`** parameter to orient and manipulate the gripper.

# Gazebo / RViz Visualization
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
