# EMPM662 Project 2 Final Report for Group 1

## Introduction
Brief Introduction about your project.

## Application
Describe the aim and scope of your robotic solution

## Robot Type
Discuss the Robot Type i.e. a manipulator, mobile robot, gantry robot etc. Justify the choice of robot type.

## DOFs and Dimensions
Describe the DOFs for your robot and any external system you may have, separately. Justify the DOF chosen with explanation.

## CAD Model
Provide .PRT and .SLDASM files of your model.

## Frame Assignment
Show the proper frame assignment compatible with DH convention  
Show the robot image and the expected joint rotations and directions.  
Make sure to have 2 images for both of the above.

## DH Parameters
Provide the DH parameters table for your robot.

## Forward Kinematics
Provide the final transformation matrix, verifying your end effector coordinates. You can copy paste the code output here or provide a screenshot of the output.

## Inverse Kinematics
Explain how to derive the inverse kinematics using Jacobian matrix.  
Provide the Jacobian matrix. You can copy paste the code output here or provide the screenshot of the output.

## Forward Kinematics Validation
Either geometrically validate the FK i.e. start with some known joint angles and verify the end effector position using the final transformation matrix. Show diagrammatic representations of the robot at the known joint values and validate the transformation matrix you get.  
Show representations and validations for at least 3 configurations

## Inverse Kinematics Validation
Validate the IK either manually i.e. start with some known end effector position and find out the joint angles to validate OR create a python script just like HW1/2 to validate the IK. Make sure to include the figure of the plot output of that script (just like in HW1/2).  
Note: You can plot a figure of your choice (straight line or arc or anything depending on your application). You can start with a configuration of your choice and choose the plane you want to draw the figure on.  
Make sure you specify the required trajectory, and how you compute the tool velocity for the required trajectory.

## Assumptions
Describe the assumptions you may have considered for your project like the external environment conditions, materials, etc.

## Control Method
Describe which control method/controllers you used and reason behind choosing it.

## Gazebo / Rviz Visualization
Gazebo Visualization: Include a video link to your final working simulation in Gazebo.  
RViz Visualization: Not mandatory; it is only required if you had indicated in your Project Proposal that you'd be using additional sensors/tools like LiDAR or Camera, etc.  

## Problems Faced
Briefly discuss the challenges faced in your project.

## Conclusion
Summarize your project, discuss what worked for you and what didn't. Does your robot fulfill it's proposed application.

## Future Work
Any future work that can be implemented or additional features that can be added

## GitHub Link
The GitHub repository should contain a proper README file with all the dependencies and instructions on how to build and run the ROS package.

## Package Submission
A working package submission build using ROS2

## References
