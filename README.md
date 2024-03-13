# RoboticsManipulationA5
Robotics Manipulation with Dynamixel Open Manipulator X
Tasks 2, 3, and 4 use the current position mode, which allows setting an arbitrary close and open goal position of the last servo (ID5) with the current limits restricting how tightly it clamps the object should it have a larger dimension than the close goal position. 

Task 1 Simulation
- 
- Forward Kinematics Calculation and Output Plots
- Forward Kinematics Simulation
- Inverse Kinematics Calculation and Output Plots
- Inverse Kinematics Simulation

Task 2 Blocks
- 
Given the spatial limitations of the robot arm, the x-y horizontal plane had to be split into suitable vs unsuitable for rotation. Suppose it is tasked to rotate a block outside the suitable region. In that case, it will move the block to an unoccupied holder in the suitable region, and rotate it, before returning it to its original holder.
- Functions (Pick up(stack), drop(stack), move, rotate)
- Code written to dynamixel for video

Task 3 Pen
-
The smooth trajectory for the arc is obtained from its projection on the y-axis. Hence, it is split into 4 cases on whether it exceeds the maximum y points of the circle and whether it is drawn on the upper half or bottom half of the circle.
- PreTraj - Finding the right centres and coordinates of circle should they not be given
- Traj - Functions (Getting coordinates of points for moving in a smooth trajectory)
- Picking up and returning pen functions
- Inverse Kinematics for each point
- Code written to Dynamixel for video
- STL and SLDPRT files of the pen cap and pen casing

Task 4 Apple picking
- 
The gripper design draws inspiration from both the Festo fin design and multi-finger grippers. This design allows the robot arm to effectively grasp irregularly shaped objects without exerting excessive pressure. This is particularly useful in tasks such as apple picking, where apples may have slight variations in size and necessitate gentle handling. This is an extension of Task 2 with the trajectory taken being arbitrary. It can be further extended into a claw machine with a joystick interface or incorporating ML and computer vision to determine the desired x,y,z and angles. 
- STL and SLDPRT files of gripper
- Functions (Pluck apple and placing it into a crate)
