Overview:
This project simulates the movement of a robotic manipulaotr moving a cube from an inital position to a final 
position using Modern Robotics library and CoppeliaSim.

The project was broken into  milestones described below. 

Milestone 1: 
Checkpoint1 calculates the nubots next confication at discrete time steps.

Milestone 2:
Checkpoint2_TrajectoryGenerator generates a reference trajectory for a robotic end-effector to grasp a cube, transport 
 it to a target location, and return to rest. By configuring initial and final positions, as well as 
 grasp and standoff poses, the system calculates a series of transformation matrices that define smooth motion segments, 
 saving the resulting trajectory to a CSV file for further use.

To use this, call the TrajectoryGenerator function with the required transformation matrices and resolution parameter k. 
The generated trajectory will be saved as GeneratedTrajectory.csv. Ensure the required library, modern_robotics, is installed. 
For this assignment run the csv in CoppeliaSim Scene8. 

Milestone 3: 
Checkpoint3 combines feedforward and feedback control to compute the commanded velocities. This allows current orientation 
and position to a desired orientation and position while tracking the trajectory. 


