# Robotic Manipulation Final Project

## Overview
This project simulates the movement of a robotic manipulator transporting a cube from an initial position to a final position. It utilizes the **Modern Robotics library** and **CoppeliaSim** for modeling, trajectory generation, and control.

The project is divided into three milestones, as described below.

---

## Milestones

### **Milestone 1: Configuration Calculation**
- Implements a discrete-time algorithm to calculate the robot's next configuration.
- Simulates the step-by-step motion of the manipulator towards a goal configuration.

---

### **Milestone 2: Trajectory Generation**
- Implements the `Checkpoint2_TrajectoryGenerator` function to:
  - Generate a reference trajectory for the robotic end-effector to grasp a cube, transport it to a target location, and return to rest.
  - Define motion segments using initial and final poses, grasp and standoff configurations, and resolution parameter `k`.
  - Save the trajectory as `GeneratedTrajectory.csv` for further use.

#### **How to Use:**
1. Configure the required transformation matrices (initial, final, grasp, standoff) in the script.
2. Call the `TrajectoryGenerator` function with the desired resolution parameter `k`.
3. Run the trajectory in **CoppeliaSim Scene8**.

#### **Dependencies:**
- Ensure the `modern_robotics` Python library is installed.

---

### **Milestone 3: Combined Control**
- Combines **feedforward** and **feedback** control:
  - Tracks the generated trajectory.
  - Aligns the robot's current position and orientation with the desired position and orientation.
- Computes commanded velocities to ensure smooth and accurate motion.

---

## Running the Project
1. Clone this repository:
   ```bash
   git clone git@github.com:m0therb0ardd/Robotic-Manipulation-Final.git
   cd Robotic-Manipulation-Final
