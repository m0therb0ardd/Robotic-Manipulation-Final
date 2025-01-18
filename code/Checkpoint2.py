import numpy as np
import modern_robotics as mr 
import csv

def TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k):
    """
    Generates reference trajectory for end effector frame. 

    This trajectory consists of eight segments (beginning and ending at rest).
    The segments include moving to the cube, 
    grasping it, moving to the desired final configuration, placing it, 
    and returning to a standoff position. The trajectory is output as a 
    sequence of configurations and saved to a CSV file.

    1. Move to grasp standoff
    2. Move to grasp
    3. Grasp
    4. Move back up to grasp standoff
    5. Move to goal configuration standoff
    6. Move to goal configuration
    7. Release the block
    8. Move gripper back up to goal configuration standoff  

    Parameters:
    ----------
    T_se_initial : numpy.ndarray
        4x4 transformation matrix representing the initial configuration of 
        the end-effector in the space frame {s}.
        
    T_sc_initial : numpy.ndarray
        4x4 transformation matrix representing the initial configuration of 
        the cube in the space frame {s}.
        
    T_sc_final : numpy.ndarray
        4x4 transformation matrix representing the final configuration of 
        the cube in the space frame {s}.
        
    T_ce_grasp : numpy.ndarray
        4x4 transformation matrix representing the configuration of the 
        end-effector relative to the cube when grasping it.
        
    T_ce_standoff : numpy.ndarray
        4x4 transformation matrix representing the standoff configuration 
        of the end-effector relative to the cube before and after grasping.
        
    k : int
        Number of reference trajectory points per 0.01 seconds. This 
        determines the resolution of the trajectory.

    Returns:
    -----------
    trajectory: numpy.ndarray
        An N x 13 array where each row represents a configuration of the 
        end-effector at a specific time step. The first 12 values are the 
        flattened top 3 rows of the 4x4 transformation matrix of the 
        end-effector frame {e}, and the 13th value is the gripper state 
        (0 for open, 1 for closed).
    """

    trajectory = []  # This will hold all configurations
    dt = 0.01 / k

    standoff_duration = 4
    grasp_duration = 2
    move_duration = 2
    gripper_duration = 0.625  # Given in notes this is how long it takes the gripper to fully open/close

    # Intermediate transformations found with matrix multiplication 
    T_se_standoff_initial = T_sc_initial @ T_ce_standoff
    T_se_grasp = T_sc_initial @ T_ce_grasp
    T_se_standoff_final = T_sc_final @ T_ce_standoff
    T_se_release = T_sc_final @ T_ce_grasp

    # STEP 1: Move to initial standoff
    N1 = int(standoff_duration / dt)
    if N1 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_1 = mr.ScrewTrajectory(T_se_initial, T_se_standoff_initial, standoff_duration, N1, 3)
    if traj_1 is None:
        raise ValueError("ScrewTrajectory returned None.")
    for T in traj_1:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [0])  # Gripper state 0 for open

    # STEP 2: Move to grasp position 
    N2 = int(grasp_duration / dt)
    if N2 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_2 = mr.ScrewTrajectory(T_se_standoff_initial, T_se_grasp, grasp_duration, N2, 3)
    if traj_2 is None:
        raise ValueError("ScrewTrajectory returned None.")
    for T in traj_2:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [0])  # Gripper state 0 for open

    # STEP 3: Grasp cube (close the gripper meaning set grip_state to 1)
    N3 = int(gripper_duration / dt)
    if N3 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_3 = [T_se_grasp] * N3  # Stay in grasp position
    for T in traj_3:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [1])  # Gripper state 1 for closed

    # STEP 4: Move to position with cube (gripper still closed meaning gripper state is 1)
    N4 = int(standoff_duration / dt)
    if N4 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_4 = mr.ScrewTrajectory(T_se_grasp, T_se_standoff_initial, standoff_duration, N4, 3)
    if traj_4 is None:
        raise ValueError("ScrewTrajectory returned None.")
    for T in traj_4:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [1])  # Gripper state 1 for closed

    # STEP 5: Move to global configuration standoff
    N5 = int(move_duration / dt)
    if N5 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_5 = mr.ScrewTrajectory(T_se_standoff_initial, T_se_standoff_final, move_duration, N5, 3)
    if traj_5 is None:
        raise ValueError("ScrewTrajectory returned None.")
    for T in traj_5:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [1])  # Gripper state 1 for closed

    # STEP 6: Move to goal configuration
    N6 = int(grasp_duration / dt)
    if N6 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_6 = mr.ScrewTrajectory(T_se_standoff_final, T_se_release, grasp_duration, N6, 3)
    if traj_6 is None:
        raise ValueError("ScrewTrajectory returned None.")
    for T in traj_6:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [1])  # Gripper state 1 for closed

    # STEP 7: Release the block (open the gripper)
    N7 = int(gripper_duration / dt)
    if N7 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_7 = [T_se_release] * N7  # Stay in release position
    for T in traj_7:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [0])  # Gripper state 0 for open

    # STEP 8: Move gripper back up to goal configuration standoff
    N8 = int(standoff_duration / dt)
    if N8 < 2:
        raise ValueError("Number of trajectory points must be at least 2. Adjust duration or dt.")
    traj_8 = mr.ScrewTrajectory(T_se_release, T_se_standoff_final, standoff_duration, N8, 3)
    if traj_8 is None:
        raise ValueError("ScrewTrajectory returned None.")
    for T in traj_8:
        flattened_T = T[:3, :3].flatten().tolist() + T[:3, 3].tolist()
        trajectory.append(flattened_T + [0])  # Gripper state 0 for open

    return trajectory


# Define the screw axes B
B1 = np.array([0, 0, 1, 0, 0.033, 0])      # Screw axis for joint 1
B2 = np.array([0, -1, 0, -0.5076, 0, 0])   # Screw axis for joint 2
B3 = np.array([0, -1, 0, -0.3526, 0, 0])   # Screw axis for joint 3
B4 = np.array([0, -1, 0, -0.2176, 0, 0])   # Screw axis for joint 4
B5 = np.array([0, 0, 1, 0, 0, 0])          # Screw axis for joint 5
B = np.column_stack((B1, B2, B3, B4, B5))  # 6x5 matrix

M_0e = np.array ([[1, 0 , 0 , 0.033],
                 [0, 1, 0, 0, ],
                 [0, 0, 1, 0.6546],
                 [0, 0, 0, 1]])

#we can assume T_0e is the same as the given M_0e because the arm starts at the home configuration
T_0e = np.array ([[1, 0 , 0 , 0.033],
                 [0, 1, 0, 0, ],
                 [0, 0, 1, 0.6546],
                 [0, 0, 0, 1]])

phi = np.radians(0.0)  # set to 0 
x = 0.0  # X position (in meters) set to 0 
y = 0.0  # Y position (in meters) set to 0 

q = np.array([phi, x, y])

# Define the homogeneous transformation matrix T_sb
T_sb = np.array([
    [np.cos(phi), -np.sin(phi), 0, x],
    [np.sin(phi),  np.cos(phi), 0, y],
    [0,            0,           1, 0.0963],
    [0,            0,           0, 1]
])

T_b0 = np.array ([[1, 0 , 0 , 0.1662],
                 [0, 1, 0, 0, ],
                 [0, 0, 1, 0.0026],
                 [0, 0, 0, 1]])


T_se = T_sb @ T_b0 @ T_0e

T_se_initial = T_se

print(T_se_initial)


#T_sc_inital = cubes intital configuration
# this was a given 
T_sc_initial = np.array ([[1, 0 , 0 , 1],
                         [0, 1, 0, 0, ],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

#T_sc_final = cubes desired final configuration
# this was a given 
T_sc_final = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])


# you just guessed pretty much the smae there is a negatigve somewhere in rotation pary of like identiy stuff 
#T_ce_grasp = end effectors cofiguration relative to cube when grasping the cube 


T_ce_grasp = np.array([[0, 0, 1,0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0],
                      [0, 0, 0, 1]])


#t_ce_standoff = end effectors standoff configuration above the cube before and after graspong relative to the cube 

T_ce_standoff = np.array([[0, 0, 1,0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0.25], # some small posive value 
                      [0, 0, 0, 1]])

k = 1 # number of reference configurations per 0.01 seconds 




# Save trajectory to CSV
def save_trajectory_to_csv(trajectory, filename="checkingtrajactory.csv"):
    """
    Saves the trajectory to a CSV file.
    """
    print(f"Saving trajectory to {filename}...")
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(trajectory)
    print(f"Trajectory saved to {filename} successfully.")

# Generate and print trajectory
trajectory = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k)

# Print trajectory to the console
print("\nGenerated Trajectory:")
for row in trajectory:
    print(row)

# Save trajectory to a CSV file
save_trajectory_to_csv(trajectory)
