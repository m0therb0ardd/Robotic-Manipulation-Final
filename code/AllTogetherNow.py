import modern_robotics as mr
import numpy as np
########## Begin Citation 2 ###########
np.set_printoptions(suppress=True) #removes exponenets 
######### End Citation 2 ###########
import matplotlib.pyplot as plt


def NextState(current_config, speed, timestep, max_omg):
    """
    Computes the next state of the robot given the current configuration, speed, timestep, and max angular velocity.
    """

    # Limit speed to max angular velocity
    speed = np.clip(speed, -max_omg, max_omg)

    # Constants
    r_wheel = 0.0475  # Radius of the wheel
    l_chassis = 0.235  # Half the length of the chassis
    w_chassis = 0.15  # Half the width of the chassis

    # Parse current configuration
    chassis_config = np.array(current_config[:3])  # Chassis configuration [phi, x, y]
    arm_config = np.array(current_config[3:8])    # Arm joint angles
    wheel_config = np.array(current_config[8:])   # Wheel angles

    # Compute new joint angles
    joint_speeds = speed[:5]
    delta_theta = joint_speeds * timestep
    new_joint_angles = arm_config + delta_theta

    # Compute new wheel angles
    wheel_speeds = np.array(speed[5:])  # Ensure it's a NumPy array
    delta_wheel = wheel_speeds * timestep
    new_wheel_angles = wheel_config + delta_wheel

    # Construct wheel_matrix
    wheel_matrix = (r_wheel / 4) * np.array([
        [-1 / (l_chassis + w_chassis),  1 / (l_chassis + w_chassis),  1 / (l_chassis + w_chassis), -1 / (l_chassis + w_chassis)],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])

    # Multiply directly; NumPy handles 1D x 2D multiplication
    V_b = wheel_matrix @ delta_wheel

    # Reshape V_b if needed and extract velocities
    z_velocity, x_velocity, y_velocity = V_b.flatten()

    # Compute change in chassis configuration in body frame
    if abs(z_velocity) < 1e-6:  # Treat as zero to avoid division by zero
        change_in_chassis_config_b = np.array([0, x_velocity, y_velocity])
    else:
        change_in_chassis_config_b = np.array([
            z_velocity,
            (x_velocity * np.sin(z_velocity) + y_velocity * (np.cos(z_velocity) - 1)) / z_velocity,
            (y_velocity * np.sin(z_velocity) + x_velocity * (1 - np.cos(z_velocity))) / z_velocity
        ])

    # Transform change in body frame to space frame
    phi = chassis_config[0]  # Current orientation (phi)
    ############ Begin Citation 3 ###########
    rotation_matrix =np.array([[1, 0, 0], 
                     [0, np.cos(phi), -np.sin(phi)], 
                     [0, np.sin(phi), np.cos(phi)]])
    ############ End Citation 3 ###########
    
    delta_q = rotation_matrix @ change_in_chassis_config_b
    new_chassis_config = chassis_config + delta_q

    # Concatenate the new configurations
    next_config = np.concatenate((new_chassis_config, new_joint_angles, new_wheel_angles))

    print("current_config:", current_config)
    print("speed:", speed)
    print("delta_wheel before multiplication:", delta_wheel)
    return next_config
    


# milestone 2: next trajectory generator all in one function for simplicity in main
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




def FeedbackControl(integral_error, current_config, current_pose, reference_pose, next_reference_pose, Kp, Ki, timestep):
    """
    Calculates the kinematic task-space feedforward plus feedback control law.

    Parameters:
    ----------
    integral_error : np.ndarray
        Accumulated integral error for the controller.
    current_config : np.ndarray
        The current robot configuration (chassis + joint angles + wheel angles).
    current_pose : np.ndarray
        The current end-effector pose in the space frame.
    reference_pose : np.ndarray
        The desired end-effector pose at the current time step.
    next_reference_pose : np.ndarray
        The desired end-effector pose at the next time step.
    Kp : np.ndarray
        Proportional gain matrix.
    Ki : np.ndarray
        Integral gain matrix.
    time_step : float
        The time step duration.

    Returns:
    -------
    joint_wheel_speeds : np.ndarray
        The commanded actuator velocities (joint + wheel speeds).
    updated_integral_error : np.ndarray
        The updated integral error.
    task_space_error : np.ndarray
        The task-space error vector.
    """

    # Constants
    wheel_radius = 0.0475
    chassis_length = 0.235
    chassis_width = 0.15
    timestep= 0.01
    dt = 0.01

    # Fixed transformations
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])
    M0e = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0, 1]])

    # Joint screw axes in the end-effector frame at the home position
    Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                      [0, -1, 0, -0.5076, 0, 0],
                      [0, -1, 0, -0.3526, 0, 0],
                      [0, -1, 0, -0.2176, 0, 0],
                      [0, 0, 1, 0, 0, 0]]).T

    # Extract joint angles
    joint_angles = current_config[3:8]

    # Wheel kinematics matrix
    F = wheel_radius/4 * np.array([[0,0,0,0],
                                   [0,0,0,0],
                                   [-1/(chassis_length+chassis_width),
                                    1/(chassis_length+chassis_width),
                                    1/(chassis_length+chassis_width),
                                    -1/(chassis_length+chassis_width)],
                                    [1,1,1,1],
                                    [-1,1,-1,1],
                                    [0,0,0,0]])


    # Compute feedforward twist (V_d)
    feedforward_twist = mr.se3ToVec((1 / dt) * mr.MatrixLog6(np.dot(mr.TransInv(reference_pose), next_reference_pose)))

    # Compute error twist
    task_space_error = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(current_pose), reference_pose)))

    # Update integral error
    integral_error += task_space_error * dt

    # Compute feedback twist
    adjoint_term = mr.Adjoint(np.dot(mr.TransInv(current_pose), reference_pose))
    feedback_twist = np.dot(adjoint_term, feedforward_twist) + np.dot(Kp, task_space_error) + np.dot(Ki, integral_error)

    # Compute the Jacobians
    T0e = mr.FKinBody(M0e, Blist, joint_angles)
    base_jacobian = mr.Adjoint(np.dot(mr.TransInv(T0e), np.linalg.inv(Tb0))).dot(F)
    arm_jacobian = mr.JacobianBody(Blist, joint_angles)
    combined_jacobian = np.hstack((base_jacobian, arm_jacobian))

    # Compute actuator velocities (pseudo-inverse of Jacobian)
    combined_jacobian_inv = np.linalg.pinv(combined_jacobian, 1e-3) #if value is really small it assumes zero 
    joint_wheel_speeds = combined_jacobian_inv.dot(feedback_twist)

    # Debugging prints
    # print("Base Jacobian shape:", base_jacobian.shape)
    # print("Arm Jacobian shape:", arm_jacobian.shape)
    # # print("Combined Jacobian shape:", combined_jacobian.shape)
    # print("Task-space error (Xerr):", task_space_error)
    # print("Commanded actuator speeds:", joint_wheel_speeds)

    print("KP", Kp)
    print("KI", Ki)


    return joint_wheel_speeds, integral_error, task_space_error



def main_loop(Tsc_ini, Tsc_fin, KP, KI, robot_config):
    """
    Implements a position-controlled motion for the wheeled mobile robot.
    """

    X_error = []  # To store the task-space error
    Final_traj_matrix = []
    integral_error = 0.0
    #np.zeros(6, dtype=float)  # Initialize integral error

    # Initial configuration matrices
    #correct
    Tse_ini = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])
  

    Tce_grasp = np.array([[0, 0, 1,0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0],
                      [0, 0, 0, 1]])
    
    Tce_standoff = np.array([[0, 0, 1,0],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0.25], # some small posive value 
                      [0, 0, 0, 1]])
    

    #T_sc_inital = cubes intital configuration
    # this was a given 
    T_sc_initial =  np.array ([[1, 0 , 0 , 1.5],
                         [0, 1, 0, 0.0 ],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

    #T_sc_final = cubes desired final configuration
    # this was a given 
    T_sc_final = np.array([[0, 1, 0, 1.0],
                        [-1, 0, 0, -1],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])



    #correct
    Blist = np.array([[0, 0, 1,   0, 0.033, 0],
                        [0,-1, 0,-0.5076,  0, 0],
                        [0,-1, 0,-0.3526,  0, 0],
                        [0,-1, 0,-0.2176,  0, 0],
                        [0, 0, 1,   0,     0, 0]]).T
    k = 1
    time_step = 0.01

    # Generate trajectory
    traj = np.asarray(TrajectoryGenerator(Tse_ini, T_sc_initial, T_sc_final, Tce_grasp, Tce_standoff, k))
    #save_animation_csv(traj)
    
    Final_traj_matrix.append(robot_config.tolist())  # Append initial configuration

    for i in range(len(traj) - 1):
        # Parse desired configurations
        Xd = np.array([[traj[i][0], traj[i][1], traj[i][2], traj[i][9]],
                       [traj[i][3], traj[i][4], traj[i][5], traj[i][10]],
                       [traj[i][6], traj[i][7], traj[i][8], traj[i][11]],
                       [0, 0, 0, 1]])
        Xd_next = np.array([[traj[i + 1][0], traj[i + 1][1], traj[i + 1][2], traj[i + 1][9]],
                            [traj[i + 1][3], traj[i + 1][4], traj[i + 1][5], traj[i + 1][10]],
                            [traj[i + 1][6], traj[i + 1][7], traj[i + 1][8], traj[i + 1][11]],
                            [0, 0, 0, 1]])

        # Compute current end-effector pose
        thetalist = robot_config[3:8]
        Tsb = np.array([[np.cos(robot_config[0]), -np.sin(robot_config[0]), 0, robot_config[1]],
                        [np.sin(robot_config[0]), np.cos(robot_config[0]), 0, robot_config[2]],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
        Tb0 = np.array([[1, 0, 0, 0.1662],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.0026],
                        [0, 0, 0, 1]])
        M0e = np.array([[1, 0, 0, 0.033],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.6546],
                        [0, 0, 0, 1]])
        T0e = mr.FKinBody(M0e, Blist, thetalist)
        current_pose = np.matmul(Tsb, np.matmul(Tb0, T0e))


        # Call FeedbackControl
        joint_wheel_speeds, integral_error, task_space_error= FeedbackControl(
            integral_error, robot_config, current_pose, Xd, Xd_next, KP, KI, time_step
        )

        X_error.append(task_space_error.tolist())

        # Update robot configuration using NextState
        wheel_speeds = joint_wheel_speeds[:4]
        joint_speeds = joint_wheel_speeds[4:]
        controls = np.concatenate((joint_speeds, wheel_speeds), axis=None)

        robot_config = NextState(robot_config[:12], controls, time_step, 15)
        traj_instant = np.concatenate((robot_config, [traj[i][12]]), axis=None)
        Final_traj_matrix.append(traj_instant.tolist())

        print('robot config',robot_config)

        # if i == 2:
        #     break

    # Plot errors and save trajectory
    plot_error(X_error)
    save_animation_csv(Final_traj_matrix)


def plot_error(X_error):
    '''
    Saves error data to a CSV file and plots the error over time.

    '''
    #save data to csv
    np.savetxt('Xerrnewtask.csv', X_error, delimiter=',')
    # plot the Xerr
    print('plotting')
    qvec = np.asarray(X_error)
    tvec = np.linspace(0,15,qvec.shape[0]) 
    plt.plot(tvec,qvec[:,0])
    plt.plot(tvec,qvec[:,1])
    plt.plot(tvec,qvec[:,2])
    plt.plot(tvec,qvec[:,3])
    plt.plot(tvec,qvec[:,4])
    plt.plot(tvec,qvec[:,5])
    plt.xlim([0,15])
    plt.title(' Xerr plot')
    plt.xlabel('Time (s)')
    plt.ylabel('error')
    plt.legend([r'$Xerr[1]$',r'$Xerr[2]$',r'$Xerr[3]$',r'$Xerr[4]$',r'$Xerr[5]$',r'$Xerr[6]$'])
    plt.grid(True)
    plt.show()
    plt.savefig("Error_plot.png")

def save_animation_csv(final_traj_matrix):
    '''
    Saves the final trajectory data to a CSV file for animation purposes.    '''
    print("Generating animation csv file")
    np.savetxt('simulation_together.csv', final_traj_matrix, delimiter=',')
    print('Done')


def main_loop_call():

    kp = np.eye(6) * 1 #  proportional gain
    ki = np.eye(6) * 0.1  #  integral gain
    # kp =0
    # ki = 0

    Tsc_initial = np.array ([[1, 0 , 0 , 1.5],
                         [0, 1, 0, 0.0, ],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])
    
    Tsc_final = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

    robot_config = np.array([0.0,0.0,0.0,0,0,0.2,-1.6, 0,0,0,0,0,0])

    
    main_loop(Tsc_initial,Tsc_final,kp, ki,robot_config)

# To generate the csv and plots.
main_loop_call()



