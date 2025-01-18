import numpy as np
from numpy import sin, cos

def NextState(current_config, speed, timestep, max_omg):
    """
    Computes the next state of the robot given the current configuration, speed, timestep, and max angular velocity.
    """
    # Validate speed length
    if len(speed) != 9:
        raise ValueError("Speed vector must have 9 elements (5 arm joint speeds + 4 wheel speeds).")

    # Parse current configuration
    chassis_config = np.array(current_config[:3])  # Chassis configuration [x, y, phi]
    arm_config = np.array(current_config[3:8])  # Arm joint angles
    wheel_config = np.array(current_config[8:])  # Wheel angles

    # Limit speed to max angular velocity
    speed = np.clip(speed, -max_omg, max_omg)

    # Compute new joint angles
    joint_speeds = speed[:5]
    delta_theta = joint_speeds * timestep
    new_joint_angles = arm_config + delta_theta

    # Compute new wheel angles
    wheel_speeds = speed[5:]
    delta_wheel = wheel_speeds * timestep
    new_wheel_angles = wheel_config + delta_wheel

    # Constants
    r_wheel = 0.0475  # Radius of the wheel
    l_chassis = 0.235  # Half the length of the chassis
    w_chassis = 0.15  # Half the width of the chassis

    # Compute chassis velocities (body twist) from wheel motion
    wheel_matrix = (r_wheel / 4) * np.array([
        [-1 / (l_chassis + w_chassis),  1 / (l_chassis + w_chassis),  1 / (l_chassis + w_chassis), -1 / (l_chassis + w_chassis)],  # Angular velocity
        [1, 1, 1, 1],  # Velocity along x-axis
        [-1, 1, -1, 1]  # Velocity along y-axis
    ])
    V_b = wheel_matrix @ delta_wheel  # Body twist [angular_velocity_z, linear_velocity_x, linear_velocity_y]

    z_velocity, x_velocity, y_velocity = V_b

    # Compute chassis displacement
    if z_velocity == 0:
        change_in_chassis_congig_b = np.array([0, x_velocity, y_velocity])  # Straight-line motion
    else:  # Rotational motion
        change_in_chassis_congig_b = np.array([
            z_velocity,
            (x_velocity * sin(z_velocity) + y_velocity * (cos(z_velocity) - 1)) / z_velocity,
            (y_velocity * sin(z_velocity) + x_velocity * (1 - cos(z_velocity))) / z_velocity
        ])

    # Transform displacement from body frame to world frame
    phi = chassis_config[2]  # Current orientation
    rotation_matrix = np.array([
        [cos(phi), -sin(phi), 0],
        [sin(phi), cos(phi), 0],
        [0, 0, 1]
    ])
    delta_q = rotation_matrix @ change_in_chassis_congig_b
    new_chassis_config = chassis_config + delta_q

    # Combine updated components into the next configuration
    next_config = np.concatenate((new_chassis_config, new_joint_angles, new_wheel_angles))
    return next_config



# def NextState(current_config, speed, timestep, max_omg):
#     """
#     Args:
#         current_config (array): A 12-vector representing the current configuration of the robot:
#             - 3 variables for the chassis configuration (x, y, phi),
#             - 5 variables for the arm configuration (joint angles),
#             - 4 variables for the wheel angles.
#         speed (array): A 9-vector indicating arm joint speeds (5 values) and wheel speeds (4 values).
#         timestep (float): The time step duration.
#         max_omg (float): Maximum angular speed for the arm joints and wheels.

#     Returns:
#         np.ndarray: A 12-vector representing the robot configuration at the next timestep.
#     """

#     # Parse current configuration
#     chassis_config = np.array(current_config[:3]) #chassis configuration (x y phi)
#     arm_config = np.array(current_config[3:8]) #arm joint angles
#     wheel_config = np.array(current_config[8:]) #wheel angles

#     # set speed limit at +- max omg
#     speed = np.clip(speed, -max_omg, max_omg) 

#     # Compute new joint angles
#     joint_speeds = speed[:5]
#     delta_theta = joint_speeds * timestep
#     new_joint_angles = arm_config + delta_theta

#     # Compute new wheel angles
#     wheel_speeds = speed[5:]
#     delta_wheel = wheel_speeds * timestep
#     new_wheel_angles = wheel_config + delta_wheel


#     # Constants
#     r_wheel = 0.0475  # Radius of the wheel
#     l_chassis = 0.235   # Half the length of the chassis
#     w_chassis = 0.15    # Half the width of the chassis


#     # Compute chassis velocoities (body twist) from wheel motion
#     wheel_matrix = (r_wheel / 4) * np.array([
#         [-1 / (l_chassis + w_chassis),  1 / (l_chassis + w_chassis),  1 / (l_chassis + w_chassis), -1 / (l_chassis + w_chassis)], #ang velocity --> velocity around z axis
#         [1, 1, 1, 1], # x axis velocity
#         [-1, 1, -1, 1] # y axis velocity
#     ])
#     #bodu twist for angular and x and y velocity
#     V_b = wheel_matrix @ delta_wheel  # Body twist angulkar_velcoity_z, linear_velocity_x, linear_velocity_y 

#     z_velocity, x_velocity, y_velocity = V_b

#     # Compute chassis displacement
#     if z_velocity == 0:
#         change_in_chassis_congig_b = np.array([0, x_velocity, y_velocity]) #straight line no rotation
#     else: #rotation
#         change_in_chassis_congig_b = np.array([
#             z_velocity,
#             (x_velocity * sin(z_velocity) + y_velocity * (cos(z_velocity) - 1)) / z_velocity,
#             (y_velocity * sin(z_velocity) + x_velocity * (1 - cos(z_velocity))) / z_velocity
#         ])

#     # Transform the displacement from body frame to world frame
#     phi = chassis_config[2] #chassis condfig in world frame

#     # Define the rotation matrix
#     rotation_matrix = np.array([
#         [cos(phi), -sin(phi), 0],  # 2D rotation around the z-axis
#         [sin(phi), cos(phi), 0],
#         [0, 0, 1]  # Identity for 3D transformations, no change in the z-component
#     ])

#     # Transform the displacement from body frame to world frame
#     delta_q = rotation_matrix @ change_in_chassis_congig_b

#     # Add the displacement to the current configuration to get the new chassis configuration
#     new_chassis_config = chassis_config[:3] + delta_q  # Update only the position part of the chassis configuration

#     # all components added to the next configuration
#     next_config = np.concatenate((new_chassis_config, new_joint_angles, new_wheel_angles))    
#     return next_config


# Test input
current_config = np.zeros(12) #init robot config to all zeros
speed = np.array([0, 0, 0, 0, 0, 10, 10, 10, 10])  # Wheels move forward
# speed = np.array([0, 0, 0, 0, 0, -10, 10, -10, 10])  # Wheels move sideways in pos yb direction 
# speed = np.array([0, 0, 0, 0, 0, -10, 10, 10, -10])  # should spin counter clockwise by 1.234 radians (70 degeres)

timestep = 0.01
max_omg = 15 #max speed

# Compute next configuration
next_config = NextState(current_config, speed, timestep, max_omg)

# simulate robot motion for 100 timesteps
trajectory = []
trajectory.append(np.concatenate((current_config, [0])))  # Add placeholder for gripper state

for _ in range(100):
    next_config = NextState(current_config, speed, timestep, max_omg)
    current_config = next_config
    trajectory.append(np.concatenate((current_config, [0])))

# Save trajectory to CSV
np.savetxt('Next_config.csv', trajectory, delimiter=',')

print("Simulation complete. Configuration saved to 'Next_config.csv'.")

###################################################


