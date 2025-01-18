
# # body jacobian review in chapter 5
# # motion control with velocity inputs chapter 11.3
# # mobile manipulation chapter 13.5

# #goal is to move from current position and oritentation to desired posiition usign contral commands 
# # to do this we need to use feedforward and feedback control 

import modern_robotics as mr
import numpy as np

def FeedbackControl(Tse, Tsed, Tsed_next, Kp, Ki, dt, integralError):
    """ Feedback control for task-space control. """
    # Compute error between current and desired pose
    Xerr_se3 = mr.MatrixLog6(np.dot(mr.TransInv(Tse), Tsed))
    Xerr = mr.se3ToVec(Xerr_se3)

    # Update the integral error
    integralError += Xerr * dt

    # Compute feedforward twist
    Vd_se3 = mr.MatrixLog6(np.dot(mr.TransInv(Tsed), Tsed_next))
    Vd = (1 / dt) * mr.se3ToVec(Vd_se3)

    # Compute feedback twist
    adjoint_term = mr.Adjoint(np.dot(mr.TransInv(Tse), Tsed))
    V = np.dot(adjoint_term, Vd) + np.dot(Kp, Xerr) + np.dot(Ki, integralError)

    return V, integralError, Xerr

# Test Inputs
phi, x, y = 0, 0, 0
theta = [0, 0, 0.2, -1.6, 0]
joint_angles = np.array(theta)

# Transformation Matrices
Tse = np.array([[0.170, 0, 0.985, 0.387],
                [0, 1, 0, 0],
                [-0.985, 0, 0.170, 0.570],
                [0, 0, 0, 1]])
Tsed = np.array([[0, 0, 1, 0.5],
                 [0, 1, 0, 0],
                 [-1, 0, 0, 0.5],
                 [0, 0, 0, 1]])
Tsed_next = np.array([[0, 0, 1, 0.6],
                      [0, 1, 0, 0],
                      [-1, 0, 0, 0.3],
                      [0, 0, 0, 1]])

# Gains
Kp = np.zeros((6, 6))
Ki = np.zeros((6, 6))
dt = 0.01
integralError = np.zeros(6)

# Compute Feedback Control
V, updated_integralError, Xerr = FeedbackControl(Tse, Tsed, Tsed_next, Kp, Ki, dt, integralError)
print("Vd:", V)  # Should match expected Vd
print("Xerr:", Xerr)
