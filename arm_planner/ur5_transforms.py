import numpy as np
from scipy.optimize import minimize

def dh_transform(theta, d, a, alpha):
    """Generate the DH transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(joint_angles,tool_offset=[0,0]):
    """Compute the end-effector position of the UR5 robot from joint angles."""
    #print(joint_angles)
    #print(np.pi)
    # DH Parameters for UR5 (in mm and radians)
    #dh_params = [
    #    [joint_angles[0], 0, 0, np.pi / 2],    # Joint 1
    #    [joint_angles[1], 0, .425, 0],           # Joint 2
    #    [joint_angles[2], 0, .392, 0],           # Joint 3
    #    [joint_angles[3], .130, 0, np.pi / 2],   # Joint 4
    #    [joint_angles[4], 0, 0, -np.pi / 2],    # Joint 5
    #    [joint_angles[5], 0, 0, 0]              # Joint 6
    #]


    dh_params = [
        [joint_angles[0], 0, 0, np.pi *(1/2)],    # Joint 1
        [joint_angles[1], 0, .425, 0],           # Joint 2
        [joint_angles[2], 0, .392, 0],           # Joint 3
        [joint_angles[3], .109, 0, np.pi / 2],   # Joint 4
        [joint_angles[4], .095, 0, -np.pi / 2],    # Joint 5
        [joint_angles[5], .082+tool_offset[0], tool_offset[1], 0]              # Joint 6  #need to add height of gripper to this
    ]
    
    # Initialize the transformation matrix as the identity matrix
    T = np.eye(4)
    
    # Multiply transformation matrices for each joint
    for params in dh_params:
        T_i = dh_transform(params[0], params[1], params[2], params[3])
        T = np.dot(T, T_i)  # Multiply the transformation matrices
    
    # The position of the end effector is the first three elements of the last column
    end_effector_position = T[0:3, 3]
    
    return end_effector_position


def getCfgForEEPos(ee_pos):
    print("write this")


def ik_objective(joint_angles, target_position):
    """Objective function to minimize: error between target and calculated position."""
    current_position = forward_kinematics(joint_angles,[0,0])  #to do, measure offset of camera (set second param to) and make this a paremeter of function
    error = np.linalg.norm(current_position - target_position)
    return error

# Solve Inverse Kinematics Using a Numerical Solver (Minimization)
def inverse_kinematics(target_position, initial_guess):
    """Solve the inverse kinematics problem for a given target position."""
    result = minimize(ik_objective, initial_guess, args=(target_position,), bounds=[(-np.pi, np.pi)] * 6)
    return result.x  # Return the joint angles that minimize the error

def testEEPosTransform():
    cfg=[-1.420226,-0.561880,0.837741,-1.528992,-0.919493,-0.068503,0.489961,0.286832,-0.508009,-0.303644,-0.225533,0.047273,.05]
    object_pos=[-.08,-.13,.31]
    eePos=forward_kinematics(cfg,[0,.185])  #make first param height of gripper
    
    print(eePos)
    cfg2=inverse_kinematics(eePos+object_pos,[0,0,0,0,0,0])
    print(cfg2*360/(2*3.141596))
    eePos2=forward_kinematics(cfg2)
    print(eePos2) 
