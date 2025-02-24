import numpy as np
from scipy.optimize import minimize

# Denavit-Hartenberg parameters for the UR5 robot
# [theta, d, a, alpha]
dh_params_ur5 = [
    [0, 0.089159, 0, np.pi/2],     # Joint 1
    [0, 0, -0.425, 0],              # Joint 2
    [0, 0, -0.39225, 0],            # Joint 3
    [0, 0.10915, 0, np.pi/2],       # Joint 4
    [0, 0.09465, 0, -np.pi/2],      # Joint 5
    [0, 0.0823, 0, 0]               # Joint 6
]

dh_params_ur5_w_tool = [
    [0, 0.089159, 0, np.pi/2],     # Joint 1
    [0, 0, -0.425, 0],              # Joint 2
    [0, 0, -0.39225, 0],            # Joint 3
    [0, 0.10915, 0, np.pi/2],       # Joint 4
    [0, 0.09465, 0, -np.pi/2],      # Joint 5
    [0, 0.2573, 0, 0]               # Joint 6 17.5 cm, which is length of tool
]


# Upper bounds for the UR5 robot joints (in radians)
bound_u = np.array([2.8973, 1.7628, 2.8973, 3.1416, 2.8973, 2.8973])

# Lower bounds for the UR5 robot joints (in radians)
bound_l = np.array([-2.8973, -1.7628, -2.8973, -3.1416, -2.8973, -2.8973])


def isInBound(x,jid):
    if x<bound_l[jid]:
        return false
    if x>bound_u[jid]:
        return false
    return true

def rot_m_from_qu(q):
    w=q.w
    x=q.x
    y=q.y
    z=q.z
    rotation = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    return rotation

def dh_transform_matrix(theta, d, a, alpha):
    """Compute the transformation matrix for a single joint using DH parameters."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_helper(joint_angles,dh_params):
    """Compute the forward kinematics (end effector position and rotation) for the UR5 robot."""
    # Initialize transformation matrix (identity matrix)
    T = np.eye(4)

    # Iterate through all joints and apply the DH transformation matrix
    for i in range(6):
        theta, d, a, alpha = dh_params[i]
        theta += joint_angles[i]  # Add joint angle to theta
        T_i = dh_transform_matrix(theta, d, a, alpha)
        T = np.dot(T, T_i)  # Multiply the current transformation matrix by the new one

    return T


def forward_kinematics(joint_angles,dh_params):
    T=forward_kinematics_helper(joint_angles,dh_params)

    # End effector position (x, y, z) is the translation part of the final transformation matrix
    position = T[:3, 3]

    # End effector rotation (3x3 matrix) is the rotation part of the final transformation matrix
    rotation = T[:3, :3]

    return position, rotation



def ik_objective(joint_angles, dh_params, target_position,target_rotation):
    """Objective function to minimize: error between target and calculated position."""
    current_position,current_rotation = forward_kinematics(joint_angles,dh_params)
    error_m=np.transpose(target_rotation)*current_rotation
    error_rot=(np.arccos((np.trace(error_m) - 1) / 2)/(2*3.141596))
    error_translation = np.linalg.norm(current_position - target_position)
    #error=error_translation
    s=.55
    error=s*error_translation+(1-s)*error_rot
    #print("error",error_translation,error_rot)
   
    return error

# Solve Inverse Kinematics Using a Numerical Solver (Minimization)
def inverse_kinematics(target_position, target_rotation,initial_guess,dh_params):
    """Solve the inverse kinematics problem for a given target position."""
    result = minimize(ik_objective, initial_guess, args=(dh_params,target_position,target_rotation,), bounds=[(-np.pi, np.pi)] * 6)
    return result.x  # Return the joint angles that minimize the error

#defuct
def testEEPosTransform():
    cfg=[-1.420226,-0.561880,0.837741,-1.528992,-0.919493,-0.068503,0.489961,0.286832,-0.508009,-0.303644,-0.225533,0.047273,.05]
    object_pos=[-.08,-.13,.31]
    eePos,rot=forward_kinematics(cfg,dh_params_ur5)  #make first param height of gripper
    
    print(eePos)
    cfg2=inverse_kinematics(eePos+object_pos,rot,[0,0,0,0,0,0],dh_params_ur5)
    print(cfg2*360/(2*3.141596))
    eePos2=forward_kinematics(cfg2,dh_params_ur5)
    print(eePos2) 
