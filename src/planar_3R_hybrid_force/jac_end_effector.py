import numpy as np
import utility as ram
from forward_kinematics import forward_kinematics


#angles
#q = np.array([0.1, -1.1, 0.3, -0.21, -0.35, 0.2]);
#q = np.array([0,0,0,0,0,0])
#q = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])

def jac_end_effector(q):
    sol,robot = forward_kinematics(q)

    #get the output
    end_eff_pos = sol.end_eff_pos
    end_eff_rot = sol.end_eff_rot

    #end-effector position
    e0 = end_eff_pos

    #frame origin
    o01 = robot.body[1].H_global[0:3,3]
    o02 = robot.body[2].H_global[0:3,3]
    o03 = robot.body[3].H_global[0:3,3]

    #joint axis
    n1 = robot.body[1].joint_axis
    n2 = robot.body[2].joint_axis
    n3 = robot.body[3].joint_axis


    #rotation matrix
    R00 = np.eye(3);
    R01 = robot.body[1].H_global[0:3,0:3]
    R02 = robot.body[2].H_global[0:3,0:3]
    R03 = robot.body[3].H_global[0:3,0:3]

    #jacobians
    Jv_E = np.column_stack([
                     ram.vec2skew(R00 @ n1) @ (e0-o01), \
                     ram.vec2skew(R01 @ n2) @ (e0-o02), \
                     ram.vec2skew(R02 @ n3) @ (e0-o03),
                     ])

    Jw_E = np.column_stack([
                        R00 @ n1,\
                        R01 @ n2,\
                        R02 @ n3,
                        ])

    J_E = np.vstack([Jv_E, Jw_E])

    return J_E
