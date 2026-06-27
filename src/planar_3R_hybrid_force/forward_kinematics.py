import numpy as np
from robot_data import robot
from types import SimpleNamespace
import utility as ram

def forward_kinematics(q):

    for i in range(1,len(robot.body)+1):
        joint_axis = robot.body[i].joint_axis
        axis_id = np.argmax(np.abs(joint_axis))
        angle = q[i-1]  # body_id goes from 1 to 6 but q goes from 0 to 5.

        R_q = ram.rotation(angle, axis_id)
        robot.body[i].R_local= R_q
        robot.body[i].o_local = robot.body[i].pos

        robot.body[i].H_local = np.block([
            [robot.body[i].R_local, robot.body[i].o_local.reshape(-1, 1)], #does 3x1
            [np.zeros((1, 3)), 1]
            ])

    temp = np.identity(4)
    for i in range(1,len(robot.body)+1):
        # Compute global transformation matrix
        robot.body[i].H_global = temp @ robot.body[i].H_local
        # Update temp for the next iteration
        temp = robot.body[i].H_global

    #this part need to be updated on case by case basis
    o = robot.body[1].H_global[:3,3]
    p = robot.body[2].H_global[:3,3]
    q = robot.body[3].H_global[:3,3]
    end_eff_pos_local = robot.params.end_eff_pos_local
    e = robot.body[3].H_global @ np.append(end_eff_pos_local, 1)
    e = e[:3]

    w = robot.params.w;
    h = robot.params.h;
    l3 = robot.params.end_eff_pos_local[0]

    H03 = robot.body[3].H_global;
    E_left1 = np.matmul(H03,np.array([l3, -w, 0, 1]))
    E_right1 = np.matmul(H03,np.array([l3, w, 0, 1]))
    E_left2 = np.matmul(H03,np.array([l3+h, -w, 0, 1]))
    E_right2 = np.matmul(H03,np.array([l3+h, w, 0, 1]))

    e_left1 = E_left1[0:2];
    e_right1 = E_right1[0:2];
    e_left2 = E_left2[0:2];
    e_right2 = E_right2[0:2];

    theta = np.arccos(H03[0,0])

    # Extract the rotation matrix from the global transformation matrix and R_end_eff
    end_eff_rot = H03[:3, :3]



    # Store in a SimpleNamespace
    sol = SimpleNamespace(
        o=np.array(o),
        p=np.array(p),
        q=np.array(q),
        e=np.array(e),
        e_left1=np.array(e_left1),
        e_right1 = np.array(e_right1),
        e_left2=np.array(e_left2),
        e_right2 = np.array(e_right2),
        theta = theta,
        end_eff_pos = e,
        end_eff_rot = end_eff_rot,
        )

    return sol,robot
