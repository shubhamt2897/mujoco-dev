import numpy as np
from forward_kinematics import forward_kinematics
import utility as ram
#
# def print_matrix(Mat, decimals=6, name="Mat"):
#     """
#     Prints an m x n matrix in Octave/Matlab style with configurable decimal accuracy and an optional name.
#
#     Args:
#         Mat (numpy.ndarray): The matrix to display.
#         decimals (int): The number of decimal places to display.
#         name (str): The name of the matrix to display. Defaults to "Mat".
#     """
#     if not isinstance(Mat, np.ndarray):
#         raise ValueError("Input must be a numpy.ndarray.")
#
#     if Mat.ndim != 2:
#         raise ValueError("Input must be a 2D numpy.ndarray (matrix).")
#
#     format_string = f"{{:.{decimals}f}}"  # Dynamic formatting for specified decimals
#     print(f"{name} = [ ...")
#     for row in Mat:
#         formatted_row = " ".join(format_string.format(elem) for elem in row)
#         print(f"  {formatted_row};")
#     print("];\n")



def jac_com(q):
    _,robot = forward_kinematics(q)

    #frame origin
    o01 = robot.body[1].H_global[0:3,3]
    o02 = robot.body[2].H_global[0:3,3]
    o03 = robot.body[3].H_global[0:3,3]

    #com
    g1 = (robot.body[1].H_global @ np.append(robot.body[1].ipos, 1))[:3]
    g2 = (robot.body[2].H_global @ np.append(robot.body[2].ipos, 1))[:3]
    g3 = (robot.body[3].H_global @ np.append(robot.body[3].ipos, 1))[:3]

    #print(g1)
    #joint axis
    n1 = robot.body[1].joint_axis
    n2 = robot.body[2].joint_axis
    n3 = robot.body[3].joint_axis

    #rotation matrix
    R00 = np.eye(3);
    R01 = robot.body[1].H_global[0:3,0:3]
    R02 = robot.body[2].H_global[0:3,0:3]
    R03 = robot.body[3].H_global[0:3,0:3]

    #jacobian
    Jv_g3 = np.column_stack([
                     ram.vec2skew(R00 @ n1) @ (g3-o01), \
                     ram.vec2skew(R01 @ n2) @ (g3-o02), \
                     ram.vec2skew(R02 @ n3) @ (g3-o03),
                     ])

    Jw_g3 = np.column_stack([
                           R00 @ n1,\
                           R01 @ n2,\
                           R02 @ n3,\
                           ])

    J_g3 = np.vstack([Jv_g3, Jw_g3])

    #print_matrix(J_g3,4,"J_g3")

    #jacobian
    Jv_g2 = np.column_stack([
                     ram.vec2skew(R00 @ n1) @ (g2-o01), \
                     ram.vec2skew(R01 @ n2) @ (g2-o02), \
                     np.zeros((3,1)),
                     ])

    Jw_g2 = np.column_stack([
                           R00 @ n1,\
                           R01 @ n2,\
                           np.zeros((3,1)),
                           ])

    J_g2 = np.vstack([Jv_g2, Jw_g2])

    # print_matrix(J_g2,4,"J_g2")

    #jacobian
    Jv_g1 = np.column_stack([
                     ram.vec2skew(R00 @ n1) @ (g1-o01), \
                     np.zeros((3,1)),
                     np.zeros((3,1))
                     ])

    Jw_g1 = np.column_stack([
                           R00 @ n1,\
                           np.zeros((3,1)),
                           np.zeros((3,1)),
                           ])

    J_g1 = np.vstack([Jv_g1, Jw_g1])

    #print_matrix(J_g1,4,"J_g1")


    return robot,J_g1,J_g2,J_g3

#angles
#q = np.array([0.1, -1.1, 0.3, -0.21, -0.35, 0.2]);
#q = np.array([0,0,0,0,0,0])
#q = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
#jac_torques(q,robot)
