import numpy as np
from jac_com import jac_com
import utility as ram
from scipy.linalg import block_diag

def print_matrix(Mat, decimals=6, name="Mat"):
    """
    Prints an m x n matrix in Octave/Matlab style with configurable decimal accuracy and an optional name.

    Args:
        Mat (numpy.ndarray): The matrix to display.
        decimals (int): The number of decimal places to display.
        name (str): The name of the matrix to display. Defaults to "Mat".
    """
    if not isinstance(Mat, np.ndarray):
        raise ValueError("Input must be a numpy.ndarray.")

    if Mat.ndim != 2:
        raise ValueError("Input must be a 2D numpy.ndarray (matrix).")

    format_string = f"{{:.{decimals}f}}"  # Dynamic formatting for specified decimals
    print(f"{name} = [ ...")
    for row in Mat:
        formatted_row = " ".join(format_string.format(elem) for elem in row)
        print(f"  {formatted_row};")
    print("];\n")


def TMT(q):

    robot,J_g1,J_g2,J_g3 = jac_com(q)

    # Initialize the overall matrix M
    M_ = np.zeros((0, 0))  # Start with an empty ndarray

    j = 0;
    for i in range(1,len(robot.body)+1):
        #quat = robot.body[i].quat
        #iquat = robot.body[i].iquat
        #joint_axis = robot.body[i].joint_axis

        m = robot.body[i].mass
        mass = m * np.eye(3)  # Create a 3x3 mass matrix as an ndarray

        #axis_id = np.argmax(np.abs(joint_axis))
        #angle = q[i-1]  # body_id goes from 1 to 6 but q goes from 0 to 5.
        #R_q = ram.rotation(angle, axis_id)
        #robot.body[i].R_local= ram.quat2rotation(quat) @ R_q
        R_local = robot.body[i].R_local

        #R_inertia = ram.quat2rotation(iquat);
        #robot.body[i].I_body = R_inertia@np.matrix(np.diag(robot.body[i].inertia))@R_inertia.T
        I = robot.body[i].inertia;
        robot.body[i].I_body = np.array([
                                [I[0], I[3], I[4]],
                                [I[3], I[1], I[5]],
                                [I[4], I[5], I[2]]
                             ])
        inertia = R_local @ robot.body[i].I_body @R_local.T  # Inertia is assumed to be a 3x3 ndarray
        #inertia = R_local.T @ robot[i].I_body @R_local  # Inertia is assumed to be a 3x3 ndarray

        # Create M_local by combining mass and inertia
        M_local = block_diag(mass, inertia)

        # Append M_local to M
        if M_.size == 0:  # If M_ is empty, initialize it directly
            M_ = M_local
        else:
            M_ = block_diag(M_, M_local)


    J = np.vstack([J_g1, J_g2, J_g3])


    # Compute the TMT matrix
    M_TMT = J.T @ M_ @ J  # Use @ for matrix multiplication

    # Ensure M_TMT is a standard ndarray
    M_TMT = np.asarray(M_TMT)

    return M_TMT
