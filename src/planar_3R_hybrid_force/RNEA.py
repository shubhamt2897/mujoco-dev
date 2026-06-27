import numpy as np
import utility as ram
from robot_data import robot
from forward_kinematics import forward_kinematics

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

def forward_backward_recursion(q,qdot,qddot,robot,g):

    _,robot = forward_kinematics(q)

    for i in range(1,len(robot.body)+1):
        I = robot.body[i].inertia;
        robot.body[i].I_body = np.array([
                                [I[0], I[3], I[4]],
                                [I[3], I[1], I[5]],
                                [I[4], I[5], I[2]]
                             ])
        #iquat = robot.body[i].iquat
        #R_inertia = ram.quat2rotation(iquat);
        #robot.body[i].I_body = R_inertia@np.diag(robot.body[i].inertia)@R_inertia.T

    #velocities
    w00 = np.array([ 0, 0, 0])
    v00 = np.array([ 0, 0, 0])
    wdot00 = np.array([ 0, 0, 0])
    vdot00 = np.array([ 0, 0, 0])

    for i in range(1,len(robot.body)+1):
        joint_axis = robot.body[i].joint_axis;
        ipos = robot.body[i].ipos;
        o = robot.body[i].o_local;
        R = robot.body[i].R_local.T

        robot.body[i].w = R@w00 + qdot[i-1]*joint_axis;
        robot.body[i].v = R@(v00 + ram.vec2skew(w00)@o);
        robot.body[i].wdot = R@wdot00 + ram.vec2skew( (R@w00))@(qdot[i-1]*joint_axis) + qddot[i-1]*joint_axis;
        robot.body[i].vdot = R@(vdot00 + ram.vec2skew(wdot00)@o + ram.vec2skew(w00)@(ram.vec2skew(w00)@o));
        robot.body[i].vdotC = robot.body[i].vdot + ram.vec2skew(robot.body[i].wdot)@ipos + ram.vec2skew(robot.body[i].w)@(ram.vec2skew(robot.body[i].w)@ipos);

        w00 = robot.body[i].w;
        v00 = robot.body[i].v;
        wdot00 = robot.body[i].wdot;
        vdot00 = robot.body[i].vdot;
        vdotC = robot.body[i].vdotC;


        #print(i)
        #print_matrix(np.array(robot[i].w),4,"w")
        #print_matrix(np.array(robot[i].wdot),4,"wdot")
        #print_matrix(np.array(robot[i].vdot),4,"vdot")
        #print_matrix(np.array(robot[i].vdotC),4,"vdotC")

    fn = np.array([ 0, 0, 0])
    nn = np.array([ 0, 0, 0])
    Rn = np.eye(3);
    o = robot.params.end_eff_pos_local;

    grav = np.array([0,g,0])

    #for i, body in reversed(sorted(robot.body.items())):
    for i in range(len(robot.body), 0, -1):
        m = robot.body[i].mass
        I = robot.body[i].I_body
        w = robot.body[i].w;
        ipos = robot.body[i].ipos;

        R_world = robot.body[i].H_global[0:3,0:3]
        g_body = R_world.T@grav
        F = m*robot.body[i].vdotC

        # tmp = I@robot.body[i].w
        # print(tmp)
        # print(np.array(tmp).reshape(-1))
        # print(np.shape(tmp))
        # print(np.shape(np.squeeze(tmp)))
        #print(np.shape(ram.vec2skew(w)))
        #hi
        # print(np.shape(I))
        # print(type(I))
        # print(np.shape(robot.body[i].wdot))
        #
        N = I@robot.body[i].wdot + ram.vec2skew(w)@(I@robot.body[i].w)
        robot.body[i].f = Rn@fn + F - m*g_body;
        robot.body[i].n = N+Rn@nn+ram.vec2skew(ipos)@robot.body[i].f+ram.vec2skew( (o-ipos))@(Rn@fn)

        fn = robot.body[i].f;
        nn = robot.body[i].n;
        Rn = robot.body[i].R_local;
        o = robot.body[i].o_local;

        # print('f and F');
        #print(i)
        #print(robot[i].f)
        # print(F)
        # print('n and N')
        #print(robot[i].n)
        #print('******')
        # print(N)


    #tau = np.zeros(len(robot.body.items()))
    tau = np.zeros(len(robot.body))
    count = 0
    for i in range(1,len(robot.body)+1):
        joint_axis = robot.body[i].joint_axis;
        j = np.dot(joint_axis,np.array([1,2,3]))
        tau[count] = robot.body[i].n[j-1]
        count +=1

    #print(tau)

    return tau

def RNEA(q,qdot,flag=1):
    nv = len(qdot);
    g = robot.params.gravity[1]

    #Compute C
    qddot = np.zeros(nv)  # Initialize qdd as a 1D array with zeros
    tau_C = forward_backward_recursion(q,qdot,qddot,robot,g=0)
    C = np.zeros((nv, 1))
    C[:,0] = tau_C

    # Compute CG
    qddot = np.zeros(nv)  # Initialize qdd as a 1D array with zeros
    tau_CG = forward_backward_recursion(q,qdot,qddot,robot,g)
    CG = np.zeros((nv, 1))
    CG[:,0] = tau_CG

    #Compute G
    G = np.zeros((nv, 1))
    G[:,0] = CG[:,0]-C[:,0]

    # Initialize the mass matrix M
    M = np.zeros((nv, nv))  # M is a 2D array with dimensions nv x nv

    if (flag==1):
        # Loop to compute columns of M
        for i in range(nv):
            qddot[i] = 1  # Set one element of qdd to 1
            tau = forward_backward_recursion(q,qdot,qddot,robot,g)  # Call RNEA with the updated qdd
            M[:, i] = tau - tau_CG  # Compute the i-th column of M
            qddot[i] = 0  # Reset qdd to zeros

    C = C[:,0]
    G = G[:,0]
    CG = CG[:,0]

    return C,G,CG,M

#testing
# q = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
# qdot = np.array([0.21,-0.38,0.41,-4.12,6.32,8.53])
# #qddot = np.array([0,0,0,0,0,0])
# M,CG = RNEA(q,qdot)
#
# print_matrix(M,4,"M_py")
# print_matrix(CG,4,"CG_py")
