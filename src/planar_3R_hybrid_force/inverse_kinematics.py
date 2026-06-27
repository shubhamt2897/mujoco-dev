from forward_kinematics import forward_kinematics

def inverse_kinematics(q,parms):

    x_ref = parms[0]
    y_ref = parms[1]
    theta_ref = parms[2]

    sol,_ = forward_kinematics(q)
    e = sol.e
    theta = sol.theta
    x = e[0]
    y = e[1]
    return x-x_ref,y-y_ref,theta-theta_ref
