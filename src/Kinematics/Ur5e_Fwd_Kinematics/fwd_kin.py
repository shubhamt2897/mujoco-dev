import numpy as np
from types import SimpleNamespace


def forward_kinematics(q,params):
    """
    Forward kinematics for a 3-link planar manipulator.
    
    Args:
        q (np.ndarray): Joint angles (shape: (3,)).
        params (Namespace): Parameters including link lengths and base position.
        
    Returns:
        np.ndarray: End-effector position (shape: (2,)).
    """
    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2]
    

    l1, l2, l3 = params;

    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c2 = np.cos(theta2)             
    s2 = np.sin(theta2)
    c3 = np.cos(theta3)
    s3 = np.sin(theta3)
    
    # Compute transformation matrices between consecutive frames
    # H01: Transformation from frame 1 to frame 0 (base frame rotation by theta1)
    H01 = np.array([
        [c1, -s1, 0, 0],
        [s1, c1,  0,0],
        [0, 0, 1, 0],
        [0, 0, 0, 1 ]])
    
    # H12: Transformation from frame 2 to frame 1 (rotation by theta2 at joint 2)
    H12 = np.array([[c2,-s2,0,0],
                   [s2, c2, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    
    # H23: Transformation from frame 3 to frame 2 (rotation by theta3 at joint 3)
    H23 = np.array([[c3, -s3, 0, 0],
                   [s3, c3,  0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    
    # Compute overall transformation from frame 3 to frame 0 (base frame)
    # H03 = H01 * H12 * H23 gives the position and orientation of frame 3 in base frame
    # Compute overall transformation from frame 3 to frame 0 (base frame)
    # H03 = H01 * H12 * H23 gives the position and orientation of frame 3 in base frame
    H03 = H01 @ H12 @ H23
    
    # Compute end-effector position (e)
    # E3 represents a point at distance l3 along x-axis in frame 3 (end of link 3)
    E3 = np.array([l3,0,0,1])
    # Transform this point to base frame coordinates
    E0 = np.matmul(H03, E3)
    # Extract x,y coordinates of end-effector in base frame
    e= np.array([E0[0], E0[1]])

    # Origin point (base frame reference)
    o= np.array([0,0])
    
    # Compute position of joint 2 (p)
    # Transform origin of frame 2 to base frame using H01*H12
    P0= H01@H12@np.array([0,0,0,1])
    # Extract x,y coordinates of joint 2 position
    p = np.array([P0[0], P0[1]])
    
    # Compute position of joint 3 (q) 
    # Transform origin of frame 3 to base frame using full transformation H03
    Q0 =H03@ np.array([0, 0, 0, 1])
    # Extract x,y coordinates of joint 3 position (should be same calculation as P0?)
    q = np.array([Q0[0], Q0[1]])

    sol = SimpleNamespace(e=e, o=o, p=p, q=q) # Create a SimpleNamespace object to hold results


    return sol # Return as a SimpleNamespace object

    