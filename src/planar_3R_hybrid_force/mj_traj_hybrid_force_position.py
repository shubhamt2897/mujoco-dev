import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from forward_kinematics import forward_kinematics
from inverse_kinematics import inverse_kinematics
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

from jac_end_effector import jac_end_effector
from TMT import TMT
from RNEA import RNEA

flag_traj_generation = 0;

xml_path = 'planar_3R.xml' #xml file (assumes this is in the same folder as this file)
simend = 1 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

#straight_line parameters
r = 0.3; f = 0.5; n = 2
t_init = 1; #time to adjust
simend = t_init + (n/f)+0.05

def straight_line(t,x0,y0,theta0,r,f):

    w = 2*np.pi*f;
    x = x0+r*np.sin(w*t)
    y = y0;
    pos = [x,y,theta0]

    xd = r*w*np.cos(w*t)
    yd = 0
    thetad = 0
    vel = [xd,yd,thetad]

    xdd = -r*w*w*np.sin(w*t)
    ydd = 0
    thetadd = 0
    acc = [xdd,ydd,thetadd]

    return pos,vel,acc


def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

C_py = np.zeros(3)
G_py = np.zeros(3)
CG_py = np.zeros(3)
M_py_TMT = np.identity(3)
n_update = 5; #times the G,C,CG should be updated
count = 0;

# kp = 1000*np.array([1,1,0.2])
# kv = 0.1*kp.copy()#np.array([0.1*kp[0],0.1*kp[1],0.1*kp[2]])
# ki = 2*np.array([1,1,1])
# e_sum = np.zeros(3);


def controller(model, data):
    #put the controller here. This function is called inside the simulation.


    global C_py, G_py, CG_py
    global M_py_TMT
    global count
    global e_sum

    count += 1

    if ( count % n_update  == 0):
        C_py,G_py,CG_py,_ = RNEA(q,u,0)

    if ( count % (n_update-1)  == 0): #the -1 is to prevent the update at the same time as RNEA
        M_py_TMT = TMT(q)


    tau_ff = M_py_TMT@ud + CG_py

    dx = pos_ref-pos
    dv = vel_ref-vel

    kp2 = 1000*np.array([1,1,0.2])
    kv2 = 0.1*kp2.copy()#np.array([0.1*kp[0],0.1*kp[1],0.1*kp[2]])

    Fx = kp2[0]*dx[0]+kv2[0]*dv[0];
    #Fy = kp2[1]*dx[1]+kv2[1]*dv[1];
    Fy = 20; #regulate force
    Mz = kp2[2]*dx[2]+kv2[2]*dv[2];

    Force = np.array([Fx,Fy,Mz])
    tau_fb = jac_E.T@Force


    #1) Feedback control
    #ctrl = tau_fb.copy()

    #2) Feedforward+Feedback
    ctrl = tau_ff.copy() + tau_fb.copy()

    data.ctrl = ctrl.copy()

    #pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path

dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = 89.8231032550546 ; cam.elevation = -88.84857804659835 ; cam.distance =  6.025299868401033
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

key_id = model.key("home").id
key_qpos = model.key_qpos[key_id]  #Access qpos
key_ctrl = model.key_ctrl[key_id] #Access ctrl

# Find the geom ID of the geom inside the body "surface"
geom_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, 'surface')
#geom_id = model.geom("surface").id
# Now get its size
geom_size = model.geom_size[geom_id]

x0 = model.body("surface").pos[0]
y0 = model.body("surface").pos[1]-geom_size[1]
theta0 = np.pi/2

#reference position
q = key_qpos.copy()
x_ref = x0; y_ref = y0; theta_ref = theta0;
parms = [x_ref, y_ref, theta_ref]
pos_ref = np.array([x_ref, y_ref, theta_ref])
vel_ref = np.array([0,0,0])
acc_ref = np.array([0,0,0])
#use fsolve for root finding
q = fsolve(inverse_kinematics,q,parms)
data.qpos = q.copy()
u = np.zeros(3)
data.qvel = u
mj.mj_forward(model, data)

mj_end_eff_pos = data.site("tip").xpos;
jac_E = jac_end_effector(q)
jac_E = np.delete(jac_E, [2, 3, 4], axis=0)
pos =np.array([mj_end_eff_pos[0],mj_end_eff_pos[1],q[0]+q[1]+q[2]])
vel = jac_E@u

time_all = []
x_ref_all = []
y_ref_all = []
theta_ref_all = []
x_all = []
y_all = []
theta_all = []
q_all = np.empty((0, 3))
u_all = np.empty((0, 3))
qpos_all = np.empty((0, 3))
qvel_all = np.empty((0, 3))

#initialize the controller
init_controller(model,data)

#set the controller
#mj.set_mjcb_control(controller)

model.opt.timestep = 0.001

#print(geom_size)
#dim = model.geom("surface").geom_size
#print(dim)


while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):

        mj_end_eff_pos = data.site("tip").xpos;
        if (data.time<t_init):
            pos_ref[0]=x0;
            pos_ref[1]=y0;
            pos_ref[2]=theta0
        else:
            pos_ref,vel_ref,acc_ref = straight_line(data.time-t_init,x0,y0,theta0,r,f)

        parms = [pos_ref[0],pos_ref[1],pos_ref[2]]



        #theta_ref = np.pi/2;
        #parms = pos #[x_ref, y_ref, theta_ref]

        #use fsolve for root finding
        q = fsolve(inverse_kinematics, q,parms)
        jac_E = jac_end_effector(q)
        jac_E = np.delete(jac_E, [2, 3, 4], axis=0)
        inv_jac_E = np.linalg.inv(jac_E)

        Xdot_ref = np.array([vel_ref[0],vel_ref[1],vel_ref[2]])
        u = inv_jac_E@Xdot_ref

        Xddot_ref = np.array([acc_ref[0],acc_ref[1],acc_ref[2]])
        ud = inv_jac_E@Xddot_ref

        pos =np.array([mj_end_eff_pos[0],mj_end_eff_pos[1],data.qpos[0]+data.qpos[1]+data.qpos[2]])
        vel = jac_E@data.qvel


        #animate

        if (flag_traj_generation):
            data.time += model.opt.timestep
            data.qpos = q.copy()
            data.qvel = u.copy()
            mj.mj_forward(model, data)
        else:
            controller(model,data)
            mj.mj_step(model, data)

        qpos = data.qpos.copy()
        qvel = data.qvel.copy()

        if (data.time>t_init):
            time_all.append(data.time-t_init)
            x_ref_all.append(pos_ref[0])
            y_ref_all.append(pos_ref[1])
            theta_ref_all.append(pos_ref[2])
            x_all.append(mj_end_eff_pos[0])
            y_all.append(mj_end_eff_pos[1])
            theta_all.append(q[0]+q[1]+q[2])
            q_all = np.vstack([q_all, q.copy()])
            u_all = np.vstack([u_all, u.copy()])
            qpos_all = np.vstack([qpos_all, qpos.copy()])
            qvel_all = np.vstack([qvel_all, qvel.copy()])


            #print(pos-pos_ref)
            #print(vel-vel_ref)

    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

#glfw.terminate()

###########

if (1):
    colors_ref = ['black', 'black', 'black']
    linestyles_ref = ['--', '--', '--']

    colors = ['red', 'blue', 'green']
    linestyles = ['-', '-', '-']

    # Create subplots (3 rows, 1 column)
    fig1, axes1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)  # Share x-axis for alignment

    for i in range(3):
        axes1[i].plot(time_all, q_all[:, i], color=colors_ref[i], linestyle=linestyles_ref[i], linewidth=2, label='ref')
        axes1[i].plot(time_all, qpos_all[:, i], color=colors[i], linestyle=linestyles[i], linewidth=4, label='act',alpha=0.5)
        axes1[i].set_ylabel(f'q_{i}')  # Label for each subplot
        axes1[i].grid(True)
        axes1[i].legend()  # Add legend inside each subplot

    # Common x-label for the bottom row
    axes1[-1].set_xlabel('Time')

    # Create subplots (3 rows, 1 column)
    fig2, axes2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)  # Share x-axis for alignment

    for i in range(3):
        axes2[i].plot(time_all, u_all[:, i], color=colors_ref[i], linestyle=linestyles_ref[i], linewidth=2, label='ref')
        axes2[i].plot(time_all, qvel_all[:, i], color=colors[i], linestyle=linestyles[i], linewidth=4, label='act',alpha=0.5)
        axes2[i].set_ylabel(f'u_{i}')  # Label for each subplot
        axes2[i].grid(True)
        axes2[i].legend()  # Add legend inside each subplot

    # Common x-label for the bottom row
    axes2[-1].set_xlabel('Time')

    # plt.show(block=False)  # Non-blocking display
    # plt.pause(10)  # Pause for 10 seconds
    # plt.close()  # Close the plot

if (1):
    #output_dir = os.path.join(os.path.dirname(__file__), 'runs', 'planar3r_force_position')
    #os.makedirs(output_dir, exist_ok=True)

   # fig1.savefig(os.path.join(output_dir, 'force_position_q_vs_time.png'), dpi=200, bbox_inches='tight')
   # fig2.savefig(os.path.join(output_dir, 'force_position_u_vs_time.png'), dpi=200, bbox_inches='tight')

    traj_fig = plt.figure()
    plt.plot(x_ref_all, y_ref_all, label='ref', color='black', linestyle='--', linewidth=2)
    plt.plot(x_all, y_all, label='act', color='red',linewidth=4,alpha=0.5)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.title('Trajectory')
    plt.grid()
    plt.axis('equal')
    #traj_fig.savefig(os.path.join(output_dir, 'force_position_trajectory.png'), dpi=200, bbox_inches='tight')
    #plt.show()
    plt.show(block=False)
    plt.pause(5)
   # plt.close()
