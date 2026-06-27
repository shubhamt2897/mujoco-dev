import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
from forward_kinematics import forward_kinematics
import utility as ut 



xml_path = '../../Models/universal_robots_ur5e/scene.xml' #xml file (assumes this is in this repo layout)
simend = 5 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
joint_index = 0
joint_step = 0.05
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def setup_compare_plot():
    plt.ion()
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("EE comparison: MuJoCo live vs FK solved")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim([-1.0, 1.0])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([0.0, 1.4])
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=22, azim=35)
    mj_trail, = ax.plot([], [], [], color="dodgerblue", linewidth=1.5, label="MuJoCo EE path")
    fk_trail, = ax.plot([], [], [], color="orange", linewidth=1.5, label="FK EE path")
    gap_line, = ax.plot([], [], [], color="crimson", linewidth=2.0, linestyle="--", label="EE gap")
    mj_marker, = ax.plot([], [], [], marker="o", markersize=10, color="dodgerblue", linestyle="none", label="MuJoCo EE")
    fk_marker, = ax.plot([], [], [], marker="^", markersize=10, color="orange", linestyle="none", label="FK solved EE")
    err_text = ax.text2D(0.02, 0.96, "", transform=ax.transAxes, va="top")
    ax.legend(loc="upper right")
    fig.tight_layout()
    return fig, ax, mj_trail, fk_trail, gap_line, mj_marker, fk_marker, err_text


def sync_qpos():
    data.qpos[:] = q
    mj.mj_forward(model, data)

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    global q
    global joint_index
    global joint_step

    if act != glfw.PRESS:
        return

    if key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        q[:] = key_qpos.copy()
        sync_qpos()
    elif key == glfw.KEY_H:
        q[:] = key_qpos.copy()
        sync_qpos()
    elif key == glfw.KEY_LEFT_BRACKET:
        joint_index = (joint_index - 1) % len(q)
    elif key == glfw.KEY_RIGHT_BRACKET:
        joint_index = (joint_index + 1) % len(q)
    elif key in (glfw.KEY_1, glfw.KEY_KP_1):
        joint_index = 0
    elif key in (glfw.KEY_2, glfw.KEY_KP_2):
        joint_index = 1
    elif key in (glfw.KEY_3, glfw.KEY_KP_3):
        joint_index = 2
    elif key in (glfw.KEY_4, glfw.KEY_KP_4):
        joint_index = 3
    elif key in (glfw.KEY_5, glfw.KEY_KP_5):
        joint_index = 4
    elif key in (glfw.KEY_6, glfw.KEY_KP_6):
        joint_index = 5
    elif key in (glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT):
        joint_step = 0.2
    elif key in (glfw.KEY_LEFT_CONTROL, glfw.KEY_RIGHT_CONTROL):
        joint_step = 0.01
    elif key in (glfw.KEY_UP, glfw.KEY_EQUAL, glfw.KEY_KP_ADD):
        q[joint_index] += joint_step
        sync_qpos()
    elif key in (glfw.KEY_DOWN, glfw.KEY_MINUS, glfw.KEY_KP_SUBTRACT):
        q[joint_index] -= joint_step
        sync_qpos()

    print(
        f"Selected joint [{joint_index + 1}/6] {joint_names[joint_index]} | "
        f"step={joint_step:.3f} | q={np.array2string(q, precision=4, suppress_small=True)}"
    )

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
cam.azimuth = -140 ; cam.elevation = -20 ; cam.distance =  2.0
cam.lookat =np.array([ 0.0 , 0.0 , 0.5 ])
#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)
key_qpos =model.key("home").qpos
#print(model.key("home").qpos) #print the home position of the robot
q = key_qpos.copy()  # copy the home position to q
compare_mj_points = []
compare_fk_points = []

compare_fig, compare_ax, mj_trail, fk_trail, gap_line, mj_marker, fk_marker, err_text = setup_compare_plot()
plt.show(block=False)

data.qpos = q.copy()
sync_qpos()

print("Keyboard controls:")
print("  [ / ]          select previous/next joint")
print("  1..6           jump directly to a joint")
print("  Up / +         increase selected joint")
print("  Down / -       decrease selected joint")
print("  Shift          coarse step")
print("  Ctrl           fine step")
print("  H or Backspace reset to home")
print(f"Starting joint: {joint_names[joint_index]}")

while not glfw.window_should_close(window):
    sync_qpos()

    mj_end_eff_pos = data.site('attachment_site').xpos
    mj_end_eff_mat = data.site('attachment_site').xmat
    mj_end_eff_quat = np.zeros(4)
    mj.mju_mat2Quat(mj_end_eff_quat, mj_end_eff_mat)

    sol, robot = forward_kinematics(q)
    py_end_eff_pos = sol.end_eff_pos
    py_end_eff_rot = sol.end_eff_rot
    py_end_eff_quat = ut.rotation2quat(py_end_eff_rot)

    compare_mj_points.append(mj_end_eff_pos.copy())
    compare_fk_points.append(py_end_eff_pos.copy())

    mj_points = np.asarray(compare_mj_points)
    fk_points = np.asarray(compare_fk_points)

    mj_trail.set_data(mj_points[:, 0], mj_points[:, 1])
    mj_trail.set_3d_properties(mj_points[:, 2])
    fk_trail.set_data(fk_points[:, 0], fk_points[:, 1])
    fk_trail.set_3d_properties(fk_points[:, 2])

    mj_marker.set_data([mj_end_eff_pos[0]], [mj_end_eff_pos[1]])
    mj_marker.set_3d_properties([mj_end_eff_pos[2]])
    fk_marker.set_data([py_end_eff_pos[0]], [py_end_eff_pos[1]])
    fk_marker.set_3d_properties([py_end_eff_pos[2]])
    gap_line.set_data([mj_end_eff_pos[0], py_end_eff_pos[0]], [mj_end_eff_pos[1], py_end_eff_pos[1]])
    gap_line.set_3d_properties([mj_end_eff_pos[2], py_end_eff_pos[2]])

    err_norm = float(np.linalg.norm(mj_end_eff_pos - py_end_eff_pos))
    err_text.set_text(
        f"EE error norm: {err_norm:.6f} m\n"
        f"MuJoCo: {np.array2string(mj_end_eff_pos, precision=4, suppress_small=True)}\n"
        f"FK: {np.array2string(py_end_eff_pos, precision=4, suppress_small=True)}"
    )

    compare_fig.canvas.draw_idle()
    plt.pause(0.001)

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

glfw.terminate()
