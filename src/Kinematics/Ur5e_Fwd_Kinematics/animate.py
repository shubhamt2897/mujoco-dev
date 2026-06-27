import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
import utility as ram


def _joint_points(robot):
    points = [np.zeros(3)]
    for body_id in range(1, len(robot.body) + 1):
        points.append(robot.body[body_id].H_global[:3, 3].copy())
    return np.asarray(points)

def animate(robot):

    end_eff_pos_local = robot.params.end_eff_pos_local
    end_eff_quat_local = robot.params.end_eff_quat_local
    R_end_eff = ram.quat2rotation(end_eff_quat_local )

    # Initialize the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Animation')
    ax.grid(True)

    # Set axis limits as [-1, 1, -1, 1, -1, 1]
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # Hold the previous point
    prev = np.array([0, 0, 0])
    # Plot the robot links
    for i, body in robot.body.items():
        # Get the current global position
        curr = robot.body[i].H_global[:3, 3]  # Extract translation component from H_global

        # Plot the link as a line
        ax.plot([prev[0], curr[0]],[prev[1], curr[1]],[prev[2], curr[2]],linewidth=5, color='magenta')

        # Update the previous point
        prev = curr
        # Handle the end-effector if this is the last body
        if i == len(robot.body):
            # Calculate the global position of the end-effector
            end_eff_pos = robot.body[i].H_global @ np.append(end_eff_pos_local, 1)  # Homogeneous coordinates
            end_eff_pos = end_eff_pos[:3]  # Extract translation\
            end_eff_rot = robot.body[i].H_global[:3, :3] @ R_end_eff

            curr = end_eff_pos
            # Plot the link to the end-effector
            ax.plot([prev[0], curr[0]],[prev[1], curr[1]],[prev[2], curr[2]],linewidth=5, color='magenta')


    # Set equal aspect ratio for proper visualization
    ax.set_box_aspect([1, 1, 1])  # Equal scaling for x, y, z axes


    #print(ax.elev)  # Current elevation angle
    #print(ax.azim)  # Current azimuth angle
    ax.view_init(elev=18, azim=33)

    origin = end_eff_pos #np.array([0,0,0])
    R = end_eff_rot  #np.eye(3)
    arrow_length = 0.2
    dirn_x = np.array([1, 0, 0]); dirn_x = R.dot(dirn_x);
    dirn_y = np.array([0, 1, 0]); dirn_y = R.dot(dirn_y);
    dirn_z = np.array([0, 0, 1]); dirn_z = R.dot(dirn_z);
    ax.quiver(origin[0],origin[1],origin[2],dirn_x[0],dirn_x[1],dirn_x[2],
             length=arrow_length, arrow_length_ratio = .1,normalize=True,color='red')
    ax.quiver(origin[0],origin[1],origin[2],dirn_y[0],dirn_y[1],dirn_y[2],
             length=arrow_length, arrow_length_ratio = .1,normalize=True,color='green')
    ax.quiver(origin[0],origin[1],origin[2],dirn_z[0],dirn_z[1],dirn_z[2],
             length=arrow_length, arrow_length_ratio = .1,normalize=True,color='blue')
    ax.scatter(origin[0], origin[1], origin[2], color='orange', s=50, label='EndEff')  # `s` sets the size of the marker


    # Display the plot
    # plt.show(block=False)
    # plt.pause(5)
    # plt.close()
    #set view
    plt.show()


def animate_with_sliders(robot, forward_kinematics, start_q, title="Robot Animation"):
    q = np.asarray(start_q, dtype=float).copy()
    _, robot = forward_kinematics(q)
    end_eff_pos_local = robot.params.end_eff_pos_local
    end_eff_quat_local = robot.params.end_eff_quat_local
    R_end_eff = ram.quat2rotation(end_eff_quat_local)
    selected_joint = [0]
    joint_step = [0.05]

    fig = plt.figure(figsize=(15, 9))
    ax = fig.add_axes([0.06, 0.26, 0.62, 0.68], projection="3d")
    ax_info = fig.add_axes([0.72, 0.30, 0.24, 0.60])
    ax_info.axis("off")

    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.grid(True)
    ax.set_xlim([-1.0, 1.0])
    ax.set_ylim([-1.0, 1.0])
    ax.set_zlim([0.0, 1.4])
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=22, azim=35)

    joint_lines = []
    for _ in range(len(robot.body)):
        line, = ax.plot([], [], [], linewidth=4, color="magenta")
        joint_lines.append(line)

    ee_marker, = ax.plot([], [], [], marker="o", markersize=8, color="orange", linestyle="none")
    ee_text = ax_info.text(0.0, 0.98, "", va="top", ha="left", fontsize=12, family="monospace")

    sliders = []
    joint_names = [robot.body[i].name for i in range(1, len(robot.body) + 1)]

    slider_top = 0.20
    slider_height = 0.022
    slider_gap = 0.008
    slider_left = 0.12
    slider_width = 0.58

    for index, name in enumerate(joint_names):
        slider_y = slider_top - index * (slider_height + slider_gap)
        ax_slider = fig.add_axes([slider_left, slider_y, slider_width, slider_height])

        joint_range = robot.body[index + 1].joint_range
        lower = float(joint_range[0])
        upper = float(joint_range[1])
        if not np.isfinite(lower) or not np.isfinite(upper) or np.isclose(lower, upper):
            lower, upper = -np.pi, np.pi

        slider = Slider(
            ax=ax_slider,
            label=name,
            valmin=lower,
            valmax=upper,
            valinit=float(q[index]),
            valfmt="%.2f",
        )
        sliders.append(slider)

    def format_vector(vec):
        return np.array2string(np.asarray(vec), precision=4, suppress_small=True)

    def update_joint(index, value):
        sliders[index].set_val(float(value))

    def on_key(event):
        if event.key in ("[", "left"):
            selected_joint[0] = (selected_joint[0] - 1) % len(sliders)
        elif event.key in ("]", "right"):
            selected_joint[0] = (selected_joint[0] + 1) % len(sliders)
        elif event.key in ("1", "2", "3", "4", "5", "6"):
            selected_joint[0] = int(event.key) - 1
        elif event.key in ("shift", "shift+up", "shift+down"):
            joint_step[0] = 0.2
            return
        elif event.key in ("control", "ctrl", "ctrl+up", "ctrl+down"):
            joint_step[0] = 0.01
            return
        elif event.key in ("up", "+"):
            update_joint(selected_joint[0], sliders[selected_joint[0]].val + joint_step[0])
        elif event.key in ("down", "-"):
            update_joint(selected_joint[0], sliders[selected_joint[0]].val - joint_step[0])
        elif event.key in ("h", "backspace"):
            for index, slider in enumerate(sliders):
                update_joint(index, float(q[index]))
        else:
            return

        update_visualization()

    def update_visualization(_=None):
        current_q = np.array([slider.val for slider in sliders], dtype=float)
        sol_state, robot_state = forward_kinematics(current_q)
        points = _joint_points(robot_state)

        for line, start, end in zip(joint_lines, points[:-1], points[1:]):
            line.set_data([start[0], end[0]], [start[1], end[1]])
            line.set_3d_properties([start[2], end[2]])

        ee_pos = robot_state.body[len(robot_state.body)].H_global @ np.append(end_eff_pos_local, 1)
        ee_pos = ee_pos[:3]
        ee_rot = robot_state.body[len(robot_state.body)].H_global[:3, :3] @ R_end_eff

        ee_marker.set_data([ee_pos[0]], [ee_pos[1]])
        ee_marker.set_3d_properties([ee_pos[2]])

        ee_text.set_text(
            "EE position\n"
            f"{format_vector(ee_pos)}\n\n"
            "Joint angles\n"
            f"{format_vector(current_q)}\n\n"
            "Controls\n"
            f"joint: {joint_names[selected_joint[0]]}\n"
            "drag sliders or use [ ] / 1..6 / up / down / h"
        )

        fig.canvas.draw_idle()

    for slider in sliders:
        slider.on_changed(update_visualization)

    fig.canvas.mpl_connect("key_press_event", on_key)

    update_visualization()
    plt.show()
