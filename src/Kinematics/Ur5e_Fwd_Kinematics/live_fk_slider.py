import argparse
from pathlib import Path

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider

from forward_kinematics import forward_kinematics


START_Q = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Live UR5e forward-kinematics viewer with sliders and optional recording."
    )
    parser.add_argument(
        "--record-path",
        type=str,
        default=None,
        help="Optional MP4 path. When set, a record toggle button appears and frames can be written to video.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Frame rate used for MP4 recording.",
    )
    return parser.parse_args()


def joint_points(robot):
    points = [np.zeros(3)]
    for body_id in range(1, len(robot.body) + 1):
        points.append(robot.body[body_id].H_global[:3, 3].copy())
    return np.asarray(points)


def format_vector(vec):
    return np.array2string(np.asarray(vec), precision=4, suppress_small=True)


def build_viewer(record_path=None, fps=30):
    q = START_Q.copy()
    sol, robot = forward_kinematics(q)

    fig = plt.figure(figsize=(15, 9))
    ax = fig.add_axes([0.06, 0.26, 0.62, 0.68], projection="3d")
    ax_info = fig.add_axes([0.72, 0.30, 0.24, 0.60])
    ax_info.axis("off")

    ax.set_title("UR5e live forward kinematics")
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
    ee_text = ax_info.text(0.0, 0.95, "", va="top", ha="left", fontsize=12, family="monospace")

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

    record_state = {
        "writer": None,
        "recording": False,
        "path": Path(record_path) if record_path else None,
    }

    def capture_frame():
        writer = record_state["writer"]
        if writer is None:
            return
        fig.canvas.draw()
        frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.append_data(frame)

    def update_visualization(_=None):
        current_q = np.array([slider.val for slider in sliders], dtype=float)
        sol, robot_state = forward_kinematics(current_q)
        points = joint_points(robot_state)

        for line, start, end in zip(joint_lines, points[:-1], points[1:]):
            line.set_data([start[0], end[0]], [start[1], end[1]])
            line.set_3d_properties([start[2], end[2]])

        ee = sol.end_eff_pos
        ee_marker.set_data([ee[0]], [ee[1]])
        ee_marker.set_3d_properties([ee[2]])

        ee_text.set_text(
            "EE position\n"
            f"{format_vector(ee)}\n\n"
            "Joint angles\n"
            f"{format_vector(current_q)}"
        )

        fig.canvas.draw_idle()
        if record_state["recording"]:
            capture_frame()

    def toggle_record(_event):
        if record_state["path"] is None:
            return

        if record_state["recording"]:
            if record_state["writer"] is not None:
                record_state["writer"].close()
                record_state["writer"] = None
            record_state["recording"] = False
            record_button.label.set_text("Start recording")
            fig.canvas.draw_idle()
            return

        output_path = record_state["path"]
        output_path.parent.mkdir(parents=True, exist_ok=True)
        record_state["writer"] = imageio.get_writer(output_path, fps=fps)
        record_state["recording"] = True
        record_button.label.set_text("Stop recording")
        capture_frame()

    for slider in sliders:
        slider.on_changed(update_visualization)

    record_button = None
    if record_state["path"] is not None:
        ax_record = fig.add_axes([0.74, 0.18, 0.18, 0.06])
        record_button = Button(ax_record, "Start recording")
        record_button.on_clicked(toggle_record)
        ax_info.text(0.0, 0.18, f"Record path\n{record_state['path']}", va="top", ha="left", fontsize=10)

    update_visualization()

    try:
        plt.show()
    finally:
        if record_state["writer"] is not None:
            record_state["writer"].close()


def main():
    args = parse_args()
    build_viewer(record_path=args.record_path, fps=args.fps)


if __name__ == "__main__":
    main()