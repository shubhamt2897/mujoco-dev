# MuJoCo Dev Run Guide

This repo contains a few standalone MuJoCo demos for UR5e kinematics and planar 3R force/control experiments.

## 1. Activate the environment

Run this from the repository root:

```bash
cd /home/shubham-0802/mujoco-dev
source mujoco-env/bin/activate
```

## 2. Create output folders

Use a clean folder structure so each run is saved separately:

```bash
mkdir -p runs/kinematics/ur5e_forward
mkdir -p runs/kinematics/ur5e_inverse
mkdir -p runs/control/planar3r_impedance
mkdir -p runs/control/planar3r_force
```

## 3. Run and record the kinematics demos

### UR5e forward kinematics

```bash
python -u src/Kinematics/Ur5e_Fwd_Kinematics/mj_forward_kinematics_ur5.py \
  2>&1 | tee runs/kinematics/ur5e_forward/run.log
```

### UR5e inverse kinematics

```bash
python -u src/Kinematics/ur5_inverse_kinematics/mj_inverse_kinematcs.py \
  2>&1 | tee runs/kinematics/ur5e_inverse/run.log
```

### UR5e inverse kinematics with circular motion

```bash
python -u src/Kinematics/ur5_inverse_kinematics/mj_inverse_kinematcs_circle.py \
  2>&1 | tee runs/kinematics/ur5e_inverse/run_circle.log
```

## 4. Run and record the control demos

### Impedance control

```bash
python -u src/planar_3R_hybrid_force/mj_traj_impedance.py \
  2>&1 | tee runs/control/planar3r_impedance/run.log
```

### Force control / hybrid force-position control

```bash
python -u src/planar_3R_hybrid_force/mj_traj_hybrid_force_position.py \
  2>&1 | tee runs/control/planar3r_force/run.log
```

## 5. Save plots and data

The scripts already generate plots at the end of the run, but they do not currently write CSV or image files automatically.

If you want file outputs in the same run folders, the next step is to add `plt.savefig(...)` and `np.savetxt(...)` inside the Python scripts. The bash commands above will already save the console output in each `run.log` file.

## 6. Useful notes

- Run the commands from the repository root so the relative output paths stay correct.
- Press `Ctrl+C` to stop a simulation if needed.
- If you want, I can add automatic CSV and PNG export to the scripts next.

## 7. Live UR5e FK slider and recording

Use the interactive FK viewer to move all six UR5e joint angles with sliders, watch the EE position update live, and optionally record the session to MP4.

```bash
python -u src/Kinematics/Ur5e_Fwd_Kinematics/live_fk_slider.py \
  --record-path runs/kinematics/ur5e_forward/live_fk.mp4
```

The recording button appears only when `--record-path` is provided. Click it to start and stop capture while you drag the sliders.