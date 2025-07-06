# MuJoCo Interactive Viewer Guide

## Overview

An interactive GUI viewer is provided as part of the Python package in the `mujoco.viewer` module. It is based on the same codebase as the **simulate** application that ships with the MuJoCo binary releases.

Three distinct use cases are supported:

---

## 🖥️ Standalone App

Launch MuJoCo viewer directly from the command line:

### Empty Viewer
```bash
python -m mujoco.viewer
```
Launches an empty visualization session where a model can be loaded by **drag-and-drop**.

### Load Specific Model
```bash
python -m mujoco.viewer --mjcf=/path/to/some/mjcf.xml
```
Launches a visualization session for the specified model file.

---

## 🔧 Managed Viewer

Called from a Python program/script through the function `viewer.launch()`. This function **blocks** user code to support precise timing of the physics loop. 

> **Use Case:** This mode should be used if user code is implemented as engine plugins or physics callbacks, and is called by MuJoCo during `mj_step`.

### Available Methods

#### Empty Session
```python
viewer.launch()
```
Launches an empty visualization session where a model can be loaded by **drag-and-drop**.

#### With Model Only
```python
viewer.launch(model)
```
Launches a visualization session for the given `mjModel` where the visualizer internally creates its own instance of `mjData`.

#### With Model and Data
```python
viewer.launch(model, data)
```
Same as above, except that the visualizer operates directly on the given `mjData` instance. Upon exit, the data object will have been modified.

---

## ⚡ Passive Viewer

Called using `viewer.launch_passive(model, data)`. This function **does not block**, allowing user code to continue execution.

```python
viewer.launch_passive(model, data)
```

> **Important:** In this mode, the user's script is responsible for timing and advancing the physics state. Mouse-drag perturbations will not work unless the user explicitly synchronizes incoming events.

---

## 📚 Source

Documentation source: [MuJoCo Python Documentation](https://mujoco.readthedocs.io/en/stable/python.html)
