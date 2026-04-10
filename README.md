# ros2_dam
[![License](https://img.shields.io/github/license/gn6ks/ros2_dam)](https://github.com/gn6ks/ros2_dam/blob/main/LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy%20Jalisco-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-orange)](https://ubuntu.com/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green)](https://gazebosim.org/)

ROS2: **Direct Application Method** — Custom framework for KUKA LBR robots, designed for robotic development and research at the [Universitat Politècnica de València (UPV)](https://www.upv.es/). Built on top of [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack), this repository provides a fully scripted, reproducible environment for simulation and motion planning of KUKA iiwa robots.

Supported robots: `iiwa7`, `iiwa14`, `med7`, `med14`.

<table>
  <tr>
    <th align="left" width="25%">LBR IIWA 7 R800</th>
    <th align="left" width="25%">LBR IIWA 14 R820</th>
    <th align="left" width="25%">LBR Med 7 R800</th>
    <th align="left" width="25%">LBR Med 14 R820</th>
  </tr>
  <tr>
    <td align="center">
      <!-- TODO: replace with image or GIF of iiwa7 simulation/demo -->
      <i>Image / GIF — iiwa7</i>
    </td>
    <td align="center">
      <!-- TODO: replace with image or GIF of iiwa14 simulation/demo -->
      <i>Image / GIF — iiwa14</i>
    </td>
    <td align="center">
      <!-- TODO: replace with image or GIF of med7 simulation/demo -->
      <i>Image / GIF — med7</i>
    </td>
    <td align="center">
      <!-- TODO: replace with image or GIF of med14 simulation/demo -->
      <i>Image / GIF — med14</i>
    </td>
  </tr>
</table>

---

## Status

| OS | ROS Distribution | Simulator | Status |
|:---|:---|:---|:---|
| `Ubuntu 24.04` | `jazzy` | `Gazebo Harmonic` | [![Build](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/gn6ks/ros2_dam) |

---

## Documentation

> All scripts must be run from inside the repository root. They use `git rev-parse --show-toplevel` to locate the workspace automatically.

---

## Requirements

| Component | Version | Check |
|-----------|---------|-------|
| Ubuntu | 24.04 LTS | `lsb_release -a` |
| ROS2 | Jazzy Jalisco | `echo $ROS_DISTRO` |
| Python | 3.10+ | `python3 --version` |
| Git | Any recent | `git --version` |
| colcon | Any recent | `colcon version` |
| Disk space | 30 GB+ free | `df -h /` |

---

## Repository Structure

```
ros2_dam/
├── scripts/                  # All automation scripts (run from here)
├── simulation/
│   └── lbr-stack/            # Fork workspace (created by setup_workspace.sh)
│       ├── src/
│       └── install/
└── docs/
```

> The `simulation/lbr-stack/` workspace is the **forked** version of `lbr_fri_ros2_stack`. Do not replace it with the upstream repository.

---

## Quick Start

### Step 1 — ROS2 + Gazebo Harmonic Installation

```shell
chmod +x scripts/setup_ros2_environment.sh
./scripts/setup_ros2_environment.sh
```

This script handles everything: locale, ROS2 Jazzy apt source, `ros-jazzy-desktop`, Gazebo Harmonic (`gz-harmonic`), colcon, rosdep, RQT, and `.bashrc` configuration.

After it finishes, open a new terminal or run:

```shell
source ~/.bashrc
```

Verify:

```shell
chmod +x scripts/verify_ros2_installation.sh
./scripts/verify_ros2_installation.sh
```

---

### Step 2 — Fork Workspace Setup

```shell
chmod +x scripts/setup_workspace.sh
./scripts/setup_workspace.sh
```

This script:
- Creates `simulation/lbr-stack/` inside the repository
- Clones the **forked** `lbr_fri_ros2_stack` into `simulation/lbr-stack/src/`
- Imports dependencies via `vcs`
- Runs `rosdep install`
- Builds with `colcon build --symlink-install`

> **Important:** The workspace lives inside this repository at `simulation/lbr-stack/`, not in `~/ros2_ws`. All launch scripts source `simulation/lbr-stack/install/setup.bash` automatically.

Verify the build:

```shell
chmod +x scripts/verify_workspace.sh
./scripts/verify_workspace.sh
```

Expected output: `lbr_bringup` sourced and ready.

---

### Step 3 — Launching the Simulation

All scripts auto-detect the repository root and source the workspace. Run them from anywhere inside the repo.

**Mock mode** (no physics, fastest)

Terminal 1:
```shell
./scripts/run_mockup.sh
```

Terminal 2:
```shell
./scripts/run_rviz.sh
```

**Gazebo mode** (full physics)

```shell
./scripts/run_simulation.sh
```

Launches Gazebo Harmonic with `joint_trajectory_controller` for the iiwa7.

---

### Step 4 — MoveIt2 via RViz2

Requires Gazebo already running (`run_simulation.sh` in another terminal).

```shell
./scripts/run_moveit_rviz.sh
```

Opens RViz2 with the MoveIt2 MotionPlanning panel. Drag the end-effector marker → **Plan & Execute**.

| Panel Button | Action |
|---|---|
| Plan | Compute collision-free trajectory |
| Execute | Send last plan to controller |
| Plan & Execute | Both in one click |
| Stop | Halt immediately |

> To close cleanly: `Ctrl + C` in the `run_moveit_rviz.sh` terminal.

---

### Step 5 — Cleanup

To kill all simulation processes (Gazebo, RViz2, controllers, orphan nodes):

```shell
./scripts/cleanup.sh
```

---

## Demos

<table>
  <tr>
    <th align="left" width="33%">Motion Planning</th>
    <th align="left" width="33%">Gravity Compensation</th>
    <th align="left" width="33%">Trajectory Execution</th>
  </tr>
  <tr>
    <td align="center">
      <!-- TODO: replace with image or GIF of motion planning demo -->
      <i>Image / GIF — Motion Planning</i>
    </td>
    <td align="center">
      <!-- TODO: replace with image or GIF of gravity compensation demo -->
      <i>Image / GIF — Gravity Compensation</i>
    </td>
    <td align="center">
      <!-- TODO: replace with image or GIF of trajectory execution demo -->
      <i>Image / GIF — Trajectory Execution</i>
    </td>
  </tr>
</table>

---

## Scripts Reference

| Script | Purpose |
|--------|---------|
| `setup_ros2_environment.sh` | Full ROS2 Jazzy + Gazebo Harmonic install |
| `verify_ros2_installation.sh` | Post-install environment check |
| `setup_workspace.sh` | Clone fork, build workspace in `simulation/lbr-stack/` |
| `verify_workspace.sh` | Health check on built workspace |
| `run_mockup.sh` | Launch iiwa7 mock (no physics) |
| `run_rviz.sh` | Launch RViz2 (use alongside mockup) |
| `run_simulation.sh` | Launch iiwa7 in Gazebo Harmonic |
| `run_moveit_rviz.sh` | Launch MoveIt2 + RViz2 (Gazebo mode) |
| `cleanup.sh` | Kill all running simulation processes |

---

## Troubleshooting

```shell
# ROS2 not found in terminal
source /opt/ros/jazzy/setup.bash

# lbr packages not found
source simulation/lbr-stack/install/setup.bash

# Workspace not built yet
cd simulation/lbr-stack && colcon build --symlink-install

# Diagnose ROS2 issues
ros2 doctor

# Check joint states are publishing (sim must be running)
ros2 topic hz /lbr/joint_states   # Expected: ~100 Hz

# Kill stuck Gazebo processes manually
pkill -9 gz
```

---

## Citation

If you use this framework in your research or work, we would appreciate ❤️ if you could leave a ⭐ and/or cite it:

```bibtex
@misc{ros2_dam,
  author       = {gn6ks and contributors},
  title        = {ROS2\_DAM: Direct Application Method for KUKA LBR Robots},
  year         = {2026},
  publisher    = {GitHub},
  howpublished = {\url{https://github.com/gn6ks/ros2_dam}}
}
```

This project builds upon [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack). If you also use the underlying stack, please cite:

```bibtex
@article{Huber2024,
  doi       = {10.21105/joss.06138},
  url       = {https://doi.org/10.21105/joss.06138},
  year      = {2024},
  publisher = {The Open Journal},
  volume    = {9},
  number    = {103},
  pages     = {6138},
  author    = {Martin Huber and Christopher E. Mower and Sebastien Ourselin and Tom Vercauteren and Christos Bergeles},
  title     = {LBR-Stack: ROS 2 and Python Integration of KUKA FRI for Med and IIWA Robots},
  journal   = {Journal of Open Source Software}
}
```

---

## Contributors

We would like to acknowledge all contributors 🚀

[![ros2_dam contributors](https://contrib.rocks/image?repo=gn6ks/ros2_dam&max=20)](https://github.com/gn6ks/ros2_dam/graphs/contributors)

---

## Acknowledgements

| Logo | Notes |
|:--:|:---|
| <img src="https://upload.wikimedia.org/wikipedia/commons/7/71/LOGOUPV.png" alt="UPV" width="150" align="left"> | Developed at the [Universitat Politècnica de València (UPV)](https://www.upv.es/), in the context of robotic research and development. |
