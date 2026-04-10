# ROS2_DAM: ROS2 Direct Application Method

**Stack:** ROS2 Jazzy Jalisco · Ubuntu 24.04 LTS · Gazebo Harmonic · KUKA iiwa 7 R800

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
ROS2_DAM/
├── scripts/                  # All automation scripts (run from here)
├── simulation/
│   └── lbr-stack/            # Fork workspace (created by setup_workspace.sh)
│       ├── src/
│       └── install/
└── docs/
```

> The `simulation/lbr-stack/` workspace is the **forked** version of `lbr_fri_ros2_stack`. Do not replace it with the upstream repository.

---

## Step 1 — ROS2 + Gazebo Harmonic Installation

```bash
chmod +x scripts/setup_ros2_environment.sh
./scripts/setup_ros2_environment.sh
```

This script handles everything: locale, ROS2 Jazzy apt source, `ros-jazzy-desktop`, Gazebo Harmonic (`gz-harmonic`), colcon, rosdep, RQT, and `.bashrc` configuration.

After it finishes, open a new terminal or run:
```bash
source ~/.bashrc
```

Verify:
```bash
chmod +x scripts/verify_ros2_installation.sh
./scripts/verify_ros2_installation.sh
```

---

## Step 2 — Fork Workspace Setup

```bash
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
```bash
chmod +x scripts/verify_workspace.sh
./scripts/verify_workspace.sh
```

Expected output: `lbr_bringup` sourced and ready.

---

## Step 3 — Launching the Simulation

All scripts auto-detect the repository root and source the workspace. Run them from anywhere inside the repo.

### Mock mode (no physics, fastest)

Terminal 1:
```bash
./scripts/run_mockup.sh
```

Terminal 2:
```bash
./scripts/run_rviz.sh
```

### Gazebo mode (full physics)

```bash
./scripts/run_simulation.sh
```

Launches Gazebo Harmonic with `joint_trajectory_controller` for the iiwa7.

---

## Step 4 — MoveIt2 via RViz2

Requires Gazebo already running (`run_simulation.sh` in another terminal).

```bash
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

## Step 5 — Cleanup

To kill all simulation processes (Gazebo, RViz2, controllers, orphan nodes):

```bash
./scripts/cleanup.sh
```

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

```bash
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

**License:** MIT · **ROS2:** Jazzy Jalisco · **Simulator:** Gazebo Harmonic