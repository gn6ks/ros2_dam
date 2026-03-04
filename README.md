# ROS2: From Simulation to Reality

## A Comprehensive Framework for Robotic Development and Deployment

**Author:** gn6ks  
**Version:** 1.0.0  
**Status:** In Progress  
**License:** MIT  

---

### Abstract

This research presents a systematic approach to learning and implementing ROS2 (Robot Operating System 2), covering the complete development lifecycle from theoretical foundations to physical deployment. The work is structured around progressive learning curves, enabling practitioners with varying levels of expertise to acquire competencies in robot simulation, navigation, and real-world implementation. All resources, configurations, and documentation are made publicly available to facilitate reproducibility and community contribution.

---

### Table of Contents

1. [Introduction](docs/01_introduction.md)
2. [Fundamental Concepts](docs/02_fundamental_concepts.md)

---

### Technical Specifications

| Component | Specification |
|-----------|---------------|
| **ROS2 Distribution** | Humble Hawksbill (LTS) |
| **Operating System** | Ubuntu 22.04 LTS / Windows 11 |
| **Simulation Environment** | Gazebo Classic / Webots |

---

### Repository Structure

    ros2-investigacion/
    ├── docs/                    # Documentation and research content
    │   ├── 01_introduction.md
    │   ├── 02_installation.md
    │   ├── assets/
    │   │   ├── figures/
    │   │   └── diagrams/
    │   └── learning_paths/
    ├── ros2_ws/                 # ROS2 workspace and packages
    │   ├── src/
    │   │   ├── mi_robot_description/
    │   │   ├── mi_robot_bringup/
    │   │   └── mi_robot_navigation/
    │   └── install_isolated.sh
    ├── simulation/              # Simulation configurations and worlds
    │   ├── gazebo/
    │   │   ├── worlds/
    │   │   └── models/
    │   └── ignition/
    ├── hardware/                # Hardware-specific configurations
    │   ├── drivers/
    │   ├── calibration/
    │   └── troubleshooting/
    ├── scripts/                 # Automation and utility scripts
    │   ├── setup_ubuntu.sh
    │   ├── build_workspace.sh
    │   └── run_simulation.sh
    ├── references/              # Bibliography and external resources
    │   ├── bibliography.md
    │   └── useful_links.md
    ├── LICENSE
    ├── CONTRIBUTING.md
    └── .gitignore

---

### Citation

If you use this work in your research, please cite as:

    gn6ks. (2026). ROS2: From Simulation to Reality. GitHub Repository.
    https://github.com/gn6ks/ros2-investigacion

---

### Contact

For inquiries regarding this research, please open an issue in this repository or contact the author directly through GitHub.
