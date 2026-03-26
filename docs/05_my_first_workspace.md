# 5. Workspace for iiwa 7 R800 on Gazebo

In this chapter, an introduction, implementation, and creation of different tools or packages to run in simulation environments such as Gazebo Harmonic, visualize the changes of force, speed, position, among others. With the different tools that were explored in the previous chapters in this investigation. It will also explore the ways in which robot behavior can be visualized in real time, not in the simulation environment but in the 3D visualization environment such as RViz.

## 5.x Introducction to Gazebo Harmonic

Gazebo Harmonic is the current long-term support release of the Gazebo robotics simulator. Unlike its predecessor Gazebo Classic, Gazebo Harmonic is built on the ignition framework and integrates natively with ROS2 through the ros_gz bridge packages. It provides physics simulation, sensor emulation, and a plugin system that allows external controllers — such as those provided by ros2_control — to actuate simulated joints.
For the iiwa 7 R800, Gazebo Harmonic simulates joint physics, applies effort limits, and exposes the robot's state through standard ROS2 topics and action servers, giving you an environment that closely mirrors the behavior of the real hardware.

### 5.1.1 How does Gazebo Harmonic interface work

When Gazebo Harmonic launches, it opens two components:

- 3D Viewport — renders the robot model, environment, and sensor data in real time.
- Entity tree — lists every model, link, and sensor present in the simulation world.

# 5.x Introducction to RViz 3D Visualizer
# 5.x.1 How does RViz interface work

# 5.x Introduction to lbr_fri_ros2_stack
# 5.x.1 How does lbr_fri_ros2_stack work
# 5.x.2 The purpose of lbr_fri_ros2_stack

# 5.x Environment setup for iiwa 7 R800
# 5.x.1 Process installation
# 5.x.2 Mock and visualization setup

# 5.x Automatic scripts for simulation workspace setup

# 5.x Demos on Python for Gazebo simulator

# 5.x How to create your own ROS2 package for KUKA iiwa 7 r800
# 5.x.1 Understanding the joint_trajectory.py demo

# 5.x Concerns and simulation limitationsb