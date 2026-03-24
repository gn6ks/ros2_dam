#!/bin/bash
#===============================================================================
# TurtleSim Automated Simulation Launcher
# ROS2: From Simulation to Reality - Research Project
# Author: gn6ks
# License: MIT
#===============================================================================
set -e 

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Helper Functions
print_header() {
    echo -e "${BLUE}"
    echo "==============================================================================="
    echo "  TurtleSim Simulation - ROS2: From Simulation to Reality"
    echo "  Author: gn6ks"
    echo "==============================================================================="
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Step 1: Verify ROS2 Environment
check_ros2_environment() {
    print_info "Checking ROS2 environment..."
    
    if [ -z "$ROS_VERSION" ]; then
        print_error "ROS2 environment is not sourced."
        print_info "Please run: source /opt/ros/jazzy/setup.bash"
        print_info "Or add it to your ~/.bashrc for automatic sourcing."
        exit 1
    fi
    
    print_success "ROS2 environment detected: $ROS_DISTRO"
}

# Step 2: Verify/Install TurtleSim Package
check_turtlesim_installation() {
    print_info "Checking TurtleSim package installation..."
    
    if ros2 pkg list | grep -q "turtlesim"; then
        print_success "TurtleSim package is installed"
    else
        print_warning "TurtleSim package not found. Installing..."
        sudo apt update
        sudo apt install ros-jazzy-turtlesim -y
        print_success "TurtleSim package installed successfully"
    fi
}

# Step 3: Verify/Install RQT Tools
check_rqt_installation() {
    print_info "Checking RQT tools installation..."
    
    if ! command -v rqt &> /dev/null; then
        print_warning "RQT tools not found. Installing..."
        sudo apt install ros-jazzy-rqt -y
        sudo apt install ros-jazzy-rqt-common-plugins -y
        print_success "RQT tools installed successfully"
    else
        print_success "RQT tools are available"
    fi
}

# Step 4: Cleanup Function (Trap for Ctrl+C)
cleanup() {
    echo ""
    print_warning "Stopping TurtleSim simulation..."
    
    if [ -n "$TURTLESIM_PID" ]; then
        kill $TURTLESIM_PID 2>/dev/null || true
    fi
    
    print_success "All nodes terminated. Simulation stopped."
    echo ""
    print_info "Thank you for using TurtleSim. See you in the next chapter!"
    echo ""
    exit 0
}

trap cleanup SIGINT SIGTERM

# Step 5: Launch Simulation Nodes
launch_simulation() {
    print_info "Launching TurtleSim simulation..."
    echo ""
    
    print_info "Starting turtlesim_node..."
    ros2 run turtlesim turtlesim_node &
    TURTLESIM_PID=$!
    sleep 2
    
    if ps -p $TURTLESIM_PID > /dev/null; then
        print_success "turtlesim_node is running (PID: $TURTLESIM_PID)"
    else
        print_error "Failed to start turtlesim_node"
        exit 1
    fi
    
    echo ""
    print_success "Simulation node launched successfully!"
}

# Step 6: Display Usage Instructions
display_instructions() {
    echo ""
    echo "==============================================================================="
    echo "  CONTROLS"
    echo "==============================================================================="
    echo ""
    echo "  Arrow Keys     →  Move the turtle (↑ forward, ↓ backward, ←/→ rotate)"
    echo "  Space          →  Stop all movement"
    echo "  Delete         →  Clear the turtle's trail"
    echo "  Q              →  Quit teleoperation node"
    echo ""
    echo "==============================================================================="
    echo "  ADDITIONAL COMMANDS (Open new terminal to execute)"
    echo "==============================================================================="
    echo ""
    echo "  List topics:           ros2 topic list"
    echo "  Monitor pose:          ros2 topic echo /turtle1/pose"
    echo "  Publish velocity:      ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.0}}\""
    echo "  Spawn new turtle:      ros2 service call /spawn turtlesim/srv/Spawn \"{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}\""
    echo "  Clear trail:           ros2 service call /clear std_srvs/srv/Empty"
    echo "  Reset simulation:      ros2 service call /reset std_srvs/srv/Empty"
    echo "  Launch RQT:            rqt"
    echo ""
    echo "==============================================================================="
    echo "  TO STOP SIMULATION"
    echo "==============================================================================="
    echo ""
    echo "  Press Ctrl + C in this terminal"
    echo ""
    echo "==============================================================================="
    echo ""
}

# Step 7: Launch Teleop in Foreground (FIXED)
launch_teleop() {
    print_info "Starting turtle_teleop_key... (Press Ctrl+C to stop)"
    echo ""
    ros2 run turtlesim turtle_teleop_key
}

main() {
    print_header
    echo ""
    
    check_ros2_environment
    check_turtlesim_installation
    check_rqt_installation
    
    echo ""
    launch_simulation
    display_instructions
    launch_teleop
    
    cleanup
}

main