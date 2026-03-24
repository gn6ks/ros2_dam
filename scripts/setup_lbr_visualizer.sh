#!/bin/bash

set -e  
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}"
    echo "==============================================================================="
    echo "  RViz visualizer"
    echo "  ROS2: From Simulation to Reality - Research Project"
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
    echo -e "${CYAN}ℹ $1${NC}"
}

print_step() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}  STEP: $1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

execute_rviz(){
    print_step "Source setup.bash & launch lbr_bringup"

    cd ~/lbr-stack
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch lbr_bringup rviz.launch.py \
    rviz_cfg_pkg:=lbr_bringup \
    rviz_cfg:=config/mock.rviz

    print_success "rviz launch [OK]"
}

execute_rviz