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
    echo "  Gazebo simulation demos iiwa7 R800 script"
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

print_header

REPO_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo ".")
WS_PATH="$REPO_ROOT/simulation/lbr-stack"

source ~/Desktop/ros2_dam/simulation/lbr-stack/install/setup.bash
cd $WS_PATH
ros2 launch lbr_bringup gazebo.launch.py \
    ctrl:=joint_trajectory_controller \
    model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
print_success "Joint Trajectory Client [OK]"
print_success "Run /.cleanup.sh for total cleanup"