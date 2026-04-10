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
    echo "  RViz2 + MoveIt (mouse interaction)"
    echo "  ROS2 Direct Application Method"
    echo "  Author: gn6ks"
    echo "  Email: pguifon@idf.upv.es"
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

mock_setup_iiwa7() {
    print_step "Launching RViz2 with MoveIt dependencies"

    # en vez de sin fisica gazebo las trae, se usa gazebo
    ros2 launch lbr_bringup move_group.launch.py \
    mode:=gazebo \
    rviz:=true \
    model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
}

mock_setup_iiwa7