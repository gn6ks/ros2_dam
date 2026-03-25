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
    echo "  Gazebo iiwa7 simulation"
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

mock_setup_iiwa7() {
    print_step "Launching mock setup via bash"


    REPO_ROOT=$(git rev-parse --show-toplevel 2>/dev/null)
    
    if [ -z "$REPO_ROOT" ]; then
        print_error "No Git repository detected. Make sure to execute script within ros2_dam."
        return 1
    fi

    #fix: se supone que ahora debe de entrar a la carpeta correcta
    WS_PATH="$REPO_ROOT/simulation/lbr-stack"
    cd "$WS_PATH"
    
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch lbr_bringup mock.launch.py \
    model:=iiwa7
}

mock_setup_iiwa7