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
    echo "  LBR_FRI_ROS2_Stack Environment health check"
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


echo -n "ROS2 Distro: "
if [ "$ROS_DISTRO" = "jazzy" ]; then
    print_success "OK (jazzy)"
else
    print_error "FAILED (Expected jazzy, found '$ROS_DISTRO')"
fi

REPO_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo ".")
WS_PATH="$REPO_ROOT/simulation/lbr-stack"

cd $WS_PATH
source install/setup.bash

echo -n "Workspace Path: "
if [ -d "$WS_PATH" ]; then
    print_success "$WS_PATH"
else
    print_error "NOT FOUND ($WS_PATH)"
fi

echo -n "Setup File: "
if [ -f "$WS_PATH/install/setup.bash" ]; then
    print_success "Compiled and Setup found"
else
    print_error "Not compiled (Run: colcon build)"
fi

echo -n "LBR Bringup Package: "
if ros2 pkg prefix lbr_bringup &> /dev/null; then
    print_success "Sourced and Ready"
else
    print_warning "Not found in ROS_PACKAGE_PATH (Did you source install/setup.bash?)"
fi

print_info "Check completed."