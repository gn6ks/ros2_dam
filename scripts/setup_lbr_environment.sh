#!/bin/bash
#===============================================================================
# LBR_FRI_ROS2_Stack Environment setup script
# ROS2: From Simulation to Reality - Research Project
# Author: gn6ks
# License: MIT
#===============================================================================

set -e  
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# helper functions

print_header() {
    echo -e "${BLUE}"
    echo "==============================================================================="
    echo "  LBR_FRI_ROS2_Stack Environment setup script"
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

# step 1 
check_prerequisites() {
    print_step "System Prerequisites Check"
    
    print_info "Checking Ubuntu version..."
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        if [ "$VERSION_ID" = "24.04" ]; then
            print_success "Ubuntu 24.04 LTS detected (Required for ROS2 Jazzy) (Required for lbr_fri_ros2_stack)"
        else
            print_warning "Detected Ubuntu $VERSION_ID. ROS2 Jazzy requires Ubuntu 24.04."
            print_info "Proceeding anyway, but compatibility issues may occur."
        fi
    fi
    
    print_info "Checking internet connectivity..."
    if ping -c 1 8.8.8.8 &> /dev/null; then
        print_success "Internet connection available"
    else
        print_error "No internet connection. Please connect and retry."
        exit 1
    fi
    
    print_info "Checking available disk space..."
    AVAILABLE_SPACE=$(df -BG / | tail -1 | awk '{print $4}' | sed 's/G//')
    if [ "$AVAILABLE_SPACE" -ge 30 ]; then
        print_success "Sufficient disk space available: ${AVAILABLE_SPACE}GB"
    else
        print_warning "Low disk space: ${AVAILABLE_SPACE}GB. ROS2 + Gazebo requires ~25-30GB."
    fi

    print_info "Checking for ROS2 Jazzy installation"
    if []
    else
    fi
}
