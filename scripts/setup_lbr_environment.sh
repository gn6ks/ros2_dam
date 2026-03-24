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

check_prerequisites() {
    print_step "System Prerequisites Check"
    
    print_info "Checking Ubuntu version..."
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        if [ "$VERSION_ID" = "24.04" ]; then
            print_success "Ubuntu 24.04 LTS detected (Required for ROS2 Jazzy)"
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

    print_info "Checking for ROS2 Jazzy installation..."
    if command -v ros2 &> /dev/null; then
        print_success "ROS2: $(ros2 --version | head -n 1)"
    else
        print_error "ROS2: Not installed"
        print_info "→ Run: ./scripts/setup_ros2_environment.sh"
    fi

    if [ -n "$ROS_DISTRO" ]; then
        print_success "ROS_DISTRO: $ROS_DISTRO"
        if [ "$ROS_DISTRO" = "jazzy" ]; then
            print_success "Correct distribution"
        else
            print_warning "Warning: Expected 'jazzy', got '$ROS_DISTRO'"
        fi
    else
        print_error "ROS_DISTRO: Not set | Run: source /opt/ros/jazzy/setup.bash"
    fi
}

installation_development_tools() {
    print_step "Installation of Development Tools"
    
    print_info "Updating package lists..."
    sudo apt update
    
    print_info "Installing ROS2 development tools and dependencies..."
    sudo apt install -y \
        ros-dev-tools \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-flake8-docstrings \
        python3-pip \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        wget

    print_info "Initializing rosdep..."
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    print_success "Development tools installed successfully"
}

workspace_creation() {
    print_step "Creating workspace, cloning and installing dependencies"
    
    export FRI_CLIENT_VERSION=1.15
    print_info "Exporting FRI client version: $FRI_CLIENT_VERSION"

    print_info "Creating workspace directory: lbr-stack/src"
    mkdir -p lbr-stack/src && cd lbr-stack || { print_error "Failed to enter workspace"; return 1; }

    print_info "Cloning lbr_fri_ros2_stack | branch: jazzy"
    if [ ! -d "src/lbr_fri_ros2_stack" ]; then
        git clone https://github.com/lbr-stack/lbr_fri_ros2_stack.git -b jazzy src/lbr_fri_ros2_stack
    else
        print_info "Repository already exists, skipping clone."
    fi
    
    print_info "Importing dependencies from .yaml"
    vcs import src < src/lbr_fri_ros2_stack/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml

    print_info "Installing ROS dependencies with rosdep"
    sudo apt update
    rosdep update
    rosdep install --from-paths src -i -r -y --rosdistro jazzy

    print_success "Workspace ready to be built with 'colcon build'"
}

colcon_build() {
    print_step "Building workspace with colcon"
    
    if colcon build --symlink-install; then
        print_success "Colcon build [OK]"
    else
        print_error "Colcon build failed"
        exit 1
    fi
}

verify_build() {
    print_step "Verifying Build Output"
    
    if [ -f "install/setup.bash" ]; then
        print_success "Setup file found: install/setup.bash"
    else
        print_error "Setup file NOT found. Build might have failed silently."
        exit 1
    fi

    if [ -d "install/lbr_bringup" ]; then
        print_success "Package 'lbr_bringup' found in install folder."
    else
        print_error "Package 'lbr_bringup' is missing."
        exit 1
    fi
}

mock_setup_iiwa7() {
    print_step "Launching mock setup via bash"
    source install/setup.bash
    
    print_info "Starting LBR Bringup Mock for iiwa7..."
    ros2 launch lbr_bringup mock.launch.py model:=iiwa7 &
    LAUNCH_PID=$!
    
    sleep 10
    
    if ps -p $LAUNCH_PID > /dev/null; then
        print_success "Mock Setup running [OK]"
    else
        print_error "Mock Setup failed to launch."
        exit 1
    fi
}

main() {
    print_header
    check_prerequisites
    installation_development_tools
    workspace_creation
    colcon_build
    verify_build
    mock_setup_iiwa7
    print_step "Environment Setup Complete"

    print_info "In second terminal run setup_lbr_visualizer.sh"
}

main