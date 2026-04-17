#!/bin/bash
#===============================================================================
# ROS2 Direct Application Method
# Author: gn6ks 
# Email: pguifon@idf.upv.es
#===============================================================================

set -e  
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Helper Functions
print_header() {
    echo -e "${BLUE}"
    echo "==============================================================================="
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

# Step 1: System Prerequisites Check
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
}

# Step 2: System Update
update_system() {
    print_step "Updating System Packages"
    
    print_info "Updating package index..."
    sudo apt update
    
    print_info "Upgrading installed packages..."
    sudo apt upgrade -y
    
    print_success "System updated successfully"
}

# Step 3: Locale Configuration
configure_locale() {
    print_step "Configuring Locale Settings"
    
    print_info "Generating UTF-8 locale..."
    sudo locale-gen en_US en_US.UTF-8
    
    print_info "Updating locale configuration..."
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    
    print_info "Setting LANG environment variable..."
    export LANG=en_US.UTF-8
    
    print_success "Locale configured for UTF-8 support"
}

# Step 4: ROS2 Repository Configuration
configure_ros2_repositories() {
    print_step "Configuring ROS2 Repository"
    
    print_info "Installing software-properties-common..."
    sudo apt install -y software-properties-common
    
    print_info "Enabling Universe repository..."
    sudo add-apt-repository -y universe
    
    print_info "Installing curl for repository setup..."
    sudo apt install -y curl
    
    print_info "Downloading ros-apt-source package..."
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    
    print_info "Installing ros-apt-source (version: $ROS_APT_SOURCE_VERSION)..."
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    
    print_info "Updating package index after repository configuration..."
    sudo apt update
    
    print_success "ROS2 repository configured successfully"
}

# Step 5: ROS2 Installation
install_ros2() {
    print_step "Installing ROS2 Jazzy"
    
    print_info "Installing ros-jazzy-desktop package..."
    print_warning "This may take 15-30 minutes depending on your internet connection."
    sudo apt install -y ros-jazzy-desktop
    
    print_success "ROS2 Jazzy installed successfully"
}

# Step 6: Gazebo Harmonic Installation (OFFICIAL INSTRUCTIONS)
install_gazebo() {
    print_step "Installing Gazebo Harmonic"
    
    print_info "Installing Gazebo Harmonic prerequisites..."
    sudo apt-get update
    sudo apt-get install -y curl lsb-release gnupg
    
    print_info "Adding Gazebo GPG key..."
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    
    print_info "Adding Gazebo repository..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    print_info "Updating package index..."
    sudo apt-get update
    
    print_info "Installing Gazebo Harmonic metapackage..."
    print_warning "Gazebo Harmonic installation may take 10-20 minutes."
    sudo apt-get install -y gz-harmonic
    
    print_info "Installing Gazebo ROS2 Jazzy integration..."
    sudo apt install -y ros-jazzy-gazebo-dev 2>/dev/null || print_warning "gazebo-dev not available"
    sudo apt install -y ros-jazzy-gazebo-msgs 2>/dev/null || print_warning "gazebo-msgs not available"
    
    print_success "Gazebo Harmonic installed successfully"
}

# Step 7: Additional Tools Installation
install_additional_tools() {
    print_step "Installing Additional Development Tools"
    
    print_info "Installing colcon build system..."
    sudo apt install -y python3-colcon-common-extensions
    
    print_info "Installing rosdep dependency manager..."
    sudo apt install -y python3-rosdep
    
    print_info "Initializing rosdep..."
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init 2>/dev/null
        print_success "rosdep initialized successfully"
    else
        print_warning "rosdep already initialized (skipping init)"
    fi
    
    print_info "Updating rosdep database..."
    rosdep update 2>/dev/null || print_warning "rosdep update encountered warnings (non-critical)"
    
    print_info "Installing RQT and common plugins..."
    sudo apt install -y ros-jazzy-rqt ros-jazzy-rqt-common-plugins
    
    print_success "Additional tools installed successfully"
}

# Step 8: Environment Configuration (.bashrc)
configure_environment() {
    print_step "Configuring Shell Environment"
    
    print_info "Adding ROS2 sourcing to ~/.bashrc..."
    
    if grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc 2>/dev/null; then
        print_warning "ROS2 sourcing already exists in ~/.bashrc"
    else
        echo "" >> ~/.bashrc
        echo "# ROS2 Jazzy Environment Setup" >> ~/.bashrc
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
        print_success "ROS2 sourcing added to ~/.bashrc"
    fi
    
    print_info "Adding Gazebo Harmonic environment to ~/.bashrc..."
    if [ -f /usr/share/gz/harmonic/setup.bash ]; then
        if grep -q "source /usr/share/gz/harmonic/setup.bash" ~/.bashrc 2>/dev/null; then
            print_warning "Gazebo Harmonic sourcing already exists in ~/.bashrc"
        else
            echo "" >> ~/.bashrc
            echo "# Gazebo Harmonic Environment" >> ~/.bashrc
            echo "source /usr/share/gz/harmonic/setup.bash" >> ~/.bashrc
            print_success "Gazebo Harmonic sourcing added to ~/.bashrc"
        fi
    else
        print_warning "Gazebo Harmonic setup.bash not found (may not be required)"
    fi
    
    print_info "Adding colcon completion to ~/.bashrc..."
    if grep -q "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc 2>/dev/null; then
        print_warning "Colcon completion already exists in ~/.bashrc"
    else
        echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
        print_success "Colcon completion added to ~/.bashrc"
    fi
    
    print_info "Sourcing ~/.bashrc for current session..."
    source ~/.bashrc
    
    print_success "Shell environment configured successfully"
}

# Step 9: Installation Verification
verify_installation() {
    print_step "Verifying Installation"

    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash 2>/dev/null
    fi
    
    print_info "Checking ROS2 version..."
    if command -v ros2 &> /dev/null; then
        ROS2_VERSION=$(ros2 --version)
        print_success "ROS2 installed: $ROS2_VERSION"
    else
        print_error "ROS2 command not found. Installation may have failed."
        exit 1
    fi
    
    print_info "Checking ROS_DISTRO environment variable..."
    if [ -n "$ROS_DISTRO" ]; then
        print_success "ROS_DISTRO: $ROS_DISTRO"
    else
        print_error "ROS_DISTRO not set. Please source the setup script."
        exit 1
    fi
    
    print_info "Checking available middleware..."
    rmw_implementation=$(ros2 doctor --report 2>/dev/null | grep -A 5 "RMW Implementation" | tail -1 | xargs)
    print_success "RMW Implementation: $rmw_implementation"
    
    print_info "Checking Gazebo Harmonic installation..."
    if command -v gz &> /dev/null; then
        GAZEBO_VERSION=$(gz sim --version 2>/dev/null || gz --version 2>/dev/null)
        print_success "Gazebo Harmonic installed: $GAZEBO_VERSION"
    elif command -v gzserver &> /dev/null; then
        GAZEBO_VERSION=$(gzserver --version)
        print_success "Gazebo Classic installed: $GAZEBO_VERSION"
    else
        print_warning "Gazebo not found (optional for basic ROS2)"
    fi
    
    print_info "Checking RQT installation..."
    if command -v rqt &> /dev/null; then
        print_success "RQT is available"
    else
        print_warning "RQT not found (optional for visualization)"
    fi
    
    print_success "All verifications passed"
}

# Step 10: Display Summary and Next Steps
display_summary() {
    print_step "Installation Complete"
    
    echo ""
    echo -e "${GREEN}===============================================================================${NC}"
    echo -e "${GREEN}  ROS2 Jazzy + Gazebo Harmonic Environment Setup Complete${NC}"
    echo -e "${GREEN}===============================================================================${NC}"
    echo ""
    echo -e "${CYAN}Installation Summary:${NC}"
    echo "  • ROS2 Distribution: Jazzy Jalisco"
    echo "  • Ubuntu Version: 24.04 LTS"
    echo "  • Simulator: Gazebo Harmonic (gz-sim7)"
    echo "  • Environment: Persistent in ~/.bashrc"
    echo ""
    echo -e "${CYAN}Next Steps:${NC}"
    echo "  1. Close and reopen your terminal (or run: source ~/.bashrc)"
    echo "  2. Verify installation: ros2 --version"
    echo "  3. Test Gazebo: gz sim"
    echo ""
    echo -e "${CYAN}Useful Commands:${NC}"
    echo "  • ros2 pkg list          - List all available packages"
    echo "  • ros2 topic list        - List all active topics"
    echo "  • ros2 node list         - List all running nodes"
    echo "  • ros2 doctor            - Diagnose ROS2 issues"
    echo "  • gz sim                 - Launch Gazebo Harmonic"
    echo "  • gz sim -v 4            - Launch with verbose output"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo "  • Official ROS2 Docs: https://docs.ros.org/en/jazzy/"
    echo "  • Gazebo Harmonic Docs: https://gazebosim.org/docs/harmonic/"
    echo ""
    echo -e "${GREEN}===============================================================================${NC}"
    echo ""
}

# Main Execution
main() {
    print_header
    echo ""
    
    print_warning "This script will install ROS2 Jazzy + Gazebo Harmonic."
    print_warning "Estimated time: 30-50 minutes depending on internet speed."
    echo ""
    read -p "Do you want to continue? (y/n): " -n 1 -r
    echo ""
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Installation cancelled by user."
        exit 0
    fi
    
    check_prerequisites
    update_system
    configure_locale
    configure_ros2_repositories
    install_ros2
    install_gazebo
    install_additional_tools
    configure_environment
    verify_installation
    display_summary
}

main