#!/bin/bash
#===============================================================================
# ROS2 Installation Quick Verification
# ROS2: From Simulation to Reality - Research Project
# Author: gn6ks
# License: MIT
#===============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo ""
echo -e "${CYAN}===============================================================================${NC}"
echo -e "${CYAN}  ROS2 Installation Verification${NC}"
echo -e "${CYAN}  ROS2: From Simulation to Reality - Research Project${NC}"
echo -e "${CYAN}  Author: gn6ks${NC}"
echo -e "${CYAN}===============================================================================${NC}"
echo ""

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
fi

if [ -f ~/.bashrc ]; then
    source ~/.bashrc 2>/dev/null
fi

# Verification Checks
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓${NC} ROS2: $(ros2 --version)"
else
    echo -e "${RED}✗${NC} ROS2: Not installed"
    echo -e "${YELLOW}  → Run: ./scripts/setup_ros2_environment.sh${NC}"
fi

if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}✓${NC} ROS_DISTRO: $ROS_DISTRO"
    if [ "$ROS_DISTRO" = "jazzy" ]; then
        echo -e "${GREEN}  → Correct distribution for this research${NC}"
    else
        echo -e "${YELLOW}  → Warning: Expected 'jazzy', got '$ROS_DISTRO'${NC}"
    fi
else
    echo -e "${YELLOW}⚠${NC} ROS_DISTRO: Not set"
    echo -e "${YELLOW}  → Run: source /opt/ros/jazzy/setup.bash${NC}"
fi

if command -v gz &> /dev/null; then
    GAZEBO_VERSION=$(gz sim --version 2>/dev/null || gz --version 2>/dev/null)
    echo -e "${GREEN}✓${NC} Gazebo Harmonic: $GAZEBO_VERSION"
elif command -v gzserver &> /dev/null; then
    GAZEBO_VERSION=$(gzserver --version)
    echo -e "${GREEN}✓${NC} Gazebo Classic: $GAZEBO_VERSION"
    echo -e "${YELLOW}  → Note: Gazebo Harmonic recommended for ROS2 Jazzy${NC}"
else
    echo -e "${YELLOW}⚠${NC} Gazebo: Not installed"
    echo -e "${YELLOW}  → Run: sudo apt install gz-harmonic${NC}"
fi

if command -v rqt &> /dev/null; then
    echo -e "${GREEN}✓${NC} RQT: Available"
else
    echo -e "${YELLOW}⚠${NC} RQT: Not installed"
    echo -e "${YELLOW}  → Run: sudo apt install ros-jazzy-rqt ros-jazzy-rqt-common-plugins${NC}"
fi

if command -v colcon &> /dev/null; then
    echo -e "${GREEN}✓${NC} colcon: Available"
else
    echo -e "${YELLOW}⚠${NC} colcon: Not installed"
    echo -e "${YELLOW}  → Run: sudo apt install python3-colcon-common-extensions${NC}"
fi

AVAILABLE=$(df -BG / | tail -1 | awk '{print $4}')
AVAILABLE_NUM=$(echo "$AVAILABLE" | sed 's/G//')
if [ "$AVAILABLE_NUM" -ge 30 ]; then
    echo -e "${GREEN}✓${NC} Available Disk Space: $AVAILABLE"
else
    echo -e "${YELLOW}⚠${NC} Available Disk Space: $AVAILABLE (Recommended: 30GB+ for Gazebo)"
fi

if grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc 2>/dev/null; then
    echo -e "${GREEN}✓${NC} ROS2 sourcing configured in ~/.bashrc"
else
    echo -e "${YELLOW}⚠${NC} ROS2 sourcing not in ~/.bashrc"
    echo -e "${YELLOW}  → Run: echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc${NC}"
fi

if [ -f /usr/share/gz/harmonic/setup.bash ]; then
    if grep -q "source /usr/share/gz/harmonic/setup.bash" ~/.bashrc 2>/dev/null; then
        echo -e "${GREEN}✓${NC} Gazebo Harmonic sourcing configured in ~/.bashrc"
    else
        echo -e "${YELLOW}⚠${NC} Gazebo Harmonic sourcing not in ~/.bashrc"
        echo -e "${YELLOW}  → Run: echo 'source /usr/share/gz/harmonic/setup.bash' >> ~/.bashrc${NC}"
    fi
fi

echo ""
echo -e "${CYAN}===============================================================================${NC}"
echo -e "${CYAN}  Verification Complete${NC}"
echo -e "${CYAN}===============================================================================${NC}"
echo ""

if command -v ros2 &> /dev/null && [ "$ROS_DISTRO" = "jazzy" ]; then
    if command -v gz &> /dev/null; then
        echo -e "${GREEN}✓ Environment is ready for ROS2 Jazzy + Gazebo Harmonic development${NC}"
    else
        echo -e "${GREEN}✓ Environment is ready for ROS2 Jazzy development${NC}"
        echo -e "${YELLOW}⚠ Gazebo Harmonic recommended for simulation tasks${NC}"
    fi
    echo -e "${GREEN}✓ Proceed to Chapter 4: TurtleSim Simulation${NC}"
    echo -e "${CYAN}  → Run: ./scripts/run_turtlesim.sh${NC}"
else
    echo -e "${YELLOW}⚠ Environment requires attention before proceeding${NC}"
    echo -e "${CYAN}  → Run: ./scripts/setup_ros2_environment.sh${NC}"
fi

echo ""