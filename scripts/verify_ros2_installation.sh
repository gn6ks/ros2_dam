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
NC='\033[0m'

echo "=== ROS2 Installation Verification ==="
echo ""

# Check ROS2 command
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓${NC} ROS2: $(ros2 --version)"
else
    echo -e "${RED}✗${NC} ROS2: Not installed"
fi

# Check ROS_DISTRO
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}✓${NC} ROS_DISTRO: $ROS_DISTRO"
else
    echo -e "${YELLOW}⚠${NC} ROS_DISTRO: Not set (run: source /opt/ros/humble/setup.bash)"
fi

# Check Gazebo
if command -v gzserver &> /dev/null; then
    echo -e "${GREEN}✓${NC} Gazebo: $(gzserver --version)"
else
    echo -e "${YELLOW}⚠${NC} Gazebo: Not installed"
fi

# Check RQT
if command -v rqt &> /dev/null; then
    echo -e "${GREEN}✓${NC} RQT: Available"
else
    echo -e "${YELLOW}⚠${NC} RQT: Not installed"
fi

# Check colcon
if command -v colcon &> /dev/null; then
    echo -e "${GREEN}✓${NC} colcon: Available"
else
    echo -e "${YELLOW}⚠${NC} colcon: Not installed"
fi

# Check disk space
AVAILABLE=$(df -BG / | tail -1 | awk '{print $4}')
echo -e "${GREEN}✓${NC} Available Disk Space: $AVAILABLE"

echo ""
echo "=== Verification Complete ==="