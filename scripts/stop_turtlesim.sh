#!/bin/bash
#===============================================================================
# TurtleSim Simulation Stopper
# ROS2: From Simulation to Reality - Research Project
# Author: gn6ks
# License: MIT
#===============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}Stopping TurtleSim simulation...${NC}"

pkill -f "turtlesim_node" 2>/dev/null && echo -e "${GREEN}✓ turtlesim_node stopped${NC}"
pkill -f "turtle_teleop_key" 2>/dev/null && echo -e "${GREEN}✓ turtle_teleop_key stopped${NC}"

echo -e "${GREEN}Simulation terminated successfully.${NC}"