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

print_header() {
    echo -e "${BLUE}"
    echo "==============================================================================="
    echo "  iiwa7 Full Stack Reset (Gazebo + MoveIt2 + RViz2)"
    echo "  ROS2 Direct Application Method"
    echo "  Author: gn6ks"
    echo "==============================================================================="
    echo -e "${NC}"
}
print_success() { echo -e "${GREEN}✓ $1${NC}"; }
print_warning() { echo -e "${YELLOW}⚠ $1${NC}"; }
print_error()   { echo -e "${RED}✗ $1${NC}"; }
print_info()    { echo -e "${CYAN}ℹ $1${NC}"; }
print_step() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}  STEP: $1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

kill_process() {
    local name=$1
    local pattern=$2

    if pgrep -f "$pattern" > /dev/null 2>&1; then
        print_info "Killing: $name"
        pkill -SIGTERM -f "$pattern" 2>/dev/null || true
        sleep 1
        # fuerza el estado de kill si es que lo otro no va
        if pgrep -f "$pattern" > /dev/null 2>&1; then
            print_warning "$name didn't respond to SIGTERM, forcing SIGKILL..."
            pkill -SIGKILL -f "$pattern" 2>/dev/null || true
            sleep 1
        fi
        # verificacion de por medio
        if pgrep -f "$pattern" > /dev/null 2>&1; then
            print_error "Failed to kill $name — manual intervention may be needed"
        else
            print_success "$name stopped"
        fi
    else
        print_info "$name was not running, skipping"
    fi
}

print_header

print_step "Stopping RViz2"
kill_process "RViz2"       "rviz2"

print_step "Stopping ROS2 Controllers"
kill_process "controller_manager"      "controller_manager"
kill_process "ros2_control_node"       "ros2_control_node"

print_step "Stopping Gazebo Harmonic"
kill_process "gz sim"   "gz sim"
kill_process "gz (ruby)" "ruby"
pkill -9 gz 2>/dev/null || true   # se carga los procesos en background

print_step "Cleaning up orphan ROS2 nodes"
kill_process "ros2 launch" "ros2 launch"
kill_process "ros2 run"    "ros2 run"

print_step "Final verification"
sleep 2

REMAINING=$(pgrep -la "gz|rviz2|move_group|controller_manager|robot_state_publisher" 2>/dev/null || true)
if [ -n "$REMAINING" ]; then
    print_warning "Some processes may still be alive:"
    echo "$REMAINING"
    print_info "Run: kill -9 <PID> manually if needed"
else
    print_success "All processes confirmed dead"
fi

echo ""
print_success "Reset complete — safe to run run_simulation.sh again"
echo ""