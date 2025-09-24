#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Function to handle errors without closing terminal
error_exit() {
  echo "$1" >&2
  exit 1
}

# Build the tube_mpc package
colcon build --packages-select tube_mpc --cmake-clean-cache || error_exit "Build failed, exiting."

# Source the setup script
source install/setup.bash

# Run the tube_mpc node
ros2 run tube_mpc tube_mpc_node || error_exit "ROS2 node failed to run, exiting."

