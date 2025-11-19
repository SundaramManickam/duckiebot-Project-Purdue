#!/bin/bash

source /environment.sh

# Ensure VEHICLE_NAME is set (required for namespacing)
export VEHICLE_NAME=dragon

# Mount /data from the robot into the container
dt-start-data-mount

# Start ROS launchfile infrastructure
dt-launchfile-init

# Run your actual launch file
roslaunch dynamic_obstacle race_demo.launch

# Attach to lifecycle hooks / cleanup
dt-launchfile-join

