#!/usr/bin/env bash

# Set some required environment variables
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib

# Run Isaac Sim
python3 $(dirname $0)/run_sim.py $@
