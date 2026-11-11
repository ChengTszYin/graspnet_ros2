#!/bin/bash
# Run the ROS node on the SYSTEM Python (3.10). No Conda here: inference lives in
# the separate worker process (graspnet_conda/run_worker.sh). The node only needs
# rclpy/cv_bridge from ROS 2 Humble and talks to the worker over localhost.
set -e
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
if [ -n "$ROS_WS_SETUP" ]; then
  source "$ROS_WS_SETUP"
fi
# Drop any Conda site-packages that leaked from the parent shell so the system
# interpreter never tries to import a numpy built for another Python version.
export PYTHONPATH="$(printf '%s' "$PYTHONPATH" | tr ':' '\n' | grep -vE 'anaconda|miniconda|/envs/' | paste -sd: -)"
exec /usr/bin/python3 -m graspnet_ros2.graspnet_node "$@"