#!/bin/bash
# Run the ROS-free GraspNet inference worker inside the Conda env (vision by
# default). This is terminal 1; the ROS node (terminal 2) connects over localhost.
# No ROS is imported here and ROS is never installed into this env.
set -e
eval "$(conda shell.bash hook)"
conda activate "${GRASPNET_ENV:-vision}"

# Outer graspnet_ros2 package dir (contains the inner graspnet_ros2/ module and
# config/). Defaults to the build workspace source; override for another checkout.
PKG_DIR="${GRASPNET_PKG_DIR:-$HOME/student_ros/src/graspnet_ros2}"
CONFIG="${GRASPNET_CONFIG:-$PKG_DIR/config/graspnet.yaml}"
export PYTHONPATH="$PKG_DIR:${PYTHONPATH}"

exec python -m graspnet_ros2.graspnet_worker --config "$CONFIG" "$@"