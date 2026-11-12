#!/bin/bash
# Start both the GraspNet worker (Conda) and the ROS node (system Python)
set -e

# ---------- 1. Start the Conda worker in the background ----------
CONDA_BASE="/home/dllm/anaconda3"
source "$CONDA_BASE/etc/profile.d/conda.sh"

CONFIG="${GRASPNET_CONFIG:-$HOME/student_ros/src/graspnet_ros2/config/graspnet.yaml}"
WORKER_DIR="${WORKER_DIR:-$HOME/student_ros/src/graspnet_ros2/graspnet-baseline}"

echo "Starting GraspNet worker..."
conda run -n vision --no-capture-output \
  bash -c "cd '$WORKER_DIR' && python graspnet_worker.py --config '$CONFIG'" &
WORKER_PID=$!

sleep 3

# ---------- 2. Start the ROS node on system Python ----------
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
if [ -n "$ROS_WS_SETUP" ]; then
  source "$ROS_WS_SETUP"
fi

# Drop any Conda site-packages so the system interpreter stays clean
export PYTHONPATH="$(printf '%s' "$PYTHONPATH" | tr ':' '\n' | grep -vE 'anaconda|miniconda|/envs/' | paste -sd: -)"

echo "Starting ROS node..."
/usr/bin/python3 -m graspnet_ros2.graspnet_node "$@"

# When the node exits, also stop the worker
kill $WORKER_PID 2>/dev/null || true