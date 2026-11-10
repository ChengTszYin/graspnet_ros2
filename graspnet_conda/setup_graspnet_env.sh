#!/bin/bash
# Conda-side provisioning for the GraspNet node.
# Run in a normal terminal with conda on PATH. This script never builds or
# installs ROS 2 — build the graspnet_ros2 package separately with system Python.
set -e

GRASPNET_WS="${GRASPNET_WS:-$HOME/graspnet_ws}"
ENV_NAME="${ENV_NAME:-graspnet}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

eval "$(conda shell.bash hook)"

echo "[1/4] Conda env '${ENV_NAME}'"
if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "      already exists; skipping creation."
  echo "      (refresh with: conda env update -n ${ENV_NAME} -f environment.yml)"
else
  conda env create -f "${HERE}/environment.yml"
fi
conda activate "$ENV_NAME"

echo "[2/4] graspnet-baseline checkout"
mkdir -p "$GRASPNET_WS"
BASELINE="$GRASPNET_WS/graspnet-baseline"
if [ -d "$BASELINE/.git" ]; then
  echo "      already cloned at $BASELINE"
else
  git clone https://github.com/graspnet/graspnet-baseline.git "$BASELINE"
fi

echo "[3/4] Compile pointnet2 and knn"
export CUDA_HOME="${CUDA_HOME:-$CONDA_PREFIX}"
for ext in pointnet2 knn; do
  if [ -f "$BASELINE/$ext/setup.py" ]; then
    (cd "$BASELINE/$ext" && python setup.py install)
  else
    echo "      ERROR: $BASELINE/$ext/setup.py not found; check the baseline checkout." >&2
    exit 1
  fi
done

echo "[4/4] RealSense checkpoint"
CKPT="$GRASPNET_WS/checkpoint-rs.tar"
if [ -f "$CKPT" ]; then
  echo "      found at $CKPT"
elif [ -n "$CHECKPOINT_URL" ]; then
  echo "      downloading from CHECKPOINT_URL"
  curl -L "$CHECKPOINT_URL" -o "$CKPT"
else
  echo "      MISSING. Download checkpoint-rs.tar (RealSense pretrained weights)"
  echo "      from the official graspnet-baseline README and place it at:"
  echo "          $CKPT"
  echo "      Or re-run with CHECKPOINT_URL=<direct link>."
fi

echo
echo "Done."
echo "  env:        $ENV_NAME"
echo "  baseline:   $BASELINE"
echo "  checkpoint: $CKPT"
echo "Set baseline_root and checkpoint_path in graspnet_ros2/config/graspnet.yaml to these paths."