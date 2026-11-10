# graspnet_conda — Conda environment for the GraspNet node

This folder provisions the **Conda side** of the system, kept separate from the
ROS 2 package [`../graspnet_ros2/`](../graspnet_ros2/README.md). It creates the
`graspnet` Python environment and the external graspnet-baseline checkout +
weights. It does **not** build or install anything from ROS 2.

```
graspnet_conda/                 graspnet_ros2/  (ROS 2 package)
  environment.yml                 built with SYSTEM python (/usr/bin/python3)
  setup_graspnet_env.sh           wrapper activates this Conda env at runtime
  README.md                       node imports detector -> graspnet-baseline
```

The two meet only at runtime: the package's `script/run_graspnet_node.sh` runs
`conda activate graspnet` and imports the system `rclpy`/`cv_bridge` through
`PYTHONPATH`. ROS is never installed into this Conda env.

## Prerequisites

- Conda (Anaconda/Miniconda) with `conda` on PATH.
- `git` and `curl`.
- Default `environment.yml` targets an **x86_64 desktop with an NVIDIA GPU**
  (CUDA 11.8). For Jetson or CPU-only, see the variants below before running.

## Quick start

```bash
cd graspnet_conda
./setup_graspnet_env.sh
```

The script:

1. Creates the `graspnet` Conda env from `environment.yml` (Python 3.10, PyTorch
   + CUDA, numpy, scipy, open3d, graspnetAPI). Skips creation if it already exists.
2. Clones `https://github.com/graspnet/graspnet-baseline` into
   `~/graspnet_ws/graspnet-baseline` (skips if already present).
3. Compiles the `pointnet2` and `knn` CUDA extensions (`python setup.py install`).
4. Looks for `~/graspnet_ws/checkpoint-rs.tar`; if absent, downloads it when
   `CHECKPOINT_URL` is set, otherwise prints where to place it.

### Configuration (environment variables)

| Variable | Default | Purpose |
|---|---|---|
| `GRASPNET_WS` | `~/graspnet_ws` | where baseline + checkpoint live |
| `ENV_NAME` | `graspnet` | Conda env name (must match the wrapper) |
| `CUDA_HOME` | `$CONDA_PREFIX` | toolkit used to compile the CUDA ops |
| `CHECKPOINT_URL` | _(unset)_ | optional direct link to `checkpoint-rs.tar` |

Example:
```bash
GRASPNET_WS=$HOME/graspnet_ws CHECKPOINT_URL="<direct-link>" ./setup_graspnet_env.sh
```

## Checkpoint

`checkpoint-rs.tar` is the RealSense pretrained model. The official download link
is in the [graspnet-baseline README](https://github.com/graspnet/graspnet-baseline)
(this script does not hard-code a mirror). Place it at
`~/graspnet_ws/checkpoint-rs.tar`, or pass `CHECKPOINT_URL`. Then set
`checkpoint_path` (and `baseline_root`) in
`../graspnet_ros2/config/graspnet.yaml` to the real paths.

## Verify

```bash
conda activate graspnet
python -c "import torch, numpy, scipy, open3d, graspnetAPI, pointnet2, knn; \
print('torch', torch.__version__, '| cuda available:', torch.cuda.is_available())"
```

All imports must succeed and `pointnet2`/`knn` must load (they are the compiled
extensions). `cuda available: False` is fine on a CPU-only machine — the node
falls back to CPU.

## Variants

**Jetson Orin NX.** Do not use the `pytorch`/`pytorch-cuda` Conda packages; install
PyTorch from NVIDIA's Jetson wheels for your JetPack instead, then install the
remaining deps. Also set `num_point: 10000` in the ROS 2 config (default 20000).

**CPU-only.** Remove `pytorch-cuda` and `cuda-toolkit` from `environment.yml` and
install a CPU build of PyTorch. `pointnet2`/`knn` still compile (CPU path); the
node runs inference on CPU (slow but functional).

## Troubleshooting

- **`pointnet2`/`knn` compile fails:** `nvcc` must be present and match torch's
  CUDA version. `cuda-toolkit=11.8` in the env provides it; set
  `CUDA_HOME=$CONDA_PREFIX` (the script does this) or point `CUDA_HOME` at a system
  toolkit. Exact torch/CUDA versions are version-sensitive — follow the official
  graspnet-baseline install notes if compilation errors persist.
- **`setup.py install` deprecation warning:** harmless; the baseline uses the
  classic install. `pip install .` in each extension dir is an alternative.
- **ROS build errors about `em`/`empy`:** that is the ROS 2 package being built
  with Conda's Python. Build `graspnet_ros2` with the system Python, not this env.

## After provisioning

Build and run the ROS 2 package as described in
[`../graspnet_ros2/README.md`](../graspnet_ros2/README.md). The wrapper activates
this env automatically; you do not `conda activate` before `colcon build`.