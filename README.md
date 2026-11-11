# graspnet_ros2

A thin ROS 2 **Humble** adapter that runs **graspnet-baseline** on a single
RealSense D435i frame and returns 6-DoF grasp poses through the
`/graspnet/detect` service. The camera is eye-in-hand; the arm is driven by a
**separate** MoveIt2 node. This package only detects — it never calls MoveIt,
moves the arm, or actuates the gripper.

Inference is **decoupled into two processes** so the ROS node and the model can
use different Python versions:

```
terminal 2 (system Python 3.10)            terminal 1 (Conda, e.g. vision 3.11)
┌─────────────────────────────┐          ┌──────────────────────────────────┐
│ graspnet_node.py            │ localhost│ graspnet_worker.py               │
│ rclpy + cv_bridge           │  bytes + │ torch + graspnet-baseline        │
│ Image -> numpy -> RPC client│ ─shape──>│ GraspNetDetector.detect()        │
│ Pose[] <- plain lists       │ <─lists──│ GraspGroup -> scipy quaternion   │
└─────────────────────────────┘          └──────────────────────────────────┘
```

The node stores the latest color/depth/intrinsics and sends **one frame per
service call** to the worker over `multiprocessing.connection` (localhost, shared
authkey). Frames cross as raw bytes + shape/dtype; grasps return as plain Python
lists/floats. No numpy object is pickled across the boundary, so the two
interpreters' numpy versions never need to match. The worker loads the checkpoint
once at startup.

Inference is graspnet-baseline (`GraspNet` + `pred_decode`). `graspnetAPI` is used
only for `GraspGroup`. No AnyGrasp / `gsnet`.

## Build type (important)

This is a single **`ament_cmake`** package, not `ament_python`. The spec requires
the custom service type `graspnet_ros2/srv/DetectGrasps`, and a pure
`ament_python` package cannot generate `.srv` interfaces. `rosidl` generates the
service inside this package, and `ament_cmake_python` installs the Python node
into the same `graspnet_ros2` namespace (the shared `__init__.py` is trivial, so
the two installs coexist). There is no `setup.py`/`setup.cfg`; `CMakeLists.txt`
replaces them.

## Layout

```
graspnet_ros2/
  package.xml
  CMakeLists.txt
  srv/DetectGrasps.srv
  launch/graspnet.launch.py
  config/graspnet.yaml
  script/run_graspnet_node.sh        # system-Python wrapper, executable
  graspnet_ros2/__init__.py
  graspnet_ros2/graspnet_node.py     # ROS node (system Python 3.10), RPC client
  graspnet_ros2/graspnet_worker.py   # ROS-free inference server (Conda Python)
  graspnet_ros2/detector.py          # ROS-free inference adapter (used by worker)
  README.md
```

`graspnet-baseline` is an external checkout, never vendored. The **worker** adds
`baseline_root` (and `baseline_root/models`) to `sys.path` at startup; the node
never imports the baseline. The worker is started from
[`../graspnet_conda/run_worker.sh`](../graspnet_conda/README.md).

## Prerequisites

System (apt):

```bash
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs \
                 ros-humble-realsense2-camera
```

ROS 2 Humble stays on the system Python (`/usr/bin/python3`, 3.10). `rclpy` and
`cv_bridge` come from `/opt/ros/humble/lib/python3.10/site-packages`; they are
**not** installed into Conda. The node imports no torch/scipy/graspnet.

The inference worker runs in a **separate Conda env** and is fully decoupled from
ROS, so its Python version is independent — an existing `vision` env (Python 3.11,
torch, open3d, graspnetAPI, compiled `pointnet2`/`knn_pytorch`) works as-is. See
[`../graspnet_conda/`](../graspnet_conda/README.md) for provisioning a fresh env or
reusing an existing one, and for `run_worker.sh`.

Set `baseline_root` and `checkpoint_path` in `config/graspnet.yaml` to the real
paths (the worker reads them). The baseline repo provides `models/graspnet.py`,
`utils/collision_detector.py`, and `utils/data_utils.py`. Both processes read the
**same** yaml: the node uses the topic/worker/filter keys, the worker uses the
baseline/checkpoint/inference keys.

## Build

From a ROS workspace whose `src/` contains this package (no Conda needed to
build — only ROS):

```bash
source /opt/ros/humble/setup.bash
cd ~/graspnet_ws_ros          # workspace root with src/graspnet_ros2
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

## Configure

Edit `config/graspnet.yaml`: replace `/home/USER/...` in `baseline_root` and
`checkpoint_path` with real paths. On Jetson Orin NX set `num_point: 10000`
(default stays `20000`).

## Run

Two terminals. Keep the same `ROS_DOMAIN_ID` on both (the worker uses no ROS, but
the node and the camera/MoveIt must share a domain).

**Terminal 1 — inference worker (Conda).** Loads the checkpoint once and listens
on `worker_host:worker_port` from the config:

```bash
cd graspnet_conda
GRASPNET_ENV=vision \
GRASPNET_PKG_DIR=$HOME/student_ros/src/graspnet_ros2 \
  ./run_worker.sh
# -> graspnet_worker listening on 127.0.0.1:50051 (device=cuda:0)
```

`GRASPNET_ENV` defaults to `vision`; `GRASPNET_PKG_DIR` defaults to
`~/student_ros/src/graspnet_ros2` (the outer package dir, so `import
graspnet_ros2.detector` resolves). `GRASPNET_CONFIG` overrides the yaml path.
Extra args pass through, e.g. `./run_worker.sh --port 50052`.

**Terminal 2 — ROS node (system Python, no Conda).** Start the RealSense driver,
MoveIt2, and TF elsewhere so the three topics are publishing, then:

```bash
source /opt/ros/humble/setup.bash
source ~/student_ros/install/setup.bash
ros2 launch graspnet_ros2 graspnet.launch.py \
     ros_ws_setup:=$HOME/student_ros/install/setup.bash
```

`ros_ws_setup` is exported as `ROS_WS_SETUP` so the wrapper can source the
workspace that provides `graspnet_ros2`. The wrapper forces `/usr/bin/python3`,
sources system Humble, strips any leaked Conda paths, and execs the node. It does
**not** activate Conda. Equivalent manual run:

```bash
ROS_WS_SETUP=$HOME/student_ros/install/setup.bash \
  ros2 run graspnet_ros2 run_graspnet_node.sh \
  --ros-args --params-file $(ros2 pkg prefix graspnet_ros2)/share/graspnet_ros2/config/graspnet.yaml
```

If the worker is not yet listening, `/graspnet/detect` returns `success=false`
with a connection error; start terminal 1 first. The node reconnects on every
call, so restarting the worker needs no node restart.

## Service

`graspnet_ros2/srv/DetectGrasps` on `/graspnet/detect`:

```
# Request
int32 max_grasps          # 0 -> parameter default
float32 min_score         # 0 -> parameter default
---
bool success
string message
string frame_id
geometry_msgs/Pose[] poses
float32[] widths
float32[] scores
```

Behavior: missing color/depth/intrinsics → `success=false` with a short message;
frame older than `max_frame_age_sec` → `"stale frame"`; a call arriving during
inference → `"busy"` (one inference at a time). The checkpoint loads once at
startup. Poses/widths/scores are in score order, truncated to `max_grasps`.

```bash
ros2 service call /graspnet/detect graspnet_ros2/srv/DetectGrasps \
     "{max_grasps: 5, min_score: 0.0}"
```

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `color_topic` | `/camera/color/image_raw` | color input |
| `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | aligned depth (mm) |
| `camera_info_topic` | `/camera/color/camera_info` | intrinsics |
| `baseline_root` | `/home/USER/graspnet_ws/graspnet-baseline` | added to `sys.path` |
| `checkpoint_path` | `/home/USER/graspnet_ws/checkpoint-rs.tar` | weights |
| `num_point` | `20000` | sampled points (10000 on Orin NX) |
| `num_view` | `300` | GraspNet views |
| `collision_thresh` | `0.01` | model-free collision threshold (0 disables) |
| `voxel_size` | `0.01` | collision-detector voxel size |
| `max_grasps` | `10` | default cap when request is 0 |
| `min_score` | `0.0` | default score floor when request is 0 |
| `max_angle_deg` | `30.0` | max approach tilt from camera +Z |
| `max_frame_age_sec` | `1.0` | staleness limit |

## Frame and grasp convention

- Returned pose is the graspnet gripper frame expressed in the **camera optical
  frame** (`frame_id` = the depth message frame, typically
  `camera_color_optical_frame`: +Z forward, +X right, +Y down).
- Translation and width are in **meters**.
- Approach axis is local **+X** (column 0) of the rotation matrix.
- Orientation is a unit quaternion from `scipy.spatial.transform.Rotation`.
- No base-frame transform is hard-coded and no `T_camera2end.yaml` is loaded; the
  caller uses TF. Gripper-axis fixing belongs in the MoveIt node.

The downstream pick node (not implemented here) must: call `/graspnet/detect`
only while the arm is stopped at a view pose; TF each pose into `base_link`;
build a pre-grasp by backing off ~0.08 m along the approach axis; ask MoveIt2 for
IK and a collision-free plan; execute pre-grasp, Cartesian approach, close
gripper, retreat.

## Acceptance checks

From a non-Conda terminal, with the camera publishing and the wrapper running:

```bash
python3 -c "import rclpy"                       # inside the wrapper env: exit 0
ros2 service list | grep /graspnet/detect
ros2 service call /graspnet/detect graspnet_ros2/srv/DetectGrasps "{max_grasps: 5, min_score: 0.0}"
```

Pass criteria: service exists; with live aligned depth `success=true` (or a
message explaining an empty scene) and no crash; every pose has a finite position
and unit quaternion; `frame_id` equals the depth frame (not `base_link`); calling
twice does not reload the checkpoint; CPU fallback works when CUDA is absent
(node starts, service returns, inference is just slow).

## Out of scope

Training, dataset download, graspnetAPI evaluation; AnyGrasp SDK / license /
`gsnet`; gripper driver, MoveIt config, hand-eye calibration capture; continuous
tracking; VLM bounding boxes (v1 uses the full valid depth image).