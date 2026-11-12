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
only for `GraspGroup`.

## Build type

This is a single **`ament_cmake`** package. `rosidl` generates the custom service
`graspnet_ros2/srv/DetectGrasps`. The Python node is installed into the same
`graspnet_ros2` namespace.

## Layout

```
graspnet_ros2/
  package.xml
  CMakeLists.txt
  srv/DetectGrasps.srv
  launch/graspnet.launch.py
  config/graspnet.yaml
  script/run_graspnet_node.sh
  graspnet_ros2/
    __init__.py
    graspnet_node.py          # ROS node (system Python 3.10)
    draw_bbox_grasp.py        # interactive box-drawing client
  graspnet-baseline/          # local checkout of graspnet-baseline
    graspnet_worker.py        # ROS-free inference server (Conda)
    detector.py               # GraspNetDetector adapter
    models/
    utils/
    ...
  README.md
```

`graspnet-baseline` lives inside the package. The worker adds `baseline_root`
(and `baseline_root/models`) to `sys.path` at startup so it can import
`models.graspnet` and `utils.*`. The ROS node never imports the baseline.

## Prerequisites

System (apt):

```bash
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs \
                 ros-humble-realsense2-camera
```

ROS 2 Humble uses the system Python (`/usr/bin/python3`, 3.10).  
The inference worker runs in a separate Conda environment (`vision`) that already
contains torch, open3d, graspnetAPI, and the compiled pointnet2 / knn extensions.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/student_ros
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

## Configure

Edit `config/graspnet.yaml`:

- `baseline_root` → absolute path to the `graspnet-baseline` folder
- `checkpoint_path` → absolute path to `checkpoint-rs.tar`
- topic names (currently defaulting to `/camera/camera/...`)

Both the node and the worker read the same yaml.

## Run

### Option A – single script (worker + node)

```bash
cd ~/student_ros/src/graspnet_ros2/script
./run_graspnet_node.sh
```

The script:
1. Activates the `vision` Conda env and starts `graspnet_worker.py` from
   `graspnet-baseline/`.
2. Starts the ROS node with the system Python.

You still need the RealSense camera running (see below).

### Option B – two terminals (original style)

**Terminal 1 – worker (Conda)**
```bash
conda activate vision
cd ~/student_ros/src/graspnet_ros2/graspnet-baseline
python graspnet_worker.py --config ../config/graspnet.yaml
```

**Terminal 2 – ROS**
```bash
source /opt/ros/humble/setup.bash
source ~/student_ros/install/setup.bash
ros2 launch graspnet_ros2 graspnet.launch.py
```

### Camera

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  rgb_camera.color_profile:=1280,720,30 \
  depth_module.depth_profile:=1280,720,30
```

Topics expected by the node (adjust in yaml / node if yours differ):

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`

### Interactive test client

```bash
ros2 run graspnet_ros2 draw_bbox_grasp.py
```

Drag a box on the image; it calls `/graspnet/detect` and prints the results.

## Service

`graspnet_ros2/srv/DetectGrasps` on `/graspnet/detect`.

```bash
ros2 service call /graspnet/detect graspnet_ros2/srv/DetectGrasps \
  "{max_grasps: 5, min_score: 0.0, xmin: 400, ymin: 150, xmax: 900, ymax: 550}"
```

## Parameters (excerpt)

| Parameter            | Default                              | Meaning                        |
|----------------------|--------------------------------------|--------------------------------|
| `color_topic`        | `/camera/camera/color/image_raw`     | color input                    |
| `depth_topic`        | `/camera/camera/aligned_depth_to_color/image_raw` | aligned depth     |
| `camera_info_topic`  | `/camera/camera/color/camera_info`   | intrinsics                     |
| `baseline_root`      | (set in yaml)                        | path to graspnet-baseline      |
| `checkpoint_path`    | (set in yaml)                        | path to checkpoint-rs.tar      |
| `worker_host`        | `127.0.0.1`                          | worker address                 |
| `worker_port`        | `50051`                              | worker port                    |
| `max_grasps`         | `10`                                 | default when request is 0      |
| `min_score`          | `0.0`                                | default score floor            |
| `max_angle_deg`      | `30.0`                               | max approach tilt from +Z      |
| `max_frame_age_sec`  | `1.0`                                | staleness limit                |

## Frame convention

Returned poses are in the **camera optical frame** (`frame_id` from the depth
message, typically `camera_color_optical_frame`).  
Translation and width are in meters. Approach axis is the local +X of the
rotation matrix. Orientation is a unit quaternion.

The caller is responsible for TF into the robot base and for MoveIt execution.

