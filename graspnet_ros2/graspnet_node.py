#!/usr/bin/env python3
"""ROS 2 node: thin adapter that delegates inference to a localhost worker.

Runs on the system Python (3.10) with rclpy/cv_bridge from ROS 2 Humble. Stores
the latest color/depth/intrinsics from the RealSense topics and, only when
/graspnet/detect is called, sends one frame to the GraspNet worker (a separate
Conda process) over localhost. Poses leave in the camera optical frame; the caller
is responsible for TF and for any MoveIt2 execution.

No torch/scipy/graspnet imports here: frames cross the wire as raw bytes + shape
and grasps come back as plain lists, so this node never touches the worker's
numpy/torch versions.
"""
import threading
from multiprocessing.connection import Client

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError

from graspnet_ros2.srv import DetectGrasps


class GraspNetNode(Node):
    def __init__(self):
        super().__init__('graspnet_node')

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._busy = False
        self._color = None
        self._depth = None
        self._camera = None
        self._depth_frame_id = ''
        self._depth_stamp_ns = None

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('worker_host', '127.0.0.1')
        self.declare_parameter('worker_port', 50051)
        self.declare_parameter('worker_authkey', 'graspnet')
        self.declare_parameter('max_grasps', 10)
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('max_frame_age_sec', 1.0)
        # Inference-side params live in the shared yaml and are read by the worker;
        # declare them here so ROS accepts the overrides without erroring.
        self.declare_parameter('baseline_root', '')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('num_point', 20000)
        self.declare_parameter('num_view', 300)
        self.declare_parameter('collision_thresh', 0.01)
        self.declare_parameter('voxel_size', 0.01)

        self._worker_address = (
            self.get_parameter('worker_host').value,
            int(self.get_parameter('worker_port').value))
        self._worker_authkey = self.get_parameter('worker_authkey').value.encode('utf-8')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, self.get_parameter('color_topic').value,
            self._color_cb, sensor_qos)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self._depth_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._camera_info_cb, sensor_qos)

        self._service_group = ReentrantCallbackGroup()
        self.create_service(
            DetectGrasps, '/graspnet/detect', self._detect_cb,
            callback_group=self._service_group)

        self.get_logger().info(
            'graspnet_node ready (worker=%s:%d)' % self._worker_address)

    def _color_cb(self, msg):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except CvBridgeError:
            return
        arr = img.astype(np.float32) / 255.0
        with self._lock:
            self._color = arr

    def _depth_cb(self, msg):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError:
            return
        arr = img.astype(np.float32)
        with self._lock:
            self._depth = arr
            self._depth_frame_id = msg.header.frame_id
            self._depth_stamp_ns = (
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

    def _camera_info_cb(self, msg):
        camera = {
            'width': int(msg.width),
            'height': int(msg.height),
            'fx': float(msg.k[0]),
            'fy': float(msg.k[4]),
            'cx': float(msg.k[2]),
            'cy': float(msg.k[5]),
            'factor_depth': 1000.0,
        }
        with self._lock:
            self._camera = camera

    def _detect_cb(self, request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = 'busy'
                return response
            if self._color is None or self._depth is None or self._camera is None:
                response.success = False
                response.message = 'missing color, depth, or camera info'
                return response
            max_age = float(self.get_parameter('max_frame_age_sec').value)
            now_ns = self.get_clock().now().nanoseconds
            age = (now_ns - self._depth_stamp_ns) * 1e-9
            if age > max_age:
                response.success = False
                response.message = 'stale frame'
                return response
            color = self._color.copy()
            depth = self._depth.copy()
            camera = dict(self._camera)
            frame_id = self._depth_frame_id
            self._busy = True

        try:
            max_grasps = (int(request.max_grasps) if request.max_grasps > 0
                          else int(self.get_parameter('max_grasps').value))
            min_score = (float(request.min_score) if request.min_score > 0
                         else float(self.get_parameter('min_score').value))
            max_angle_deg = float(self.get_parameter('max_angle_deg').value)

            payload = {
                'color_bytes': color.tobytes(),
                'color_shape': list(color.shape),
                'depth_bytes': depth.tobytes(),
                'depth_shape': list(depth.shape),
                'camera': camera,
                'max_grasps': max_grasps,
                'min_score': min_score,
                'max_angle_deg': max_angle_deg,
                'xmin': int(request.xmin),
                'ymin': int(request.ymin),
                'xmax': int(request.xmax),
                'ymax': int(request.ymax),
            }
            result = self._call_worker(payload)

            response.frame_id = frame_id
            if not result.get('success', False):
                response.success = False
                response.message = result.get('message', 'worker failed')
                return response

            response.success = True
            response.message = result.get('message', '')
            for grasp in result.get('grasps', []):
                pose = Pose()
                translation = grasp['translation']
                pose.position.x = float(translation[0])
                pose.position.y = float(translation[1])
                pose.position.z = float(translation[2])
                quat = grasp['quaternion']
                pose.orientation.x = float(quat[0])
                pose.orientation.y = float(quat[1])
                pose.orientation.z = float(quat[2])
                pose.orientation.w = float(quat[3])
                response.poses.append(pose)
                response.widths.append(float(grasp['width']))
                response.scores.append(float(grasp['score']))
        except Exception as exc:  # noqa: BLE001 - report failures, never crash the node
            response.success = False
            response.message = 'inference failed: %s' % exc
            response.frame_id = frame_id
        finally:
            with self._lock:
                self._busy = False
        return response

    def _call_worker(self, payload):
        conn = Client(self._worker_address, authkey=self._worker_authkey)
        try:
            conn.send(payload)
            return conn.recv()
        finally:
            conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()