#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from ultralytics import SAM
import cv2
from graspnet_ros2.srv import DetectGraspsMask
from robot_arm_interfaces.srv import RobotEntries

class SamGrasp(Node):
    def __init__(self):
        super().__init__('draw_bbox_grasp')
        self.bridge = CvBridge()
        self.frame = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.declare_parameter('sam_model', "/home/dllm/student_ros/src/graspnet_ros2/sam_ckp/mobile_sam.pt")
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.on_image, qos)
        self.grasp_client = self.create_client(DetectGraspsMask, '/graspnet/sam_detect')
        self.moveit_client = self.create_client(RobotEntries, '/Moveit_Robo_Node')
        self.last_result = None          # last DetectGraspsMask response
        self.last_mask = None
        self.build_sam_model()
        
    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    def build_sam_model(self):
        sam_pt = self.get_parameter('sam_model').value
        self.model = SAM(sam_pt)
        self.model.info()
    
    def call_detect(self, node, point):
        if not node.grasp_client.wait_for_service(timeout_sec=2.0):
            print('service /graspnet/sam_detect not available')
            return 'service not available'
        
        results = self.model(node.frame, points=point, labels=[1])
        if results is None or len(results)==0 or results[0].masks is None:
            print("Nothing has segmented")
            return 'no mask'
        
        masks = results[0].masks.data[0]
        # choose the largest mask
        if masks.ndim == 2:
            # only one mask: (H, W)
            mask_tensor = masks
        elif masks.ndim == 3:
            # multiple masks: (N, H, W)
            areas = masks.sum(dim=(1, 2))
            best_idx = areas.argmax()
            mask_tensor = masks[best_idx]
        else:
            print("Unexpected mask shape", masks.shape)
            return 'bad mask', None
        mask_np = mask_tensor.cpu().numpy().astype(np.uint8) * 255
        
        # Dilate to give the point cloud more thickness
        kernel = np.ones((9, 9), np.uint8)          # try 5, 9, 15 …
        mask_np = cv2.dilate(mask_np, kernel, iterations=1)
        
        req = DetectGraspsMask.Request()
        req.max_grasps = 5
        req.min_score = 0.0
        req.mask = self.bridge.cv2_to_imgmsg(mask_np, encoding='mono8')
        
        future = self.grasp_client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        result = future.result()
        if result is None:
            print('service call failed')
            return None, mask_np, 'service call failed'
        
        info = {
            'success': result.success,
            'message': result.message,
            'frame_id': result.frame_id,
            'poses': list(result.poses),
            'widths': list(result.widths),
            'scores': list(result.scores)
        }
        self.get_logger().info(
            f"GraspNet: success={result.success} grasps={len(result.poses)} msg={result.message}"
        )
        return info, mask_np, 'ok'
    
    def call_moveit(self, poses):
        """Send a list of Pose to RobotEntries service."""
        if not self.moveit_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('MoveIt (RobotEntries) service not available')
            return False, 'service not available'

        req = RobotEntries.Request()
        req.poses = poses

        future = self.moveit_client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        result = future.result()
        if result is None:
            self.get_logger().error('MoveIt service call failed')
            return False, 'call failed'

        self.get_logger().info(f"MoveIt: success={result.success} msg={result.message}")
        return result.success, result.message

def main():
    rclpy.init()
    node = SamGrasp()
    window = 'Sam grasp'
    cv2.namedWindow(window)
    
    drag = {'down': False, 'x': 0, 'y': 0}
    pending = {'point': None}
    
    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            drag['x'] = x
            drag['y'] = y
            drag['down'] = True
        elif event == cv2.EVENT_LBUTTONUP:
            drag['down'] = False
    
    cv2.setMouseCallback(window, on_mouse)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.frame is None:
            continue
        view = node.frame.copy()
        if drag["down"] and drag['x'] and drag['y']:
            pending['point'] = (drag['x'], drag['y'])
        
        if pending['point'] is not None:
            info, mask_np, status = node.call_detect(node, pending['point'])
            node.last_result = info
            node.last_mask = mask_np
            pending['point'] = None
            if mask_np is not None:
                overlay = view.copy()
                overlay[mask_np > 0] = (0, 255, 0)          # green where mask is on
                overlay = cv2.addWeighted(overlay, 0.6, overlay, 0.4, 0)
                if info and info.get('scores'):
                    best_score = max(info['scores'])
                    cv2.putText(
                        overlay,
                        f"best score: {best_score:.3f}  grasps: {len(info['poses'])}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                    )
                    
            cv2.putText(
                overlay,
                "m = send best grasp to MoveIt | c = close overlay | q = quit",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
            cv2.imshow("mobile_sam", overlay)
            # waitKey = cv2.waitKey(1)
                        
        cv2.imshow(window, view)
        waitKey = cv2.waitKey(1)
        if waitKey == ord('q'):
            break
        elif waitKey == ord('c'):
            cv2.destroyWindow("mobile_sam")
        elif waitKey == ord('m'):
            if node.last_result is None or not node.last_result.get('poses'):
                node.get_logger().warn("No grasp result to send. Click an object first.")
                continue

            # pick highest score
            scores = node.last_result['scores']
            poses = node.last_result['poses']
            best_idx = int(np.argmax(scores))
            best_pose = poses[best_idx]
            best_score = scores[best_idx]
            best_width = node.last_result['widths'][best_idx] if node.last_result['widths'] else 0.0

            node.get_logger().info(
                f"Sending best grasp (score={best_score:.3f}, width={best_width:.3f}) to MoveIt..."
            )
            success, msg = node.call_moveit([best_pose])
            print(f"MoveIt result: success={success} message={msg}")
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


