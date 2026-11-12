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
        self.client = self.create_client(DetectGraspsMask, '/graspnet/sam_detect')
        self.build_sam_model()
        
    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    def build_sam_model(self):
        sam_pt = self.get_parameter('sam_model').value
        self.model = SAM(sam_pt)
        self.model.info()
    
    def call_detect(self, node, point):
        if not node.client.wait_for_service(timeout_sec=2.0):
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
        
        future = self.client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        result = future.result()
        if result is None:
            print('service call failed')
            return 'service call failed'
        print('success=%s message=%s grasps=%d' % (result.success, result.message, len(result.poses)))
        return result.message, mask_np

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
            result, mask_np = node.call_detect(node, pending['point'])
            pending['point'] = None
            if mask_np is not None:
                overlay = view.copy()
                overlay[mask_np > 0] = (0, 255, 0)          # green where mask is on
                overlay = cv2.addWeighted(overlay, 0.6, overlay, 0.4, 0)
            help = "Press c to close this window"
            cv2.putText(overlay, help, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("mobile_sam", overlay)
            waitKey = cv2.waitKey(1)
                        
        cv2.imshow(window, view)
        waitKey = cv2.waitKey(1)
        if waitKey == ord('q'):
            break
        if waitKey == ord('c'):
            cv2.destroyWindow("mobile_sam")
    cv2.destroyAllWindows()
        

if __name__ == "__main__":
    main()


