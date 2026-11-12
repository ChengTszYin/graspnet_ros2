#!/usr/bin/env python3
"""Show /camera/color/image_raw. Drag a box, then call /graspnet/detect."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from graspnet_ros2.srv import DetectGrasps
import cv2


class BoxGrasp(Node):
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
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.on_image, qos)
        self.client = self.create_client(DetectGrasps, '/graspnet/detect')

    def on_image(self, msg):
        print('got image', msg.width, msg.height, flush=True)
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def call_detect(node, box):
    xmin, ymin, xmax, ymax = box
    if not node.client.wait_for_service(timeout_sec=2.0):
        print('service /graspnet/detect not available')
        return 'service not available'
    req = DetectGrasps.Request()
    req.max_grasps = 5
    req.min_score = 0.0
    req.xmin = int(xmin)
    req.ymin = int(ymin)
    req.xmax = int(xmax)
    req.ymax = int(ymax)
    print('request box', req.xmin, req.ymin, req.xmax, req.ymax)
    future = node.client.call_async(req)
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    result = future.result()
    if result is None:
        print('service call failed')
        return 'service call failed'
    print('success=%s message=%s grasps=%d frame=%s' % (
        result.success, result.message, len(result.poses), result.frame_id))
    for i, pose in enumerate(result.poses):
        p = pose.position
        print('  #%d score %.3f width %.3f xyz %.3f %.3f %.3f' % (
            i, result.scores[i], result.widths[i], p.x, p.y, p.z))
    return '%s  box=(%d,%d)-(%d,%d)' % (result.message, xmin, ymin, xmax, ymax)


def main():
    rclpy.init()
    node = BoxGrasp()
    window = 'grasp bbox'
    cv2.namedWindow(window)

    drag = {'down': False, 'x0': 0, 'y0': 0, 'x1': 0, 'y1': 0}
    box = None
    pending = {'box': None}
    status = 'drag a box on the object'

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            drag['down'] = True
            drag['x0'] = drag['x1'] = x
            drag['y0'] = drag['y1'] = y
        elif event == cv2.EVENT_MOUSEMOVE and drag['down']:
            drag['x1'], drag['y1'] = x, y
        elif event == cv2.EVENT_LBUTTONUP and drag['down']:
            drag['down'] = False
            drag['x1'], drag['y1'] = x, y
            xmin, xmax = sorted((drag['x0'], drag['x1']))
            ymin, ymax = sorted((drag['y0'], drag['y1']))
            if xmax - xmin > 5 and ymax - ymin > 5:
                pending['box'] = (xmin, ymin, xmax, ymax)

    cv2.setMouseCallback(window, on_mouse)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.frame is None:
            continue
        view = node.frame.copy()

        if pending['box'] is not None:
            box = pending['box']
            pending['box'] = None
            status = 'calling service %s' % (box,)
            cv2.putText(view, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(window, view)
            cv2.waitKey(1)
            status = call_detect(node, box)
            box = None

        draw = box
        if drag['down']:
            draw = (drag['x0'], drag['y0'], drag['x1'], drag['y1'])
        if draw is not None:
            x0, y0, x1, y1 = draw
            cv2.rectangle(view, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(view, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow(window, view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            box = None
            break
        if key == ord('c'):
            box = None
            status = 'drag a box on the object'

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()