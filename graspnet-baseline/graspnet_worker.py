"""ROS-free GraspNet inference worker (runs in the vision Conda env).

The ROS node (system Python 3.10) and this worker (Conda Python 3.11) are separate
processes that talk over localhost using multiprocessing.connection. Frames cross
the wire as raw bytes + shape/dtype and grasps come back as plain Python
lists/floats, so the two interpreters' numpy versions never need to agree and no
numpy object is ever pickled across the boundary.

This module imports no ROS packages. It loads GraspNetDetector once at startup and
runs one inference per request.
"""

import argparse
import os
import sys
from pathlib import Path
import traceback
from multiprocessing.connection import Listener

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def load_params(config_path):
    with open(config_path) as handle:
        data = yaml.safe_load(handle)
    return data['graspnet_node']['ros__parameters']


def build_detector(params):
    baseline_root = params['baseline_root']
    for path in (baseline_root, os.path.join(baseline_root, 'models')):
        if path and path not in sys.path:
            sys.path.insert(0, path)
    
    # dector_base = Path(__file__).resolve().parent.parent / "graspnet_host"
    # if str(dector_base) not in sys.path:
    #     sys.path.insert(0, str(dector_base))
        
    from detector import GraspNetDetector, CameraInfo
    
    detector = GraspNetDetector(
        checkpoint_path=params['checkpoint_path'],
        num_point=params.get('num_point', 20000),
        num_view=params.get('num_view', 300),
        collision_thresh=params.get('collision_thresh', 0.01),
        voxel_size=params.get('voxel_size', 0.01),
    )
    return detector, CameraInfo


def reconstruct_frames(payload):
    color = np.frombuffer(
        payload['color_bytes'], dtype=np.float32).reshape(payload['color_shape'])
    depth = np.frombuffer(
        payload['depth_bytes'], dtype=np.float32).reshape(payload['depth_shape'])
    return color, depth

def reconstruct_mask(payload):
    if 'mask_bytes' not in payload:
        return None
    return np.frombuffer(
        payload['mask_bytes'], dtype=np.uint8).reshape(payload['mask_shape'])


def handle_request(detector, camera_info_cls, params, payload):
    color, depth = reconstruct_frames(payload)
    mask = reconstruct_mask(payload)    ### return None if no mask
    cam = payload['camera']
    camera = camera_info_cls(
        cam['width'], cam['height'], cam['fx'], cam['fy'],
        cam['cx'], cam['cy'], factor_depth=cam.get('factor_depth', 1000.0))

    max_grasps = int(payload.get('max_grasps', params.get('max_grasps', 10)))
    min_score = float(payload.get('min_score', params.get('min_score', 0.0)))
    max_angle_deg = float(
        payload.get('max_angle_deg', params.get('max_angle_deg', 30.0)))

    bbox = (
            int(payload.get('xmin', 0)),
            int(payload.get('ymin', 0)),
            int(payload.get('xmax', 0)),
            int(payload.get('ymax', 0)),
        )
    print('bbox', bbox, 'color', color.shape, 'depth', depth.shape, flush=True)
    grasps = detector.detect(color, depth, camera, max_grasps, min_score, max_angle_deg, bbox=bbox, mask=mask, visualize=True)

    out = []
    for grasp in grasps:
        quat = Rotation.from_matrix(grasp['rotation_matrix']).as_quat()
        translation = grasp['translation']
        out.append({
            'translation': [float(translation[0]), float(translation[1]),
                            float(translation[2])],
            'quaternion': [float(quat[0]), float(quat[1]),
                           float(quat[2]), float(quat[3])],
            'width': float(grasp['width']),
            'score': float(grasp['score']),
        })
    return {'success': True, 'message': '%d grasps' % len(out), 'grasps': out}


def serve(host, port, authkey, detector, camera_info_cls, params):
    address = (host, int(port))
    with Listener(address, authkey=authkey.encode('utf-8')) as listener:
        print('graspnet_worker listening on %s:%d (device=%s)'
              % (host, port, detector.device), flush=True)
        while True:
            conn = listener.accept()
            try:
                payload = conn.recv()
                try:
                    response = handle_request(
                        detector, camera_info_cls, params, payload)
                except Exception as exc:  # noqa: BLE001 - report, keep serving
                    traceback.print_exc()
                    response = {'success': False,
                                'message': 'inference failed: %s' % exc,
                                'grasps': []}
                conn.send(response)
            except (EOFError, OSError):
                pass
            finally:
                conn.close()


def main():
    parser = argparse.ArgumentParser(description='GraspNet localhost worker')
    parser.add_argument('--config', default="../config/graspnet.yaml",
                        help='Path to graspnet.yaml (shared with the ROS node)')
    parser.add_argument('--host', default=None)
    parser.add_argument('--port', type=int, default=None)
    parser.add_argument('--authkey', default=None)
    args = parser.parse_args()

    params = load_params(args.config)
    host = args.host if args.host is not None else params.get('worker_host', '127.0.0.1')
    port = args.port if args.port is not None else params.get('worker_port', 50051)
    authkey = args.authkey if args.authkey is not None else params.get('worker_authkey', 'graspnet')

    print('graspnet_worker loading checkpoint: %s' % params['checkpoint_path'],
          flush=True)
    detector, camera_info_cls = build_detector(params)
    serve(host, port, authkey, detector, camera_info_cls, params)


if __name__ == '__main__':
    main()