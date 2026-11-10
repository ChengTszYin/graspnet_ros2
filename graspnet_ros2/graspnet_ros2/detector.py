"""GraspNet-baseline inference adapter.

This module has no ROS imports. The ROS node adds the external graspnet-baseline
checkout to sys.path before constructing GraspNetDetector, so the baseline
modules (models.graspnet, utils.*) are imported lazily inside the constructor.
"""

import numpy as np
import torch


class CameraInfo:
    """Pinhole intrinsics expected by create_point_cloud_from_depth_image."""

    def __init__(self, width, height, fx, fy, cx, cy, factor_depth=1000.0):
        self.width = int(width)
        self.height = int(height)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.factor_depth = float(factor_depth)
        # graspnet-baseline's create_point_cloud_from_depth_image divides depth by
        # camera.scale; expose the same value under both names for API variants.
        self.scale = float(factor_depth)


class GraspNetDetector:
    def __init__(self, checkpoint_path, num_point=20000, num_view=300,
                 collision_thresh=0.01, voxel_size=0.01):
        from models.graspnet import GraspNet, pred_decode
        from utils.collision_detector import ModelFreeCollisionDetector
        from utils.data_utils import create_point_cloud_from_depth_image
        from graspnetAPI import GraspGroup

        self._pred_decode = pred_decode
        self._collision_detector_cls = ModelFreeCollisionDetector
        self._create_cloud = create_point_cloud_from_depth_image
        self._grasp_group_cls = GraspGroup

        self.num_point = int(num_point)
        self.num_view = int(num_view)
        self.collision_thresh = float(collision_thresh)
        self.voxel_size = float(voxel_size)
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

        net = GraspNet(
            input_feature_dim=0,
            num_view=num_view,
            num_angle=12,
            num_depth=10,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        net.load_state_dict(checkpoint['model_state_dict'])
        net.to(self.device)
        net.eval()
        self.net = net

    def detect(self, color, depth, camera, max_grasps, min_score, max_angle_deg, bbox=(0, 0, 0, 0), visualize=False):
        """bbox is (xmin, ymin, xmax, ymax) in color-image pixels. All zeros: no box."""
        
        cloud = self._create_cloud(depth, camera, organized=True)
        xmin, ymin, xmax, ymax = [int(v) for v in bbox]
        if (color is not None and xmax > xmin and ymax > ymin
            and color.shape[:2] != depth.shape[:2]):
            sx = depth.shape[1] / float(color.shape[1])
            sy = depth.shape[0] / float(color.shape[0])
            xmin, xmax = int(xmin * sx), int(xmax * sx)
            ymin, ymax = int(ymin * sy), int(ymax * sy)
            
        valid = depth > 0
        if xmax > xmin and ymax > ymin:
            height, width = depth.shape[:2]
            xmin = int(np.clip(xmin, 0, width - 1))
            xmax = int(np.clip(xmax, 0, width))
            ymin = int(np.clip(ymin, 0, height - 1))
            ymax = int(np.clip(ymax, 0, height))
            yy, xx = np.indices(depth.shape[:2])
            valid &= (xx >= xmin) & (xx < xmax) & (yy >= ymin) & (yy < ymax)
        print('bbox', (xmin, ymin, xmax, ymax), 'kept', int(valid.sum()),
            'of', int(depth.size), flush=True)

        cloud = cloud[valid]
        color_masked = None
        if color is not None and color.ndim == 3 and color.shape[:2] == depth.shape[:2]:
            color_masked = color[valid]

        near = (cloud[:, 2] > 0.15) & (cloud[:, 2] < 0.70)
        cloud = cloud[near]
        if color_masked is not None:
            color_masked = color_masked[near]

        if cloud.shape[0] < 100:
            return []

        num_point = self.num_point
        if cloud.shape[0] >= num_point:
            idxs = np.random.choice(cloud.shape[0], num_point, replace=False)
        else:
            idxs1 = np.arange(cloud.shape[0])
            idxs2 = np.random.choice(
                cloud.shape[0], num_point - cloud.shape[0], replace=True)
            idxs = np.concatenate([idxs1, idxs2])
        point_cloud = cloud[idxs].astype(np.float32)

        end_points = {
            'point_clouds': torch.from_numpy(point_cloud).unsqueeze(0).to(self.device)
        }
        with torch.no_grad():
            preds = self._pred_decode(self.net(end_points))
        # This baseline's pred_decode returns a list of per-batch (N, 17) tensors;
        # convert batch 0 to numpy for GraspGroup. Other releases return a
        # GraspGroup or a raw array directly, so handle those too.
        if isinstance(preds, self._grasp_group_cls):
            gg = preds
        else:
            arr = preds[0] if isinstance(preds, (list, tuple)) else preds
            if hasattr(arr, 'detach'):
                arr = arr.detach().cpu().numpy()
            gg = self._grasp_group_cls(np.asarray(arr))
        if len(gg) == 0:
            return []

        if self.collision_thresh > 0:
            collision_detector = self._collision_detector_cls(
                cloud, voxel_size=self.voxel_size)
            collision_mask = collision_detector.detect(
                gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
            gg = gg[~collision_mask]

        gg.nms()
        gg.sort_by_score()

        if len(gg) > 0:
            approach = gg.rotation_matrices[:, :, 0]
            norm = np.linalg.norm(approach, axis=1)
            cos = np.clip(approach[:, 2] / (norm + 1e-8), -1.0, 1.0)
            angles = np.degrees(np.arccos(cos))
            gg = gg[angles <= max_angle_deg]

        if len(gg) > 0:
            gg = gg[gg.scores >= min_score]

        if max_grasps > 0:
            gg = gg[:max_grasps]

        if len(gg) > 0:
            self._show(cloud, color_masked, gg)

        results = []
        for i in range(len(gg)):
            results.append({
                'translation': np.asarray(gg.translations[i], dtype=np.float64),
                'rotation_matrix': np.asarray(gg.rotation_matrices[i], dtype=np.float64),
                'width': float(gg.widths[i]),
                'score': float(gg.scores[i]),
                'depth': float(gg.depths[i]),
            })
        return results
    
    def _show(self, points, colors, gg):
        import open3d as o3d
        print('opening Open3D, close the window to finish the service call', flush=True)
        points = np.asarray(points, dtype=np.float32)
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        if colors is not None and len(colors) == len(points):
            cloud.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float32))
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='GraspNet')
        vis.add_geometry(cloud)
        vis.add_geometry(frame)
        for gripper in gg.to_open3d_geometry_list():
            vis.add_geometry(gripper)

        center = points.mean(axis=0)
        view = vis.get_view_control()
        view.set_lookat(center.tolist())
        view.set_front([0.0, 0.0, -1.0])  # stand on the camera side, look along +Z
        view.set_up([0.0, -1.0, 0.0])      # image up is -Y
        view.set_zoom(0.5)
        vis.run()
        vis.destroy_window()