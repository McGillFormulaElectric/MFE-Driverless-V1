#!/usr/bin/env python3

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def SensorDataQoS():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)


import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import PointCloud2
from mfe_msgs.msg import Cone, Track
from std_msgs.msg import Header


@dataclass
class ConeTrack:
    id: int
    color: int
    x: float
    y: float
    # state: [x, y, vx, vy]
    state: np.ndarray = field(default_factory=lambda: np.zeros(4))
    cov: np.ndarray   = field(default_factory=lambda: np.eye(4) * 1.0)
    hits: int   = 0
    misses: int = 0


# Constant-velocity Kalman model, 10 Hz
_DT = 0.1
_F  = np.array([[1, 0, _DT, 0],
                [0, 1, 0,  _DT],
                [0, 0, 1,   0],
                [0, 0, 0,   1]], dtype=float)
_H  = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0]], dtype=float)
_Q  = np.diag([0.01, 0.01, 0.1, 0.1])
_R  = np.diag([0.04, 0.04])


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Finite XYZ points from a PointCloud2 as an Nx3 float32 array (x, y, z must be float32)."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)

    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = msg.point_step

    def extract_field(offset: int) -> np.ndarray:
        idx = np.arange(n) * step + offset
        col = np.stack([raw[idx], raw[idx+1], raw[idx+2], raw[idx+3]], axis=1)
        return col.view(np.float32).reshape(-1)

    xs = extract_field(field_map['x'].offset)
    ys = extract_field(field_map['y'].offset)
    zs = extract_field(field_map['z'].offset)

    pts = np.stack([xs, ys, zs], axis=1)
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid].astype(np.float32)


def _apply_tf(pts_xyz: np.ndarray, transform) -> np.ndarray:
    """Apply a geometry_msgs/TransformStamped to an Nx3 float32 array."""
    t = transform.transform
    tx, ty, tz = t.translation.x, t.translation.y, t.translation.z
    qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),   1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float32)
    t_vec = np.array([tx, ty, tz], dtype=np.float32)
    return (pts_xyz @ R.T) + t_vec


class BoundaryExtractor(Node):

    GRID_STEP       = 0.3   # spatial bin size for confidence tracking
    MATCH_RADIUS = 0.5   # max distance to associate a LiDAR centroid with a camera cone
    GT_MATCH_RADIUS = 2.0  # looser match for GT color fallback
    CAM_DEDUP_RADIUS = 1.2  # an unmatched camera cone this close to a LiDAR centroid is the same cone
                            # with camera position error; appending it would make a phantom duplicate
    CAM_ONLY_MAX_RANGE = 12.0  # camera-only cones beyond this are dropped: depth error grows with
                               # range and the planner's global cone map keeps phantoms forever

    def __init__(self):
        super().__init__('boundary_extractor')

        self.declare_parameter('min_observations', 3)
        self.declare_parameter('confidence_decay', 1)
        self._min_obs     = int(self.get_parameter('min_observations').value)
        self._conf_decay  = int(self.get_parameter('confidence_decay').value)

        # grid key -> [count, color, x, y, z]
        self._confidence_map: dict = {}

        self._lidar_pts: np.ndarray = np.zeros((0, 3), dtype=np.float32)
        self._camera_cones: list = []
        self._gt_cones: list = []

        self._lidar_header: Header = Header()
        self._camera_header: Header = Header()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._tracks: list[ConeTrack] = []
        self._next_track_id: int = 0

        self.declare_parameter('min_hits', 3)
        self.declare_parameter('max_misses', 5)
        self.declare_parameter('match_radius', 1.0)
        self._min_hits   = int(self.get_parameter('min_hits').value)
        self._max_misses = int(self.get_parameter('max_misses').value)
        self._match_r    = float(self.get_parameter('match_radius').value)

        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/perception/cones_uncolored',
            self._lidar_callback,
            SensorDataQoS())

        self.sub_camera = self.create_subscription(
            Track,
            'image/track',
            self._camera_callback,
            10)

        # Sim-only GT color fallback, used when the camera is not running. The full track list
        # (map frame) is independent of car position, unlike /ground_truth/cones_colored.
        self.sub_gt = self.create_subscription(
            Track,
            '/ground_truth/track_colored',
            self._gt_callback,
            10)

        self.pub_track = self.create_publisher(Track, '/planning/cones', 10)

        self.timer = self.create_timer(0.1, self._publish_fused)

        self.get_logger().info(
            f'BoundaryExtractor started. '
            f'Fusing LiDAR + camera cones with Hungarian+Kalman tracking '
            f'(min_hits={self._min_hits}, max_misses={self._max_misses}, '
            f'match_radius={self._match_r} m).')

    def _lidar_callback(self, msg: PointCloud2) -> None:
        self._lidar_pts = pointcloud2_to_xyz(msg)
        self._lidar_header = msg.header

    def _camera_callback(self, msg: Track) -> None:
        self._camera_cones = msg.track
        if msg.track:
            self._camera_header = msg.track[0].header

    def _gt_callback(self, msg: Track) -> None:
        self._gt_cones = msg.track

    def _lookup_map_tf(self, src_frame: str, stamp_msg):
        """
        Look up map <- src_frame at the sensor capture stamp. The latest TF would smear positions
        by the car's motion since capture. Falls back to the latest TF. Returns None on failure.
        """
        if stamp_msg is not None and (stamp_msg.sec or stamp_msg.nanosec):
            try:
                return self._tf_buffer.lookup_transform(
                    'map', src_frame, rclpy.time.Time.from_msg(stamp_msg))
            except TransformException:
                pass
        try:
            return self._tf_buffer.lookup_transform('map', src_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(
                f'Could not transform from {src_frame!r} to map: {e}',
                throttle_duration_sec=2.0)
            return None

    def _transform_lidar_pts_to_map(self, pts: np.ndarray, src_frame: str) -> tuple:
        """Transform an Nx3 array to map. Returns (points, success); on failure, (input, False)."""
        if pts.shape[0] == 0 or not src_frame:
            return pts, True

        tf_stamped = self._lookup_map_tf(src_frame, self._lidar_header.stamp)
        if tf_stamped is None:
            return pts, False
        return _apply_tf(pts, tf_stamped), True

    def _transform_camera_cones_to_map(self, cones: list, src_frame: str) -> tuple:
        """Transform camera cone positions to map. Returns (Nx3 array, success)."""
        cam_xyz = np.array([[c.location.x, c.location.y, c.location.z]
                            for c in cones], dtype=np.float32) if cones \
            else np.zeros((0, 3), dtype=np.float32)
        if not cones or not src_frame:
            return cam_xyz, True

        tf_stamped = self._lookup_map_tf(src_frame, self._camera_header.stamp)
        if tf_stamped is None:
            return cam_xyz, False
        return _apply_tf(cam_xyz, tf_stamped), True

    def _update_tracks(self, fused_cones: list[Cone]) -> list[ConeTrack]:
        """One predict-update cycle (Hungarian association, Kalman update). Returns confirmed tracks."""
        for t in self._tracks:
            t.state = _F @ t.state
            t.cov   = _F @ t.cov @ _F.T + _Q
            t.x = float(t.state[0])
            t.y = float(t.state[1])

        if not fused_cones:
            for t in self._tracks:
                t.misses += 1
            self._tracks = [t for t in self._tracks if t.misses <= self._max_misses]
            return [t for t in self._tracks if t.hits >= self._min_hits]

        det_xy = np.array([[c.location.x, c.location.y] for c in fused_cones], dtype=float)

        matched_track_ids: set[int] = set()
        matched_det_indices: set[int] = set()

        if self._tracks:
            track_xy = np.array([[t.x, t.y] for t in self._tracks], dtype=float)

            dx = track_xy[:, 0:1] - det_xy[:, 0]
            dy = track_xy[:, 1:2] - det_xy[:, 1]
            cost = np.sqrt(dx**2 + dy**2)

            row_ind, col_ind = linear_sum_assignment(cost)

            for r, c in zip(row_ind, col_ind):
                if cost[r, c] <= self._match_r:
                    track = self._tracks[r]
                    det   = fused_cones[c]

                    z = np.array([det.location.x, det.location.y], dtype=float)
                    S = _H @ track.cov @ _H.T + _R
                    K = track.cov @ _H.T @ np.linalg.inv(S)
                    track.state = track.state + K @ (z - _H @ track.state)
                    track.cov   = (np.eye(4) - K @ _H) @ track.cov

                    track.x = float(track.state[0])
                    track.y = float(track.state[1])

                    track.hits  += 1
                    track.misses = 0

                    if track.color == Cone.UNKNOWN and det.color != Cone.UNKNOWN:
                        track.color = det.color

                    matched_track_ids.add(id(track))
                    matched_det_indices.add(c)

        for t in self._tracks:
            if id(t) not in matched_track_ids:
                t.misses += 1

        for j, det in enumerate(fused_cones):
            if j not in matched_det_indices:
                init_state = np.array(
                    [det.location.x, det.location.y, 0.0, 0.0], dtype=float)
                new_track = ConeTrack(
                    id=self._next_track_id,
                    color=det.color,
                    x=det.location.x,
                    y=det.location.y,
                    state=init_state,
                    cov=np.eye(4) * 1.0,
                    hits=1,
                    misses=0,
                )
                self._tracks.append(new_track)
                self._next_track_id += 1

        self._tracks = [t for t in self._tracks if t.misses <= self._max_misses]

        return [t for t in self._tracks if t.hits >= self._min_hits]

    def _publish_fused(self) -> None:
        now = self.get_clock().now().to_msg()
        fused: list[Cone] = []

        lidar_pts = self._lidar_pts
        camera_cones = self._camera_cones

        lidar_src_frame = self._lidar_header.frame_id
        lidar_pts_map, lidar_tf_ok = self._transform_lidar_pts_to_map(lidar_pts, lidar_src_frame)
        if not lidar_tf_ok:
            # Drop LiDAR cones this tick: untransformed points would be mislabelled as map cones.
            lidar_pts_map = np.zeros((0, 3), dtype=np.float32)

        cam_src_frame = ''
        if camera_cones:
            cam_src_frame = self._camera_header.frame_id
        cam_xyz_map, _cam_tf_ok = self._transform_camera_cones_to_map(camera_cones, cam_src_frame)

        # Pixel-coordinate cones (no depth: z == -1 sentinel, frame 'camera_pixel') are not metric
        # map positions, so drop them. Drop all camera cones if map<-camera TF failed.
        if not _cam_tf_ok:
            camera_cones = []
            cam_xyz_map = np.zeros((0, 3), dtype=np.float32)
        else:
            metric = [(c, cam_xyz_map[k]) for k, c in enumerate(camera_cones)
                      if c.location.z != -1.0 and c.header.frame_id != 'camera_pixel'
                      and bool(np.isfinite(cam_xyz_map[k]).all())]
            camera_cones = [c for c, _ in metric]
            cam_xyz_map = (np.array([xyz for _, xyz in metric], dtype=np.float32)
                           if metric else np.zeros((0, 3), dtype=np.float32))

        cam_xy = None
        if camera_cones and cam_xyz_map.shape[0] > 0:
            cam_xy = cam_xyz_map[:, :2]

        # GT colors fill in LiDAR centroids the camera did not color. Camera color wins on a match.
        gt_xy = None
        gt_colors = None
        if self._gt_cones:
            gt_xy = np.array([[c.location.x, c.location.y]
                              for c in self._gt_cones], dtype=np.float32)
            gt_colors = [c.color for c in self._gt_cones]

        matched_camera_indices = set()

        # Step 1: color each LiDAR centroid from the nearest camera cone, then the nearest GT cone.
        for i in range(len(lidar_pts_map)):
            lx = float(lidar_pts_map[i, 0])
            ly = float(lidar_pts_map[i, 1])
            lz = float(lidar_pts_map[i, 2])

            cone = Cone()
            cone.header.stamp = now
            cone.header.frame_id = 'map'
            cone.location.x = lx
            cone.location.y = ly
            cone.location.z = lz
            cone.color = Cone.UNKNOWN

            if cam_xy is not None and len(cam_xy) > 0:
                dists = np.sqrt((cam_xy[:, 0] - lx)**2 + (cam_xy[:, 1] - ly)**2)
                best_idx = int(np.argmin(dists))
                if dists[best_idx] <= self.MATCH_RADIUS:
                    cone.color = camera_cones[best_idx].color
                    matched_camera_indices.add(best_idx)
            if cone.color == Cone.UNKNOWN and gt_xy is not None and len(gt_xy) > 0:
                dists = np.sqrt((gt_xy[:, 0] - lx)**2 + (gt_xy[:, 1] - ly)**2)
                best_idx = int(np.argmin(dists))
                if dists[best_idx] <= self.GT_MATCH_RADIUS:
                    cone.color = gt_colors[best_idx]

            fused.append(cone)

        # Step 2: add unmatched camera cones. Only within CAM_ONLY_MAX_RANGE of the car, and only if
        # map<-base_footprint is available: a phantom in the persistent map is worse than a missing cone.
        car_xy = None
        if any(j not in matched_camera_indices for j in range(len(camera_cones))):
            try:
                tf_car = self._tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
                car_xy = (tf_car.transform.translation.x, tf_car.transform.translation.y)
            except TransformException:
                pass
        for j, cam_cone in enumerate(camera_cones):
            if j not in matched_camera_indices:
                if car_xy is None:
                    continue
                if np.hypot(cam_xyz_map[j, 0] - car_xy[0],
                            cam_xyz_map[j, 1] - car_xy[1]) > self.CAM_ONLY_MAX_RANGE:
                    continue
                if lidar_pts_map.shape[0] > 0:
                    d2 = np.hypot(lidar_pts_map[:, 0] - cam_xyz_map[j, 0],
                                  lidar_pts_map[:, 1] - cam_xyz_map[j, 1])
                    if float(d2.min()) <= self.CAM_DEDUP_RADIUS:
                        continue
                cone = Cone()
                cone.header.stamp = now
                cone.header.frame_id = 'map'
                cone.location.x = float(cam_xyz_map[j, 0])
                cone.location.y = float(cam_xyz_map[j, 1])
                cone.location.z = float(cam_xyz_map[j, 2])
                cone.color = cam_cone.color
                fused.append(cone)

        # Step 3: Hungarian + Kalman tracking.
        confirmed = self._update_tracks(fused)

        # Confirmed tracks feed the grid confidence filter below, so both persistence gates apply.
        tracked_cones: list[Cone] = []
        for t in confirmed:
            c = Cone()
            c.header.stamp = now
            c.header.frame_id = 'map'
            c.location.x = float(t.state[0])
            c.location.y = float(t.state[1])
            c.location.z = 0.0
            c.color = t.color
            tracked_cones.append(c)

        color_counts: dict = {}
        for c in tracked_cones:
            color_counts[c.color] = color_counts.get(c.color, 0) + 1
        self.get_logger().info(
            f'detections={len(fused)} '
            f'tracks_total={len(self._tracks)} '
            f'tracks_confirmed={len(confirmed)} '
            f'| lidar={len(lidar_pts)} gt_src={len(self._gt_cones)} '
            f'cam={len(camera_cones)} colors={color_counts}',
            throttle_duration_sec=2.0)

        # Grid-binned confidence: +1 per frame seen, decays when unseen, publish at min_observations.
        step = self.GRID_STEP
        seen_keys: set = set()
        for cone in tracked_cones:
            gx = round(cone.location.x / step) * step
            gy = round(cone.location.y / step) * step
            key = (gx, gy)
            seen_keys.add(key)
            if key in self._confidence_map:
                entry = self._confidence_map[key]
                entry[0] = min(entry[0] + 1, 20)
                entry[1] = cone.color
            else:
                self._confidence_map[key] = [1, cone.color,
                                             cone.location.x, cone.location.y, cone.location.z]

        to_remove = []
        for key, entry in self._confidence_map.items():
            if key not in seen_keys:
                entry[0] -= self._conf_decay
                if entry[0] <= 0:
                    to_remove.append(key)
        for key in to_remove:
            del self._confidence_map[key]

        stable_cones: list[Cone] = []
        for key, (count, color, x, y, z) in self._confidence_map.items():
            if count >= self._min_obs:
                c = Cone()
                c.header.stamp    = now
                c.header.frame_id = 'map'
                c.location.x = x
                c.location.y = y
                c.location.z = z
                c.color      = color
                stable_cones.append(c)

        self.get_logger().info(
            f'temporal filter: raw={len(fused)} stable={len(stable_cones)} '
            f'tracked={len(self._confidence_map)}',
            throttle_duration_sec=2.0)

        track_msg = Track()
        track_msg.track = stable_cones
        self.pub_track.publish(track_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
