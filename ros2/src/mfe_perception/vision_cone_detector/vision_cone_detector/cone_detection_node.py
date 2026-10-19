#!/usr/bin/env python3

import threading
import cv2
import torch
import numpy as np
import queue

from typing import Generator

import math

import rclpy
from rclpy.node import Node
from ultralytics import YOLO

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from mfe_msgs.msg import Cone, Track

class ConeDetectionNode(Node):

    def __init__(self):
        super().__init__("cone_detection_node")

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.declare_parameter('depth_callback', False)
        self.declare_parameter('camera_frame', 'camera_base')
        # Pinhole back-projection yields OPTICAL-convention coords (X right, Y down,
        # Z forward). If camera_frame is a ROS body-convention frame (X forward,
        # Y left, Z up) — e.g. the EUFS sim's zed_depth_camera_frame, whose
        # "optical" TF joints are identity-rotated — set output_body_frame so the
        # published cone locations match the frame they are stamped with.
        self.declare_parameter('output_body_frame', False)

        self.camera_frame = self.get_parameter("camera_frame").value
        self.output_body_frame = self.get_parameter("output_body_frame").value

        # Declare and get model path parameter
        self.declare_parameter("model_path", "yolo11n.pt")  # default to yolo11n.pt if none found
        self.model_path = self.get_parameter("model_path").value

        self.get_logger().info(f"Using YOLO Model: { self.model_path }")

        # Load YOLO model with custom weights
        self.device = "cuda" if torch.cuda.is_available() else "cpu"  
        self.model = YOLO(self.model_path).to(self.device)  # load model onto the device

        # Obtain ROS parameters for YOLO mode
        self.declare_parameter("yolo_conf", value=0.5)
        self.declare_parameter("yolo_iou", value=0.3)
        self.conf = self.get_parameter("yolo_conf").value
        self.iou = self.get_parameter("yolo_iou").value

        # Constructs queue for frame generation
        self.frame_queue = queue.Queue(maxsize=10)

        # Camera intrinsics (populated from CameraInfo topic)
        self.fx = None
        self.fy = None
        self.cx_c = None
        self.cy_c = None
        self.camera_info_received = False

        # subscribers
        self.create_subscription(Image, "image/input", self.callback, qos_profile)

        if self.get_parameter("depth_callback").value:
            self.create_subscription(Image, "image/depth", self.depth_callback, qos_profile) # exists if depth camera provides it

        self.create_subscription(CameraInfo, "camera/camera_info", self.camera_info_callback, qos_profile)

        # publisher
        self.cone_image_publisher = self.create_publisher(Image, "image/cones_image", qos_profile) 
        self.cone_publisher = self.create_publisher(Cone, "image/cones", qos_profile) 
        self.track_publisher = self.create_publisher(Track, "image/track", qos_profile) 
        self.centres_point_publisher = self.create_publisher(PointCloud2, "image/cone_centres", qos_profile) # the cone locations (can be visualized in rviz)

        # store latest depth frame (if any)
        self.latest_depth = None

        # start processing thread so detections run
        self.get_logger().info("Starting object prediction thread")
        self.processing_t = threading.Thread(target=self.object_predicting, daemon=True)
        self.processing_t.start()

        # UNUSED: Start tracker thread
        # self.object_tracking_t = threading.Thread(target=self.object_predicting, daemon=True)
        # self.object_tracking_t.start()

    @staticmethod
    def _imgmsg_to_bgr(msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to a BGR numpy array without cv_bridge."""
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        enc = msg.encoding
        if enc in ('rgb8', 'bgr8'):
            frame = data.reshape(msg.height, msg.width, 3)
            return frame[..., ::-1].copy() if enc == 'rgb8' else frame.copy()
        if enc in ('rgba8', 'bgra8'):
            frame = data.reshape(msg.height, msg.width, 4)[..., :3]
            return frame[..., ::-1].copy() if enc == 'rgba8' else frame.copy()
        # fallback: treat as raw bytes, best-effort
        channels = max(1, len(data) // (msg.height * msg.width))
        return data.reshape(msg.height, msg.width, channels)

    def callback(self, msg: Image) -> None:
        """
        ROS2 subscription callback. Convert the ROS Image to OpenCV (numpy) and enqueue it.
        If the queue is full, drop the oldest frame to make room (so we stay 'real-time').
        
        Args:
            msg: sensor_msgs.msg.Image
        """
        frame = self._imgmsg_to_bgr(msg)

        # Enqueue (frame, capture_stamp): the stamp travels with the frame so
        # detections are published with the CAPTURE time, not the publish time —
        # stamp-aware TF consumers then place cones where the car was when the
        # image was taken, cancelling YOLO inference latency (100-300 ms on CPU
        # ≈ 1-2 m of car motion at speed).
        # Non-blocking enqueue: a ROS callback must never block the executor.
        item = (frame, msg.header.stamp)
        try:
            self.frame_queue.put_nowait(item)
            self.get_logger().debug(f"Enqueued frame, queue size={self.frame_queue.qsize()}")
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frame_queue.put_nowait(item)
            except queue.Full:
                pass
            self.get_logger().debug("Frame queue full; dropped oldest frame and enqueued new one")

        return
    
    def depth_callback(self, msg: Image) -> None:
        """
        ROS2 Subscription callback. Applies the concurrent rgbd object bounding box onto a integer
        depth mask. Obtains the average over the bounding box, and calculates the depth of
        objects.
        """

        try:
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if msg.encoding == '32FC1':
                depth = data.view(np.float32).reshape(msg.height, msg.width)
            else:
                # 16UC1 (e.g. D435i): integer millimeters -> meters
                depth = data.view(np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        except Exception:
            self.get_logger().error("Failed to convert depth image")
            return

        self.latest_depth = depth
        return

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """
        ROS2 subscription callback for camera intrinsics.
        Stores focal lengths and principal point from the camera matrix K
        (row-major: [fx, 0, cx, 0, fy, cy, 0, 0, 1]).
        Only reads the first message received — intrinsics are assumed static.
        """
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx_c = msg.k[2]
            self.cy_c = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f"Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.cx_c:.2f}, cy={self.cy_c:.2f}"
            )

    def frame_generator(self) -> Generator[np.ndarray, None, None]:
        """
        Python generator func that yields OpenCV frames pulled from self.frame_queue.
        model.track() can treat this as a 'video stream'.

        Note: Function blocks the node until video frames are available

        Args:
            None

        Returns:
            frame: Generator[np.ndarray, None, None]
        """
        while rclpy.ok():
            try:
                item = self.frame_queue.get()   # blocking get
            except queue.Empty:
                continue
            if item is None:
                break

            yield item   # (frame, capture_stamp)

    def object_predicting(self) -> None:
        """
        Launches YOLO's prediction model on a continuous stream of frames coming 
        from frame_generator().
        """

        self.get_logger().info("object_predicting thread started")
    
        for frame, capture_stamp in self.frame_generator():
            self.get_logger().debug("Pulled frame from generator")

            # run object detection using the configured confidence/IoU thresholds
            detections = self.model(frame, conf=self.conf, iou=self.iou, verbose=False)

            annotated_frame = detections[0].plot()

            # process detections FIRST: mean-colour ROI sampling must read the clean
            # frame, before render_detection draws green boxes into it
            try:
                self.process_and_publish_detections(detections, frame, self.latest_depth, capture_stamp)
            except Exception as e:
                self.get_logger().error(f"Error processing detections: {e}")

            # render bounding boxes (mutates frame — visualisation only)
            self.render_detection(frame, detections)

            # publish annotated image
            image_msg = Image()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.camera_frame
            image_msg.height = annotated_frame.shape[0]
            image_msg.width = annotated_frame.shape[1]
            image_msg.encoding = 'bgr8'
            image_msg.is_bigendian = 0
            image_msg.step = annotated_frame.shape[1] * 3
            image_msg.data = annotated_frame.tobytes()
            self.cone_image_publisher.publish(image_msg)

        return
    

    def process_and_publish_detections(self, detections, frame: np.ndarray, depth_frame: np.ndarray,
                                       capture_stamp=None):
        """
        From detections produced by YOLO, compute per-object centre (x,y,z), average colour,
        object pixel clusters as point cloud (x,y,z in pixel coords + depth), and publish:
          - image/objects2  PointCloud2 with all points from object clusters
          - image/cone_centres PointCloud2 with one point per object (centres)
          - image/track (Track msg) if available (best-effort)
        Notes:
          - Converting pixel (u,v,depth) to 3D in meters requires camera intrinsics; this code
            publishes points in image pixel coordinates with z=depth (meters if depth provided).
        """
        all_cluster_points = []
        all_cluster_colors = []

        # Depth may arrive at a different resolution than RGB (e.g. decimated depth
        # stream). Scale RGB pixel coords into depth-image coords before indexing.
        d_sy = d_sx = 1.0
        dh = dw = 0
        if depth_frame is not None:
            dh, dw = depth_frame.shape[:2]
            fh, fw = frame.shape[:2]
            d_sy, d_sx = dh / fh, dw / fw

        # YOLO class index → mfe_msgs/Cone color constant.
        # Trained model classes (best.pt): 0=blue_cone, 1=yellow_cone,
        # 2=orange_cone (SMALL), 3=large_orange_cone (BIG), 4=unknown_cone.
        # (2/3 were swapped before — small oranges came out as ORANGE_BIG, which
        # pollutes the finish detector's orange-gate clustering.)
        _YOLO_CLS_TO_CONE = {0: Cone.BLUE, 1: Cone.YELLOW, 2: Cone.ORANGE_SMALL, 3: Cone.ORANGE_BIG}

        centres = []       # list of (x,y,z) — metric 3D or pixel sentinel
        centre_colors = [] # list of Cone color ints (from YOLO class) — for Track msg
        centre_rgb = []    # list of (R,G,B) uint8 triples — for PointCloud2 visualization
        centre_frames = [] # 'camera_base' (metric) or 'camera_pixel' (no depth)
        centre_areas = []

        for r in detections:
            boxes = r.boxes
            for box in boxes:
                # box.xyxy returns tensor [[x1,y1,x2,y2]]

                xyxy = box.xyxy[0].to('cpu').detach().numpy().astype(int)
                x1, y1, x2, y2 = xyxy.tolist()

                # clamp to image bounds
                h, w = frame.shape[:2]
                x1 = max(0, min(w - 1, x1))
                x2 = max(0, min(w - 1, x2))
                y1 = max(0, min(h - 1, y1))
                y2 = max(0, min(h - 1, y2))
                if x2 <= x1 or y2 <= y1:
                    continue

                # centre pixel coordinates
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # YOLO predicted class → cone color (replaces unreliable RGB heuristic)
                yolo_cls = int(box.cls[0].item()) if box.cls is not None and len(box.cls) > 0 else -1
                cone_color = int(_YOLO_CLS_TO_CONE.get(yolo_cls, Cone.UNKNOWN))

                # avg colour inside bbox kept for pointcloud visualisation only
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                mean_bgr = cv2.mean(roi)[:3]
                mean_rgb = (int(mean_bgr[2]), int(mean_bgr[1]), int(mean_bgr[0]))

                # depth for centre pixel
                z = float('nan')
                if depth_frame is not None:
                    try:
                        # depth value at centre (scaled into depth-image coords)
                        dcy = min(dh - 1, int(cy * d_sy))
                        dcx = min(dw - 1, int(cx * d_sx))
                        depth_val = float(depth_frame[dcy, dcx])

                        # if depth is zero, nan or inf (Gazebo publishes +inf for
                        # no-return pixels), attempt average over ROI
                        if depth_val == 0 or not math.isfinite(depth_val):
                            valid = depth_frame[int(y1 * d_sy):max(int(y2 * d_sy), int(y1 * d_sy) + 1),
                                                int(x1 * d_sx):max(int(x2 * d_sx), int(x1 * d_sx) + 1)]
                            valid = valid[np.isfinite(valid) & (valid > 0)]
                            if valid.size > 0:
                                depth_val = float(np.mean(valid))
                            else:
                                depth_val = float('nan')

                        z = depth_val

                    except Exception:
                        z = float('nan')

                # Pinhole back-projection: pixel (cx,cy) + depth z (meters) -> 3D camera frame (meters)
                # isfinite, not just !isnan: +inf depth (Gazebo no-return) back-projects
                # to inf/NaN coordinates (0 * inf = NaN) that crash consumers downstream
                if self.camera_info_received and depth_frame is not None and math.isfinite(z) and z > 0.0:
                    X = (cx - self.cx_c) * z / self.fx
                    Y = (cy - self.cy_c) * z / self.fy
                    Z = z
                    if self.output_body_frame:
                        # optical (X right, Y down, Z fwd) → ROS body (X fwd, Y left, Z up)
                        X, Y, Z = Z, -X, -Y
                    frame_mode = self.camera_frame
                else:
                    # No depth: encode pixel centre as (cx, cy) and use z=-1 sentinel.
                    # Consumers that understand this convention (e.g. fusion_evaluator) can
                    # project LiDAR centroids to image space and match by pixel distance.
                    X, Y, Z = float(cx), float(cy), -1.0
                    frame_mode = 'camera_pixel'
                centres.append((X, Y, Z))
                centre_colors.append(cone_color)   # YOLO class int — for Track msg
                centre_rgb.append(mean_rgb)         # RGB tuple — for PointCloud2 only
                centre_frames.append(frame_mode)
                area = float((x2 - x1) * (y2 - y1))
                centre_areas.append(area)

                # TODO: verify/fix this
                # Build cluster points: we will sample pixels in the bbox to avoid huge clouds
                # sample stride to limit points
                stride = 4
                ys = np.arange(y1, y2, stride)
                xs = np.arange(x1, x2, stride)
                if ys.size == 0 or xs.size == 0:
                    continue
                grid_x, grid_y = np.meshgrid(xs, ys)
                px = grid_x.ravel()
                py = grid_y.ravel()
                pz = np.zeros_like(px, dtype=np.float32)

                # get depth per sampled pixel if available (scaled into depth coords)
                if depth_frame is not None:
                    dpy = np.minimum(dh - 1, (py * d_sy).astype(int))
                    dpx = np.minimum(dw - 1, (px * d_sx).astype(int))
                    depth_vals = depth_frame[dpy, dpx].astype(np.float32)
                    depth_vals[depth_vals == 0] = np.nan
                    pz = depth_vals
                else:
                    pz[:] = np.nan

                # pack colors per point
                colors = np.tile(np.array(mean_rgb, dtype=np.uint8), (px.shape[0], 1))

                pts = np.stack([px.astype(np.float32), py.astype(np.float32), pz.astype(np.float32)], axis=1)
                all_cluster_points.append(pts)
                all_cluster_colors.append(colors)

        # centres pointcloud
        if len(centres) > 0:
            pts_centres = np.array(centres, dtype=np.float32)
            cols_centres = np.array(centre_rgb, dtype=np.uint8)   # RGB for visualisation
            centres_msg = self.make_pointcloud2(pts_centres, cols_centres)
            self.centres_point_publisher.publish(centres_msg)
        else:
            # publish empty centres cloud
            empty = PointCloud2()
            empty.header = Header()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = self.camera_frame
            empty.width = 0
            empty.height = 1
            empty.is_dense = False
            empty.is_bigendian = False
            empty.point_step = 0
            empty.row_step = 0
            empty.fields = []
            empty.data = b''
            self.centres_point_publisher.publish(empty)

        # build and publish Track message — colour comes directly from YOLO class prediction
        try:
            track_msg = Track()
            cones_list = []
            # Cones are stamped with the image CAPTURE time so stamp-aware TF
            # lookups place them where the car was when the camera saw them.
            now = capture_stamp if capture_stamp is not None else self.get_clock().now().to_msg()
            for (cx_3d, cy_3d, cz_3d), cone_color, fid in zip(centres, centre_colors, centre_frames):
                cmsg = Cone()
                cmsg.header.stamp = now
                cmsg.header.frame_id = fid   # 'camera_base' (metric) or 'camera_pixel' (no depth)
                loc = Point()
                loc.x = float(cx_3d)
                loc.y = float(cy_3d)
                loc.z = float(cz_3d)   # -1.0 sentinel means pixel-coordinate mode
                cmsg.location = loc
                cmsg.color = cone_color   # YOLO class index (BLUE=0, YELLOW=1, …)
                self.cone_publisher.publish(cmsg)
                cones_list.append(cmsg)

            track_msg.track = cones_list
            self.track_publisher.publish(track_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to build/publish Track msg: {e}")

        return

    def make_pointcloud2(self, points: np.ndarray, colors: np.ndarray) -> PointCloud2:
        """
        Build a sensor_msgs/PointCloud2 from numpy arrays.
        # points: Nx3 float32 (X, Y, Z) in camera frame, meters
        colors: Nx3 uint8 (R,G,B)
        """
        if points.shape[0] == 0:
            empty = PointCloud2()
            empty.header = Header()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = self.camera_frame
            empty.width = 0
            empty.height = 1
            empty.is_dense = False
            empty.is_bigendian = False
            empty.point_step = 0
            empty.row_step = 0
            empty.fields = []
            empty.data = b''
            return empty

        assert points.shape[0] == colors.shape[0], "points/colors must match"
        N = points.shape[0]

        # replace NaN z with 0.0 so RViz will render (or set to metric depth if available)
        z = points[:, 2].astype(np.float32)
        nan_mask = np.isnan(z)
        is_dense = not nan_mask.any()
        z[nan_mask] = 0.0

        # structured array: x,y,z (float32) and rgb
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
        arr = np.zeros(N, dtype=dtype)
        arr['x'] = points[:, 0].astype(np.float32)
        arr['y'] = points[:, 1].astype(np.float32)
        arr['z'] = z

        # pack rgb into uint32 then reinterpret as float32 
        r = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 2].astype(np.uint32)
        rgb_uint32 = (r << 16) | (g << 8) | b
        arr['rgb'] = rgb_uint32.view(np.float32)

        binary = arr.tobytes()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.camera_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # use FLOAT32 for rgb so RViz/PCL interprets packed color correctly
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = N
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = arr.dtype.itemsize
        cloud.row_step = cloud.point_step * N
        cloud.is_dense = is_dense
        cloud.data = binary

        return cloud
        

    def render_detection(self, frame, detections):
        """
        Renders bounding boxes around the objects detected on the frames for displaying purposes
        Args:
            frame (numpy.ndarray) : image in cv2-readable type from the camera
            detections (pd.DataFrame) : dataframe of info of objects detected by model
        """

        for r in detections:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls

                # render
                cv2.rectangle(frame, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
                # cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return

    def object_tracking(self) -> None:
        """
        DEPRECATED WARNING: Not used currently.

        Launch YOLO's tracker model on a continuous stream of frames coming from frame_generator().
        The .track(...) call will loop internally over frames until shutdown.

        Args:
            None
        
         Returns:
            None
        """

        for frame in self.frame_generator():

            results = self.model.track(
                source=frame,
                tracker="bytetrack.yaml",
                persist=True,
                stream=True,
                conf=self.conf,
                iou=self.iou,
                verbose=True
            )

            for result in results:
                annotated_frame = result.plot()
                cv2.imshow("YOLO11 Tracking", annotated_frame)

                if ((result.boxes and result.boxes.is_track) and 
                    self.cone_image_publisher.get_subscription_count()):
                    
                    boxes = result.boxes.xywh.cpu()
                    track_ids = result.boxes.id.int().cpu().tolist()

                    for frame_result in zip(boxes, track_ids):
                        # Publish the results through ROS subscribers and publishers
                        continue
        
def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()