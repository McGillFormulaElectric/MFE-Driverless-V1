#!/usr/bin/env python3

import threading
import cv2
import torch
import numpy as np
import queue

from typing import Generator

import rclpy
from rclpy.node import Node
from ultralytics import YOLO

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from mfe_msgs.msg import Cone, Track

class ConeDetectionNode(Node):

    def __init__(self):
        super().__init__("camera_cone_node")

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Declare and get model path parameter
        self.declare_parameter("model_path", "yolov8n.pt")  # default to yolov8n.pt it none found
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

        self.declare_parameter("depth_callback", False)

        # subscribers
        self.create_subscription(Image, "image/raw", self.callback, qos_profile)

        if self.get_parameter("depth_callback").value:
            self.create_subscription(Image, "image/depth", self.depth_callback, qos_profile) # exists if depth camera provides it 

        # publisher
        self.cone_image_publisher = self.create_publisher(Image, "image/cones_image", qos_profile) 
        self.cone_publisher = self.create_publisher(Cone, "image/cones", qos_profile) 
        self.track_publisher = self.create_publisher(Track, "image/track", qos_profile) 
        self.centres_point_publisher = self.create_publisher(PointCloud2, "image/cone_centres", qos_profile) # the cone locations (can be visualized in rviz)

        self.bridge = CvBridge()

        # store latest depth frame (if any)
        self.latest_depth = None

        # start processing thread so detections run
        self.get_logger().info("Starting object prediction thread")
        self.processing_t = threading.Thread(target=self.object_predicting, daemon=True)
        self.processing_t.start()

        # UNUSED: Start tracker thread
        # self.object_tracking_t = threading.Thread(target=self.object_predicting, daemon=True)
        # self.object_tracking_t.start()

    def callback(self, msg: Image) -> None:
        """
        ROS2 subscription callback. Convert the ROS Image to OpenCV (numpy) and enqueue it.
        If the queue is full, drop the oldest frame to make room (so we stay 'real-time').
        
        Args:
            msg: sensor_msgs.msg.Image
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        try:
            self.frame_queue.put(frame, timeout=3.0)
            self.get_logger().debug(f"Enqueued frame, queue size={self.frame_queue.qsize()}")
        except queue.Full:
            self.frame_queue.get_nowait()
            self.frame_queue.put_nowait(frame)
            self.get_logger().debug("Frame queue full; dropped oldest frame and enqueued new one")

        return
    
    def depth_callback(self, msg: Image) -> None:
        """
        ROS2 Subscription callback. Applies the concurrent rgbd object bounding box onto a integer
        depth mask. Obtains the average over the bounding box, and calculates the depth of
        objects.
        """

        try:
            # attempt to convert depth into float32 meters
            depth = self.bridge.imgmsg_to_cv2(msg)
        except Exception:
            self.get_logger().error("Failed to convert depth image")
            return

        # If depth is uint16 (common for depth sensors), convert to meters if required.
        if depth.dtype == np.uint16:
            # no intrinsic scaling info available here, keep raw integers
            depth = depth.astype(np.float32)
        elif depth.dtype == np.float32:
            depth = depth
        else:
            depth = depth.astype(np.float32)

        self.latest_depth = depth
        return
    

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
                frame = self.frame_queue.get()   # blocking get 
            except queue.Empty:
                continue
            if frame is None:
                break

            yield frame

    def object_predicting(self) -> None:
        """
        Launches YOLO's prediction model on a continuous stream of frames coming 
        from frame_generator().
        """

        self.get_logger().info("object_predicting thread started")
    
        for frame in self.frame_generator():
            self.get_logger().debug("Pulled frame from generator")

            # run object detection
            detections = self.model(frame)

            annotated_frame = detections[0].plot()

            # render bounding boxes
            self.render_detection(frame, detections)

            # process detections: compute centres, colours, point clouds, and publish
            try:
                self.process_and_publish_detections(detections, frame, self.latest_depth)
            except Exception as e:
                self.get_logger().error(f"Error processing detections: {e}")

            # publish annotated image
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.cone_image_publisher.publish(image_msg)

        return
    

    def process_and_publish_detections(self, detections, frame: np.ndarray, depth_frame: np.ndarray):
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

        centres = []  # list of (x,y,z)
        centre_colors = []  # list of rgb uint8 triples
        centre_areas = []  # area of bbox for heuristic color sizing

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

                # avg colour inside bbox (BGR -> convert to RGB)
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                mean_bgr = cv2.mean(roi)[:3]
                mean_rgb = (int(mean_bgr[2]), int(mean_bgr[1]), int(mean_bgr[0]))

                # depth for centre pixel
                z = float('nan')
                if depth_frame is not None:
                    try:
                        # depth value at centre
                        depth_val = float(depth_frame[cy, cx])

                        # if depth is zero or nan, attempt average over ROI
                        if depth_val == 0 or np.isnan(depth_val):
                            valid = depth_frame[y1:y2, x1:x2]
                            valid = valid[valid > 0]
                            if valid.size > 0:
                                depth_val = float(np.mean(valid))
                            else:
                                depth_val = float('nan')

                        z = depth_val

                    except Exception:
                        z = float('nan')

                centres.append((float(cx), float(cy), z))
                centre_colors.append(mean_rgb)
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

                # get depth per sampled pixel if available
                if depth_frame is not None:
                    depth_vals = depth_frame[py, px].astype(np.float32)
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
            cols_centres = np.array(centre_colors, dtype=np.uint8)
            centres_msg = self.make_pointcloud2(pts_centres, cols_centres, frame_id='camera')
            self.centres_point_publisher.publish(centres_msg)
        else:
            # publish empty centres cloud
            empty = PointCloud2()
            empty.header = Header()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = 'camera'
            empty.width = 0
            empty.height = 1
            empty.is_dense = False
            empty.is_bigendian = False
            empty.point_step = 0
            empty.row_step = 0
            empty.fields = []
            empty.data = b''
            self.centres_point_publisher.publish(empty)

        # build and publish Track message containing Cone[] using colour heuristic
        try:
            track_msg = Track()
            cones_list = []
            now = self.get_clock().now().to_msg()
            for (cx, cy, cz), rgb, area in zip(centres, centre_colors, centre_areas):
                cmsg = Cone()
                # header
                cmsg.header.stamp = now
                cmsg.header.frame_id = 'camera'
                # location uses geometry_msgs/Point
                loc = Point()
                loc.x = float(cx)
                loc.y = float(cy)
                loc.z = float(cz) if not np.isnan(cz) else 0.0
                cmsg.location = loc

                # simple colour heuristic
                r, g, b = rgb
                color_enum = Cone.UNKNOWN
                # Yellow: both R and G high and B low
                if r > 150 and g > 150 and b < 120:
                    color_enum = Cone.YELLOW
                # Blue dominant
                elif b > r and b > g:
                    color_enum = Cone.BLUE
                # Orange (red dominant) - use area to decide big vs small
                elif r > g and r > b:
                    if area > 3000:
                        color_enum = Cone.ORANGE_BIG
                    else:
                        color_enum = Cone.ORANGE_SMALL
                else:
                    color_enum = Cone.UNKNOWN

                cmsg.color = int(color_enum)
                self.cone_publisher.publish(cmsg)
                cones_list.append(cmsg)

            track_msg.track = cones_list
            self.track_publisher.publish(track_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to build/publish Track msg: {e}")

        return
    
    def make_pointcloud2(self, points: np.ndarray, colors: np.ndarray, frame_id: str = 'camera') -> PointCloud2:
        """
        Build a sensor_msgs/PointCloud2 from numpy arrays.
        points: Nx3 float32 (x, y, z) - here x,y are pixel coordinates, z is depth in meters (or nan)
        colors: Nx3 uint8 (R,G,B)
        """
        if points.shape[0] == 0:
            empty = PointCloud2()
            empty.header = Header()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = frame_id
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
        header.frame_id = frame_id

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