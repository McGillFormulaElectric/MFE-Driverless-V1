# Dump /lidar/points_raw frames from a rosbag2 to raw float32 N x 4 (x,y,z,intensity) files.
import sys, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

bag, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
n = 0
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic != '/lidar/points_raw':
        continue
    msg = deserialize_message(data, PointCloud2)
    fields = {f.name for f in msg.fields}
    names = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in fields else [])
    pts = point_cloud2.read_points(msg, field_names=names, skip_nans=False)
    arr = np.zeros((len(pts), 4), dtype=np.float32)
    for i, nm in enumerate(['x', 'y', 'z']):
        arr[:, i] = pts[nm]
    if 'intensity' in names:
        arr[:, 3] = pts['intensity']
    arr.tofile(os.path.join(out, f'{n:04d}.f32'))
    n += 1
print(f'wrote {n} frames to {out}', flush=True)
