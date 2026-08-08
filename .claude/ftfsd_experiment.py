#!/usr/bin/env python3
"""Offline: can ft-fsd trackdrive produce a full-loop path from the complete peanut map?"""
import csv
import numpy as np
from fsd_path_planning import PathPlanner, ConeTypes, MissionTypes

cones = {'blue': [], 'yellow': []}
with open('/root/Develop/MFE26-eufs-sim/eufs_tracks/csv/peanut.csv') as f:
    for row in csv.DictReader(f):
        if row['tag'] in cones:
            cones[row['tag']].append((float(row['x']), float(row['y'])))

blue = np.array(cones['blue'])
yellow = np.array(cones['yellow'])

cones_by_type = [np.zeros((0, 2)) for _ in ConeTypes]
cones_by_type[ConeTypes.LEFT] = blue      # blue = left
cones_by_type[ConeTypes.RIGHT] = yellow   # yellow = right

car_pos = np.array([0.0, 0.0])
car_dir = np.array([1.0, 0.0])

planner = PathPlanner(MissionTypes.trackdrive)
result = planner.calculate_path_in_global_frame(
    cones_by_type, car_pos, car_dir, return_intermediate_results=True)
path, sorted_left, sorted_right, *rest = result

p = np.array(path)[:, 1:3]
print(f"path points: {len(p)}")
print(f"path arc length: {np.sum(np.hypot(np.diff(p[:,0]), np.diff(p[:,1]))):.1f} m")
print(f"path start: ({p[0,0]:.2f},{p[0,1]:.2f})  end: ({p[-1,0]:.2f},{p[-1,1]:.2f})")
print(f"start-end gap: {np.hypot(p[0,0]-p[-1,0], p[0,1]-p[-1,1]):.2f} m")
print(f"sorted_left: {len(sorted_left)} of {len(blue)} blue")
print(f"sorted_right: {len(sorted_right)} of {len(yellow)} yellow")
print(f"path X range [{p[:,0].min():.1f},{p[:,0].max():.1f}]  Y [{p[:,1].min():.1f},{p[:,1].max():.1f}]")
# Sample the path every ~10 points to eyeball the route
for i in range(0, len(p), max(1, len(p)//15)):
    print(f"  [{i:3d}] ({p[i,0]:7.2f}, {p[i,1]:7.2f})")
