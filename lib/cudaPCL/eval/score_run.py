"""Scores one mission run: time to finish (wall + sim), final x, max |y|, and how close the car got to each track cone.

  score_run.py <timeout_s> <track>     (start BEFORE the mission is set to DRIVING; ground truth is used for scoring only)
"""
import rclpy, numpy as np, sys, time, os
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mfe_msgs.msg import Track
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
class N(Node):
    def odom(s,m):
        s.simt=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        s.p.append((m.pose.pose.position.x,m.pose.pose.position.y))
        if s.t_move is None and len(s.p)>1 and abs(s.p[-1][0]-s.p[0][0])>0.5: s.t_move=(time.time(),s.simt)

    def __init__(s):
        super().__init__('hits'); s.cones=None; s.p=[]; s.state=''; s.simt=0.0; s.t_start=None; s.t_move=None; s.t_fin=None
        s.create_subscription(Track,'/ground_truth/track',lambda m:setattr(s,'cones',np.array([[c.location.x,c.location.y] for c in m.track])),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        s.create_subscription(Odometry,'/ground_truth/state_odom',s.odom,100)
        s.create_subscription(String,'/ros_can/state_str',lambda m:setattr(s,'state',m.data),10)
TRACK=sys.argv[2]   # track csv name: acceleration | skidpad | peanut
rclpy.init(); n=N(); t=time.time()
started=False
while time.time()-t<float(sys.argv[1]):
    rclpy.spin_once(n,timeout_sec=0.05)
    if "DRIVING" in n.state and n.t_start is None: n.t_start=(time.time(),n.simt)
    if "FINISHED" in n.state and n.t_fin is None: n.t_fin=(time.time(),n.simt)
    started = started or "DRIVING" in n.state
    if "FINISHED" in n.state or (started and "OFF" in n.state): break
p=np.array(n.p); print("state:",n.state,"| samples",len(p))
if n.t_fin and n.t_move: print("TIME to finish: %.1f s wall-clock, %.1f s SIM time (from first movement); DRIVING->FINISHED %.1f s wall / %.1f s sim"%(n.t_fin[0]-n.t_move[0],n.t_fin[1]-n.t_move[1],n.t_fin[0]-n.t_start[0],n.t_fin[1]-n.t_start[1]))
else: print("did not finish; moved:",bool(n.t_move),"sim time since first movement: %s"%("%.1f s"%(n.simt-n.t_move[1]) if n.t_move else "n/a"))
print("final x %.1f m, max |y| %.2f m"%(p[-1,0],np.abs(p[:,1]).max()))
import csv
_EUFS_MASTER=os.environ.get("EUFS_MASTER", os.path.expanduser("~/Develop/MFE26-eufs-sim"))
allrows=list(csv.DictReader(open("%s/eufs_sim/eufs_tracks/csv/%s.csv"%(_EUFS_MASTER,TRACK))))
rows=[r for r in allrows if r["tag"] in ("blue","yellow","orange","big_orange")]
cs=[r for r in allrows if r["tag"]=="car_start"][0]; X0,Y0,YAW=float(cs["x"]),float(cs["y"]),float(cs["direction"])
cones=np.array([[float(r["x"]),float(r["y"])] for r in rows]); tags=[r["tag"] for r in rows]
# odom origin = car start (0,-14.4) yaw 90deg -> map
c,sn=np.cos(YAW),np.sin(YAW); P=np.c_[X0+c*p[:,0]-sn*p[:,1], Y0+sn*p[:,0]+c*p[:,1]]
d=np.linalg.norm(P[:,None]-cones[None],axis=2); m=d.min(0)
print("closest car-centre approach to any cone: %.2f m"%d.min())
for th in (0.3,0.5,0.8,1.0,1.2): print("  cones within %.1f m of car centre at some point: %d of %d"%(th,(m<th).sum(),len(cones)))
print("  path extent map x %.1f..%.1f y %.1f..%.1f; cones nearest-approach by colour:"%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max()), {t:round(float(min(m[i] for i in range(len(tags)) if tags[i]==t)),2) for t in set(tags)})
import os
os.makedirs(os.path.expanduser("~/mfe_logs"),exist_ok=True)
np.save(os.path.expanduser("~/mfe_logs/path_%s.npy"%TRACK),P)
