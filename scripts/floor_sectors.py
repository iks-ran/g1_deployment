#!/usr/bin/env python3
"""Lidar floor height minus the lowest foot contact point (URDF contact spheres), per sector x distance.

The lowest foot is on the floor while walking, so ~0 means lidar extrinsics + leg FK agree. A rigid lidar pose error
shows as a plane (offset/tilt); sectors that rise with distance in every direction are the environment.
  scripts/floor_sectors.py src/g1_description/urdf/g1_29dof_rev_1_0.urdf <bag_dir>
"""
import sys, xml.etree.ElementTree as ET, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation as R
urdf, bag = sys.argv[1], sys.argv[2]
URDF_P, URDF_Z = 0.05112069379091391, 0.428434
VARIANTS = {'urdf': (URDF_P, URDF_Z)}
BINS = {}
names = [f'{s}_{j}_joint' for s in ('left', 'right') for j in ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')]
names += ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']
joints = {}
for j in ET.parse(urdf).getroot().findall('joint'):
    o = j.find('origin'); ax = j.find('axis')
    if o is None: o = ET.Element('origin', xyz='0 0 0')
    joints[j.find('child').get('link')] = [j.get('name'), j.find('parent').get('link'),
        np.array([float(v) for v in o.get('xyz').split()]), [float(v) for v in o.get('rpy', '0 0 0').split()],
        np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else None]
SPHERES = np.array([[-0.05, 0.025, -0.03], [-0.05, -0.025, -0.03], [0.12, 0.03, -0.03], [0.12, -0.03, -0.03]])
def T(link, q):
    M = np.eye(4)
    while link in joints:
        name, parent, xyz, rpy, ax = joints[link]
        A = np.eye(4); A[:3, :3] = R.from_euler('xyz', rpy).as_matrix(); A[:3, 3] = xyz
        if ax is not None:
            B = np.eye(4); B[:3, :3] = R.from_rotvec(ax * q.get(name, 0.0)).as_matrix(); A = A @ B
        M = A @ M; link = parent
    return M
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/lowstate', '/odommodestate', '/utlidar/cloud_livox_mid360']))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
q = odom = None; res = {k: [] for k in VARIANTS}; speeds = []; footgap = []
while r.has_next():
    topic, data, t = r.read_next()
    m = deserialize_message(data, get_message(types[topic]))
    if topic == '/lowstate':
        q = {names[i]: m.motor_state[i].q for i in range(15)}
    elif topic == '/odommodestate':
        w, x, y, z = m.imu_state.quaternion
        odom = (np.array(m.position), R.from_quat([x, y, z, w]), np.linalg.norm(m.velocity[:2]))
    elif q and odom:
        Ro = odom[1].as_matrix()
        feet = []
        for f in ('left_ankle_roll_link', 'right_ankle_roll_link'):
            F = T(f, q); feet.append(((Ro @ (F[:3, :3] @ SPHERES.T + F[:3, 3:4])).T)[:, 2].min() - 0.005)
        sole = min(feet)
        footgap.append(abs(feet[0] - feet[1])); speeds.append(odom[2])
        s = point_cloud2.read_points(m, field_names=('x', 'y', 'z'))
        pts = np.c_[s['x'], s['y'], s['z']].astype(float)
        pts = pts[np.linalg.norm(pts, axis=1) > 0.3]
        H = R.from_euler('z', odom[1].as_euler('ZYX')[0]).inv()
        for k, (p, z) in VARIANTS.items():
            joints['mid360_link'][3] = [np.pi, p, 0.0]; joints['mid360_link'][2] = np.array([0.0002835, 0.00003, z])
            L = T('mid360_link', q)
            P = H.apply((Ro @ ((L[:3, :3] @ pts.T).T + L[:3, 3]).T).T)
            d = np.linalg.norm(P[:, :2], axis=1)
            fl = P[(d > 1.0) & (d < 3.5) & (np.abs(P[:, 2] - sole) < 0.15)]
            if len(fl) < 300: continue
            dfl = np.linalg.norm(fl[:, :2], axis=1); bear = np.degrees(np.arctan2(fl[:, 1], fl[:, 0]))
            sectors = {'front': np.abs(bear) < 45, 'left': (bear >= 45) & (bear < 135), 'right': (bear >= -135) & (bear < -45), 'back': np.abs(bear) >= 135}
            for nm, sm in sectors.items():
                for lo, hi in ((1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.5)):
                    sel = sm & (dfl >= lo) & (dfl < hi)
                    if sel.sum() > 30: BINS.setdefault(f'{nm:5s} {lo}-{hi} m', []).append(np.median(fl[sel, 2]) - sole)
            for nm, (a, b) in ():
                sel = (bear >= a) & (bear < b)
                if sel.sum() > 30: BINS.setdefault(f'bearing {nm}', []).append(np.median(fl[sel, 2]) - sole)

            c, *_ = np.linalg.lstsq(np.c_[fl[:, :2], np.ones(len(fl))], fl[:, 2], rcond=None)
            res[k].append((np.degrees(np.arctan(c[0])), np.degrees(np.arctan(c[1])), np.median(fl[:, 2]) - sole))
speeds = np.array(speeds); footgap = np.array(footgap)
print(f'clouds {len(speeds)}, moving (>0.1 m/s) {np.mean(speeds > 0.1)*100:.0f}%, max speed {speeds.max():.2f} m/s, '
      f'feet height diff max {footgap.max()*100:.1f} cm')
for k, v in res.items():
    a = np.array(v)
    print(f'{k:26s} n={len(a)}  tilt fwd {a[:,0].mean():+.2f}±{a[:,0].std():.2f}  left {a[:,1].mean():+.2f}±{a[:,1].std():.2f} deg   '
          f'floor - lowest foot {a[:,2].mean()*100:+.1f} cm (median {np.median(a[:,2])*100:+.1f}, std {a[:,2].std()*100:.1f})')

for k, v in BINS.items():
    print(f'{k:16s} n={len(v):4d}  floor - lowest foot median {np.median(v)*100:+.1f} cm, IQR {np.subtract(*np.percentile(v,[75,25]))*100:.1f}')
