#!/usr/bin/env python3
"""Lidar mount pitch/roll from gravity seen by the torso IMU (/secondary_imu) and the MID-360 IMU.

Independent of the floor, the pelvis IMU, the waist encoders and odometry: only still 1 s windows are used.
Assumes imu_in_torso is aligned with torso_link (URDF rpy 0).  scripts/check_mount_imu.py [bag_dir]
"""
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
bag = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                      'bags', 'run_2026-10-24_2052')
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/secondary_imu', '/utlidar/imu_livox_mid360', '/lowstate']))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
# 1 s windows: (torso acc mean, lidar acc mean, waist q)
win = {}
k = 0
while r.has_next():
    topic, data, t = r.read_next()
    if topic == '/secondary_imu':
        k += 1
        if k % 10: continue   # 1 kHz -> 100 Hz is plenty
    m = deserialize_message(data, get_message(types[topic]))
    b = win.setdefault(t // 10**9, {'torso': [], 'lidar': [], 'gyro': []})
    if topic == '/secondary_imu':
        b['torso'].append(list(m.accelerometer)); b['gyro'].append(list(m.gyroscope))
    elif topic == '/utlidar/imu_livox_mid360':
        a = m.linear_acceleration; b['lidar'].append([a.x, a.y, a.z])
T, L = [], []
for s, b in sorted(win.items()):
    if len(b['torso']) < 50 or len(b['lidar']) < 100: continue
    if np.linalg.norm(np.mean(b['gyro'], 0)) > 0.02 or np.std(np.array(b['torso']), 0).max() > 0.2: continue  # still only
    T.append(np.mean(b['torso'], 0)); L.append(np.mean(b['lidar'], 0))
T, L = np.array(T), np.array(L)
Tn = T / np.linalg.norm(T, axis=1, keepdims=True); Ln = L / np.linalg.norm(L, axis=1, keepdims=True)
print(f'still 1 s windows: {len(T)};  |g| torso {np.linalg.norm(T,axis=1).mean():.3f} m/s2, lidar {np.linalg.norm(L,axis=1).mean():.4f} g')
print('torso IMU gravity dir (mean):', np.round(Tn.mean(0), 4), ' lidar IMU:', np.round(Ln.mean(0), 4))
print(f'torso tilt spread across windows: {np.degrees(np.ptp(np.arcsin(Tn[:,0]))):.2f} deg (pitch-ish)')
def mount(p):  # torso <- lidar rotation, URDF rpy (pi + droll, pitch, 0)
    return R.from_euler('xyz', [np.pi + p[1], p[0], 0])
for name, p in (('urdf 2.93', [0.05112, 0]), ('doc 2.3', [np.radians(2.3), 0])):
    pred = mount(p).apply(Ln)
    err = np.degrees(np.arccos(np.clip((pred * Tn).sum(1), -1, 1)))
    print(f'{name:10s} angle between lidar-IMU gravity (mapped to torso) and torso-IMU gravity: {err.mean():.2f} deg')
fit = least_squares(lambda p: (mount(p).apply(Ln) - Tn).ravel(), [0.05, 0])
print(f'best fit: pitch {np.degrees(fit.x[0]):.2f} deg, roll offset {np.degrees(fit.x[1]):+.2f} deg, '
      f'residual {np.degrees(np.sqrt(np.mean(fit.fun**2))):.3f} deg')
