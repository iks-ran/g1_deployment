#!/usr/bin/env python3
"""Lidar mount rotation from gyros while the robot moves: torso IMU (/secondary_imu) and MID-360 IMU sit on the
same rigid body, so w_torso = R w_lidar. Full roll/pitch/yaw, independent of floor, waist encoders and odometry.
Assumes imu_in_torso is aligned with torso_link (URDF rpy 0). Needs rotation: use a walking bag.
  scripts/check_mount_gyro.py <bag_dir>
"""
import sys, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
bag = sys.argv[1]  # noqa
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/secondary_imu', '/utlidar/imu_livox_mid360']))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
tt, wt, at, tl, wl, al = [], [], [], [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    m = deserialize_message(data, get_message(types[topic]))
    if topic == '/secondary_imu':
        tt.append(t * 1e-9); wt.append(list(m.gyroscope)); at.append(list(m.accelerometer))
    else:
        tl.append(t * 1e-9); wl.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        al.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
tt, wt, at, tl, wl, al = map(np.array, (tt, wt, at, tl, wl, al))
t0 = max(tt[0], tl[0]) + 1; t1 = min(tt[-1], tl[-1]) - 1
grid = np.arange(t0, t1, 0.005)
def res(t, v, g): return np.stack([np.interp(g, t, v[:, i]) for i in range(3)], 1)
WL = res(tl, wl, grid); AL = res(tl, al, grid) * 9.80665
nominal = R.from_euler('xyz', [np.pi, 0.05112069379091391, 0])
best = None
for lag in np.arange(-0.1, 0.1001, 0.0025):
    WT = res(tt, wt, grid + lag)
    c = np.mean(np.sum(WT * nominal.apply(WL), 1))
    if best is None or c > best[0]: best = (c, lag)
lag = best[1]
WT = res(tt, wt, grid + lag); AT = res(tt, at, grid + lag)
mov = np.linalg.norm(WT, axis=1) > 0.15
print(f'{len(grid)} samples, rotating >0.15 rad/s: {mov.sum()} ({mov.mean()*100:.0f}%), |w| max {np.linalg.norm(WT,axis=1).max():.2f} rad/s; best lag {lag*1000:+.1f} ms')
# gyro bias: estimate from still samples of each
still = np.linalg.norm(WT, axis=1) < 0.03
bt, bl = WT[still].mean(0), WL[still].mean(0)
print(f'still samples {still.sum()}; gyro bias torso {np.round(bt, 4)}, lidar {np.round(bl, 4)}')
rot, rssd = R.align_vectors(WT[mov] - bt, WL[mov] - bl)
rpy = rot.as_euler('xyz')
print(f'gyro fit  torso<-lidar rpy: roll {np.degrees(rpy[0]):+.3f}  pitch {np.degrees(rpy[1]):+.3f}  yaw {np.degrees(rpy[2]):+.3f} deg; '
      f'rms residual {np.sqrt(rssd**2/mov.sum()):.4f} rad/s')
for name, ref in (('urdf', nominal), ('doc 2.3', R.from_euler('xyz', [np.pi, np.radians(2.3), 0]))):
    e = (ref.inv() * rot).magnitude()
    print(f'  angle to {name}: {np.degrees(e):.3f} deg')
# bootstrap over 5 s chunks for an error bar
idx = np.where(mov)[0]; chunks = np.array_split(idx, 12); ps = []
for _ in range(200):
    sel = np.concatenate([chunks[i] for i in np.random.randint(0, len(chunks), len(chunks))])
    ps.append((nominal.inv() * R.align_vectors(WT[sel] - bt, WL[sel] - bl)[0]).as_rotvec())  # small: no wrap
ps = np.degrees(np.array(ps))
print(f'  bootstrap std about lidar x/y/z: {ps[:,0].std():.3f} {ps[:,1].std():.3f} {ps[:,2].std():.3f} deg')
