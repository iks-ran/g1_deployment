#!/usr/bin/env python3
"""Offline test of the C++ ClockOffset (g1_bringup/clock_offset.hpp) on recorded bags (no DDS): the (bag receive
time, robot stamp) pairs of the MID-360 IMU and cloud are piped, in receive order, through the `clock_offset_check`
binary, which makes the same calls as StateBridge::on_imu / on_cloud. Then checks the converted stamps:
  - IMU: receive - converted stamp = latency >= ~0 and small (p1 > -1 ms, median < 5 ms);
  - cloud: receive - converted stamp = scan duration + latency (median 90..130 ms), and the last point of the
    scan (stamp + max per-point `time`) is not after its receipt (> -10 ms: the point timing and the IMU stamps
    disagree by a few ms, measured down to -4.5 ms; this guards the stamp convention, which is a ~100 ms effect).
  scripts/test_clock_offset.py [bag_dir ...]   (default: every bag under bags/)
"""
import glob
import os
import subprocess
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from ament_index_python.packages import get_package_prefix

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU, CLOUD = '/utlidar/imu_livox_mid360', '/utlidar/cloud_livox_mid360'


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def check(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[IMU, CLOUD]))
    rows, t_max = [], {}  # (kind, receive_ns, stamp_ns); row index -> max per-point time (sampled clouds)
    n_cloud = 0
    while r.has_next():
        topic, data, receive_ns = r.read_next()
        if topic == IMU:
            rows.append(('i', receive_ns, stamp_ns(deserialize_message(data, Imu))))
        else:
            msg = deserialize_message(data, PointCloud2)
            if n_cloud % 50 == 0:  # per-point times are slow to read: sample
                t_max[len(rows)] = float(point_cloud2.read_points(msg, field_names=('time',))['time'].max())
            n_cloud += 1
            rows.append(('c', receive_ns, stamp_ns(msg)))
    exe = os.path.join(get_package_prefix('g1_bringup'), 'lib', 'g1_bringup', 'clock_offset_check')
    out = subprocess.run([exe], input=''.join(f'{k} {rc} {st}\n' for k, rc, st in rows), capture_output=True,
                         text=True, check=True).stdout.split()
    assert len(out) == len(rows), f'clock_offset_check printed {len(out)} lines for {len(rows)} inputs'
    imu_lat, cloud_lat, scan_end = [], [], []
    for i, ((kind, receive_ns, _), host) in enumerate(zip(rows, out)):
        if host == 'none':
            continue  # cloud before the first IMU: dropped by the bridge too
        lat = receive_ns - int(host)
        (imu_lat if kind == 'i' else cloud_lat).append(lat)
        if i in t_max:
            scan_end.append(lat - t_max[i])
    offset_s = (rows[-1][1] - rows[-1][2] - imu_lat[-1]) * 1e-9 if rows[-1][0] == 'i' else float('nan')
    imu_lat, cloud_lat, scan_end = (np.array(v) * 1e-6 for v in (imu_lat, cloud_lat, scan_end))
    ok = (np.percentile(imu_lat, 1) > -1.0 and np.median(imu_lat) < 5.0 and 90 <= np.median(cloud_lat) <= 130
          and scan_end.min() > -10.0)
    print(f"[{'OK ' if ok else 'BAD'}] {os.path.basename(bag)}: IMU latency p1/median/p99 "
          f'{np.percentile(imu_lat, 1):+.2f}/{np.median(imu_lat):.2f}/{np.percentile(imu_lat, 99):.2f} ms; '
          f'cloud receive - stamp median {np.median(cloud_lat):.1f} ms; scan end -> receipt min {scan_end.min():+.1f} ms; '
          f'offset {offset_s:+.3f} s')
    return ok


def main():
    bags = sys.argv[1:] or sorted(glob.glob(os.path.join(WS, 'bags', '*', 'metadata.yaml')))
    bags = [os.path.dirname(b) if b.endswith('metadata.yaml') else b for b in bags]
    if not bags:
        print('clock offset test: skipped (no bags)')
        return 0
    ok = all([check(b) for b in bags])
    print(f"clock offset test: {'passed' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
