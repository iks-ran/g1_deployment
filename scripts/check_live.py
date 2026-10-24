#!/usr/bin/env python3
"""Live check of the bringup: topic rates, TF chains, and (--floor) lidar/URDF consistency.

Run while `ros2 launch g1_bringup g1.launch.py` is up against the robot (init.sh --robot) or a replayed
bag (scripts/test_bag_bringup.py does both). Exits non-zero if anything fails.

--floor: transforms /g1/cloud into odom and compares the floor height (median of the clearest of 4 sectors,
1-2.5 m around the robot) with the sole height from the URDF (ankle_roll_link - 0.035 m contact spheres).
Odom height cancels out. Robot on flat ground (walking bag run_2026-10-24_2147): expect ~0. Robot suspended
(run_2026-10-24_2052): expect about -gap.
"""
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# topic: (type, qos, minimum Hz)
TOPICS = {
    '/joint_states': (JointState, 10, 50.0),
    '/tf': (TFMessage, 10, 50.0),
    '/g1/odom': (Odometry, 10, 50.0),
    # Reliable: a best-effort reader drops whole ~440 KB clouds on one lost fragment.
    '/g1/cloud': (PointCloud2, QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE), 8.0),
    '/g1/imu': (Imu, qos_profile_sensor_data, 150.0),
}
TF_CHAINS = [('odom', 'livox_imu'), ('odom', 'left_ankle_roll_link'), ('pelvis', 'right_wrist_yaw_link'),
             ('odom', 'robot_center')]
# Robot stamps converted to host time (state_bridge clock offset): receipt here - stamp. The cloud's stamp is the
# scan START (scan = 100 ms); the IMU is stamped at measurement. 0 ms for the cloud = stamped on receipt (old bug).
STAMP_AGE_MS = {'/g1/cloud': (80.0, 200.0), '/g1/imu': (0.0, 30.0)}
# Processes whose CPU is reported: label -> (argv[0] suffix, required argument). g1.launch.py runs bridge +
# robot_state_publisher in one container; the standalone executables are for profiling the bridge alone.
# Match argv[0], not the whole cmdline: shells whose command text merely mentions these must not count.
CPU_PROCS = {'g1_container (bridge + rsp)': ('/component_container', '__node:=g1_container'),
             'state_bridge': ('g1_bringup/state_bridge', ''),
             'robot_state_publisher': ('robot_state_publisher/robot_state_publisher', '')}
SOLE_BELOW_ANKLE_ROLL = 0.035
FLOOR_TOL = 0.05


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def cpu_ticks():
    """utime + stime (clock ticks) of the bringup's processes, keyed by (pid, name)."""
    ticks = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            argv = open(f'/proc/{pid}/cmdline', 'rb').read().decode(errors='replace').split('\0')
            name = next((n for n, (exe, arg) in CPU_PROCS.items()
                         if argv[0].endswith(exe) and (not arg or any(arg in a for a in argv[1:]))), None)
            if name:
                f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
                ticks[(pid, name)] = int(f[11]) + int(f[12])
        except OSError:
            continue  # process exited
    return ticks


def floor_check(node, buf, cloud):
    t = Time.from_msg(cloud.header.stamp)
    end = time.time() + 1.0  # the TF listener is fed by this (single-threaded) node: spin while waiting
    while time.time() < end and not buf.can_transform('odom', cloud.header.frame_id, t):
        rclpy.spin_once(node, timeout_sec=0.02)
    try:
        lid = buf.lookup_transform('odom', cloud.header.frame_id, t)
        soles = [buf.lookup_transform('odom', f, t).transform.translation.z
                 for f in ('left_ankle_roll_link', 'right_ankle_roll_link')]
        base = buf.lookup_transform('odom', 'pelvis', t).transform.translation
    except Exception as e:  # noqa: BLE001
        return False, f'TF lookup for the cloud failed: {e}'
    s = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
    pts = np.c_[s['x'], s['y'], s['z']].astype(float)
    pts = pts[np.linalg.norm(pts, axis=1) > 0.3]
    tr = lid.transform.translation
    P = pts @ quat_matrix(lid.transform.rotation).T + [tr.x, tr.y, tr.z]
    sole = min(soles) - SOLE_BELOW_ANKLE_ROLL
    d = np.hypot(P[:, 0] - base.x, P[:, 1] - base.y)
    near = (d > 1.0) & (d < 2.5) & (np.abs(P[:, 2] - sole) < 0.25)
    # Clearest of 4 sectors around the robot: cluttered sectors read the floor too high (scripts/floor_sectors.py).
    bearing = np.arctan2(P[:, 1] - base.y, P[:, 0] - base.x)
    sector = ((bearing + np.pi / 4) // (np.pi / 2)).astype(int) % 4
    medians = [float(np.median(P[near & (sector == k), 2])) for k in range(4) if (near & (sector == k)).sum() >= 100]
    if not medians:
        return False, f'no floor sector with >= 100 points near the sole height {sole:.3f}'
    floor = min(medians)
    ok = abs(floor - sole) < FLOOR_TOL
    return ok, (f'floor z {floor:+.3f} (clearest of {len(medians)} sectors) vs URDF sole z {sole:+.3f} '
                f'(diff {100 * (floor - sole):+.1f} cm, tol {100 * FLOOR_TOL:.0f})')


def run(warmup=4.0, window=5.0, floor=False):
    node = Node('g1_check_live')
    buf = Buffer()
    TransformListener(buf, node)
    counts = {t: 0 for t in TOPICS}
    last = {}
    ages = {t: [] for t in STAMP_AGE_MS}

    def make_cb(topic):
        def cb(msg):
            counts[topic] += 1
            last[topic] = msg
            if topic in ages:
                ages[topic].append((node.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-6)
        return cb
    for topic, (msg_type, qos, _) in TOPICS.items():
        node.create_subscription(msg_type, topic, make_cb(topic), qos)

    def spin_for(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    spin_for(warmup)
    for t in counts:
        counts[t] = 0
    cpu0 = cpu_ticks()
    spin_for(window)
    cpu1 = cpu_ticks()

    ok = True
    hz = os.sysconf('SC_CLK_TCK')
    for (pid, name), t1 in sorted(cpu1.items(), key=lambda kv: kv[0][1]):
        if (pid, name) in cpu0:  # informational (no threshold): bridge cost on the robot vs the bag
            print(f"[CPU] {name:28s} {100 * (t1 - cpu0[(pid, name)]) / hz / window:5.1f} % of one core")
    for topic, (_, _, min_hz) in TOPICS.items():
        hz = counts[topic] / window
        ok &= hz >= min_hz
        print(f"[{'OK ' if hz >= min_hz else 'LOW'}] {topic:28s} {hz:7.1f} Hz (min {min_hz})")
    for parent, child in TF_CHAINS:
        good = buf.can_transform(parent, child, Time())
        ok &= good
        print(f"[{'OK ' if good else 'BAD'}] TF {parent} -> {child}")
    imu = last.get('/g1/imu')
    good = imu is not None and imu.header.frame_id == 'livox_imu'
    ok &= good
    print(f"[{'OK ' if good else 'BAD'}] /g1/imu frame_id {imu.header.frame_id if imu else None!r}")
    for topic, (lo, hi) in STAMP_AGE_MS.items():
        med = float(np.median(ages[topic])) if ages[topic] else float('nan')
        good = lo <= med <= hi
        ok &= good
        print(f"[{'OK ' if good else 'BAD'}] {topic} receipt - stamp median {med:6.1f} ms (expect {lo}..{hi})")
    if floor:
        good, text = floor_check(node, buf, last['/g1/cloud']) if '/g1/cloud' in last else (False, 'no cloud')
        ok &= good
        print(f"[{'OK ' if good else 'BAD'}] lidar floor vs URDF: {text}")
    node.destroy_node()
    return ok


def main():
    rclpy.init()
    ok = run(floor='--floor' in sys.argv)
    rclpy.shutdown()
    print('live check:', 'passed' if ok else 'FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
