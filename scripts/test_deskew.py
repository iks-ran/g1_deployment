#!/usr/bin/env python3
"""Offline test of the C++ deskew (g1_bringup/deskew.hpp) on a recorded bag (no DDS): the MID-360 IMU samples and
clouds are piped, in receive order, through the `deskew_check` binary (same deskew_to_end() call as
StateBridge::try_deskew), with a translation track from the bag's /odommodestate (pelvis position, 5 ms samples;
plausible motion for the exactness check; the TF-based lidar-origin track of the bridge is checked in the bag gate).
Checks:
  1. every cloud after the first IMU second is deskewed; intensity is passed through;
  2. the output (x y z intensity, scan-END frame) matches a NumPy reference that sees the same gyro samples (those
     received before the cloud): max point difference < 0.5 mm;
  3. on turning scans (|w| > 0.4 rad/s) the scan's late half (50-100 ms) lies closer to its early half: median
     nearest-neighbour distance late -> early drops by >= 10 % (the raw cloud is not motion-compensated). This
     proxy sees rotation, not translation (scripts/sim_deskew_bench.py has ground truth). Still scans: no worse.
  scripts/test_deskew.py [bag_dir]   (default: the walking bag)
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, PointCloud2

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU, CLOUD = '/utlidar/imu_livox_mid360', '/utlidar/cloud_livox_mid360'
DT = np.dtype({'names': ['x', 'y', 'z', 'i', 'time'], 'formats': ['<f4'] * 5, 'offsets': [0, 4, 8, 12, 18],
               'itemsize': 22})
OUT = np.dtype({'names': ['x', 'y', 'z', 'i'], 'formats': ['<f4'] * 4})  # /g1/cloud_deskewed layout
TRANS_STEP = 0.005
GRID = 1e-4  # same integration grid as deskew.hpp
MAX_DIFF = 0.0005
TURN = 0.4


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def grid_index(t, n):
    return np.minimum(n - 1, np.floor(t / GRID + 0.5).astype(int))  # std::round (halves away from 0), not np.round


def reference(p, tp, t0, ti, wi, trans=None):
    """deskew.hpp in NumPy: midpoint gyro rate on a 0.1 ms grid (linear interpolation, held beyond the ends);
    points into the scan-END frame; trans = (n, 3) translation samples every TRANS_STEP from the scan start."""
    n = int(tp.max() / GRID) + 2
    tm = t0 + ((np.arange(1, n) - 0.5) * GRID * 1e9).astype(np.int64)
    w = np.stack([np.interp(tm, ti, wi[:, i]) for i in range(3)], 1)
    rots = [R.identity()]
    for k in range(n - 1):
        rots.append(rots[-1] * R.from_rotvec(w[k] * GRID))
    end_inv = rots[int(grid_index(np.array([tp.max()]), n)[0])].inv()
    k = grid_index(tp, n)
    out = np.empty_like(p)
    for j in np.unique(k):
        out[k == j] = (end_inv * rots[j]).apply(p[k == j])
    if trans is not None:
        x = np.maximum(0.0, tp / TRANS_STEP)
        i = np.minimum(len(trans) - 1, x.astype(int))
        nxt = np.minimum(len(trans) - 1, i + 1)
        f = np.where(i + 1 >= len(trans), 0.0, x - i)[:, None]
        out += trans[i] + f * (trans[nxt] - trans[i])
    return out, float(np.linalg.norm(w.mean(0)))


def pelvis_track(bag):
    """/odommodestate (receipt time, position, orientation) and the robot->host offset of the lidar IMU."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/odommodestate', IMU]))
    od, off = [], []
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        if topic == IMU:
            off.append(t - stamp_ns(m))
        elif np.linalg.norm(m.imu_state.quaternion) > 0.5:
            q = m.imu_state.quaternion
            od.append((t, *m.position, q[1], q[2], q[3], q[0]))
    return np.array(od), int(np.min(off))


def translation(od, t_start_host, span):
    """Pelvis displacement every TRANS_STEP over the scan, in the scan-end pelvis frame, relative to the end."""
    ts = t_start_host + (np.arange(int(span / TRANS_STEP) + 2) * TRANS_STEP * 1e9).astype(np.int64)
    end = t_start_host + int(span * 1e9)
    pos = np.stack([np.interp(ts, od[:, 0], od[:, 1 + i]) for i in range(3)], 1)
    pend = np.array([np.interp(end, od[:, 0], od[:, 1 + i]) for i in range(3)])
    rend = R.from_quat(od[np.searchsorted(od[:, 0], end).clip(0, len(od) - 1), 4:8])
    return rend.inv().apply(pos - pend)


def half_nn(p, tp):
    n = np.linalg.norm(p, axis=1)
    keep = (n > 1.0) & (n < 12)
    p, tp = p[keep], tp[keep]
    d, _ = cKDTree(p[tp < 0.05]).query(p[tp >= 0.05], distance_upper_bound=0.5)
    return np.median(d[np.isfinite(d)])


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else os.path.join(WS, 'bags', 'run_2026-10-24_2147')
    if not os.path.isdir(bag):
        print(f'deskew test: skipped (no bag at {bag})')
        return 0
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[IMU, CLOUD]))
    od, offset = pelvis_track(bag)
    tmp = tempfile.mkdtemp(prefix='g1_deskew_test_')
    lines, clouds, gyro = [], [], []  # gyro: what the bridge has seen so far (monotonic stamps, as GyroBuffer)
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == IMU:
            m = deserialize_message(data, Imu)
            w = m.angular_velocity
            lines.append(f'i {stamp_ns(m)} {w.x!r} {w.y!r} {w.z!r}\n')
            if not gyro or stamp_ns(m) > gyro[-1][0]:
                gyro.append((stamp_ns(m), w.x, w.y, w.z))
            continue
        m = deserialize_message(data, PointCloud2)
        assert m.point_step == 22 and [f.name for f in m.fields] == ['x', 'y', 'z', 'intensity', 'ring', 'time']
        src = os.path.join(tmp, f'{len(clouds)}.bin')
        open(src, 'wb').write(bytes(m.data))
        span = float(np.frombuffer(bytes(m.data), DT)['time'].max()) * 1e-9
        tr = translation(od, stamp_ns(m) + offset, span)
        lines.append(f'c {stamp_ns(m)} {src} {src}.out {TRANS_STEP!r} {len(tr)} '
                     + ' '.join(f'{v!r}' for v in tr.ravel()) + '\n')
        clouds.append((stamp_ns(m), src, np.array(gyro[-300:]), tr))  # 1.5 s of gyro >= GyroBuffer's 1 s
    exe = os.path.join(get_package_prefix('g1_bringup'), 'lib', 'g1_bringup', 'deskew_check')
    out = subprocess.run([exe], input=''.join(lines), capture_output=True, text=True, check=True).stdout
    status = out.splitlines()
    assert len(status) == len(clouds), f'deskew_check printed {len(status)} lines for {len(clouds)} clouds'

    t_first_imu = min(int(ln.split()[1]) for ln in lines if ln[0] == 'i')
    expected = [i for i, (t0, _, _, _) in enumerate(clouds) if t0 > t_first_imu + 1_000_000_000]
    missed = [i for i in expected if status[i] != 'ok']
    max_diff, rows, intensity_ok = 0.0, [], True
    for i in expected:
        if status[i] != 'ok':
            continue
        t0, src, g, tr = clouds[i]
        a = np.frombuffer(open(src, 'rb').read(), DT)
        b = np.frombuffer(open(src + '.out', 'rb').read(), OUT)
        p = np.c_[a['x'], a['y'], a['z']].astype(float)
        q = np.c_[b['x'], b['y'], b['z']].astype(float)
        intensity_ok &= len(b) == len(a) and np.array_equal(a['i'], b['i'], equal_nan=True)
        tp = a['time'].astype(float) * 1e-9
        ref, wmag = reference(p, tp, t0, g[:, 0], g[:, 1:], tr)
        ok = np.isfinite(p).all(1)
        max_diff = max(max_diff, float(np.abs(q[ok] - ref[ok]).max()))
        rows.append((wmag, half_nn(p[ok], tp[ok]), half_nn(q[ok], tp[ok])))
    rows = np.array(rows)

    print(f'{os.path.basename(bag.rstrip("/"))}: {len(clouds)} clouds, {len(expected)} after the first IMU second')
    good_n = not missed and len(expected) > 0
    print(f"[{'OK ' if good_n else 'BAD'}] deskewed {len(expected) - len(missed)}/{len(expected)}"
          + (f' (first failure: {status[missed[0]]})' if missed else ''))
    print(f"[{'OK ' if intensity_ok else 'BAD'}] point count and intensity passed through")
    good_n &= intensity_ok
    good_diff = max_diff < MAX_DIFF
    print(f"[{'OK ' if good_diff else 'BAD'}] C++ vs NumPy reference: max point difference {max_diff * 1000:.3f} mm "
          f"(tol {MAX_DIFF * 1000:.1f})")
    turn, still = rows[:, 0] > TURN, rows[:, 0] < 0.05
    good_nn = True
    if turn.sum() >= 20:
        raw, desk = np.median(rows[turn, 1]), np.median(rows[turn, 2])
        good_nn = desk < 0.9 * raw
        print(f"[{'OK ' if good_nn else 'BAD'}] {turn.sum()} turning scans (|w| > {TURN} rad/s): late->early half NN "
              f"median raw {raw * 100:.2f} cm -> deskewed {desk * 100:.2f} cm; better in "
              f"{(rows[turn, 2] < rows[turn, 1]).mean() * 100:.0f} %")
    else:
        print(f'[-- ] only {turn.sum()} turning scans: improvement not checked')
    if still.sum() >= 20:
        raw, desk = np.median(rows[still, 1]), np.median(rows[still, 2])
        good = desk < raw + 0.001
        good_nn &= good
        print(f"[{'OK ' if good else 'BAD'}] {still.sum()} still scans: raw {raw * 100:.2f} cm, deskewed {desk * 100:.2f} cm")
    for f in os.listdir(tmp):
        os.remove(os.path.join(tmp, f))
    os.rmdir(tmp)
    ok = good_n and good_diff and good_nn
    print(f"deskew test: {'passed' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
