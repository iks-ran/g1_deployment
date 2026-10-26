#!/usr/bin/env python3
"""Synthetic deskew benchmark with ground truth (the bags have none).

Kinematic, not a robot simulator: deskew only depends on the lidar's motion during the scan, the scene and the
sensor models.
  - Motion: the real lidar trajectory of a bag. Rotation = integrated MID-360 gyro (bias removed); position =
    livox_frame origin from /odommodestate + waist FK (URDF), 15 ms moving average. Optionally time-compressed
    (--speed 2 = twice as fast: up to ~1.4 m/s, ~3 rad/s).
  - Scan pattern: the real MID-360 ray directions + per-point times of the bag's clouds (rays shorter than 1 m on
    the robot are its own body and are dropped).
  - Scene: a box room (12 x 10 x 3 m) with pillars, tables and crates; every ray is traced at the pose of its time.
    Ground truth = the hit point in the scan-START lidar frame.
  - Sensors: gyro 200 Hz (white noise 0.002 rad/s, bias 0.002 rad/s per axis, optional delay); leg odometry 500 Hz
    (pose noise 2 mm / 0.05 deg, optional lag). Range noise 2 cm (for the half-NN proxy only).
Methods: A raw | B gyro rotation (= deskew.hpp) | C gyro rotation + leg-odometry translation | D leg-odometry 6-DoF
| GT oracle (must be ~0). Reports the per-point error vs ground truth and the half-scan NN proxy used on real bags.
  scripts/sim_deskew_bench.py [bag_dir] [--speed 1] [--gyro-delay-ms 0] [--leg-lag-ms 0]
"""
import argparse
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_deskew import OUT, TRANS_STEP, half_nn, reference  # noqa: E402  (NumPy twin of deskew.hpp)

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(WS, 'src', 'g1_description', 'urdf', 'g1.urdf')
DT = np.dtype({'names': ['x', 'y', 'z', 'i', 'ring', 'time'], 'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
               'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 22})
WAIST = {'waist_yaw_joint': 12, 'waist_roll_joint': 13, 'waist_pitch_joint': 14}
STEP = 0.001  # ground-truth trajectory grid (s)
ROOM = (np.array([-6.0, -5.0, 0.0]), np.array([6.0, 5.0, 3.0]))  # odom frame, floor at z = 0 (pelvis ~0.71)
OBSTACLES = [((-2.5, 2.0, 0.0), (-2.0, 2.5, 3.0)), ((1.5, -2.5, 0.0), (2.0, -2.0, 3.0)),   # pillars
             ((-4.5, -3.0, 0.0), (-3.0, -2.2, 0.75)), ((2.5, 1.5, 0.0), (3.7, 2.3, 0.75)),  # tables
             ((0.5, 3.0, 0.0), (1.1, 3.6, 0.5)), ((-1.0, -4.2, 0.0), (0.0, -3.6, 1.2)),     # crates
             ((4.5, -1.0, 0.0), (5.2, 0.5, 1.8)), ((-5.5, 0.5, 0.0), (-4.8, 1.5, 2.2))]   # shelves


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def urdf_chain():
    joints = {}
    for j in ET.parse(URDF).getroot().findall('joint'):
        o, ax = j.find('origin'), j.find('axis')
        joints[j.find('child').get('link')] = dict(
            name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
            xyz=np.array([float(v) for v in o.get('xyz', '0 0 0').split()]) if o is not None else np.zeros(3),
            rpy=np.array([float(v) for v in o.get('rpy', '0 0 0').split()]) if o is not None else np.zeros(3),
            axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.array([1., 0, 0]))
    chain, link = [], 'livox_frame'
    while link != 'pelvis':
        chain.insert(0, joints[link])
        link = joints[link]['parent']
    return chain


def fk_origin(chain, q):
    """livox_frame origin in the pelvis frame for waist angles q (dict)."""
    Rm, t = R.identity(), np.zeros(3)
    for j in chain:
        Rj = R.from_euler('xyz', j['rpy'])
        if j['type'] == 'revolute':
            Rj = Rj * R.from_rotvec(j['axis'] * q[j['name']])
        t = t + Rm.apply(j['xyz'])
        Rm = Rm * Rj
    return t


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/utlidar/cloud_livox_mid360', '/utlidar/imu_livox_mid360',
                                                  '/odommodestate', '/lowstate']))
    clouds, imu, od, ls = [], [], [], []
    while r.has_next():
        tp, d, trec = r.read_next()
        trec *= 1e-9
        m = deserialize_message(d, get_message(types[tp]))
        if tp.endswith('cloud_livox_mid360'):
            a = np.frombuffer(bytes(m.data), DT)
            clouds.append((stamp(m.header), np.c_[a['x'], a['y'], a['z']].astype(float), a['time'] * 1e-9))
        elif tp.endswith('imu_livox_mid360'):
            w = m.angular_velocity
            imu.append((trec, stamp(m.header), w.x, w.y, w.z))
        elif tp == '/odommodestate':
            q = m.imu_state.quaternion
            if np.linalg.norm(q) > 0.5:
                od.append((trec, *m.position, q[1], q[2], q[3], q[0]))
        else:
            ls.append((trec, *[m.motor_state[i].q for i in WAIST.values()]))
    return clouds, *map(np.array, (imu, od, ls))


def trajectory(imu, od, ls, speed):
    """Ground truth on a STEP grid of robot time: rotations (world <- lidar) and lidar origins (world)."""
    imu = imu[np.argsort(imu[:, 1])]
    d = imu[:, 0] - imu[:, 1]  # host - robot offset: lower envelope over 2 s (as ClockOffset)
    k = np.floor((imu[:, 1] - imu[0, 1]) / 2).astype(int)
    et = np.array([imu[k == i, 1].mean() for i in np.unique(k)])
    eo = np.array([d[k == i].min() for i in np.unique(k)])
    ti, wi = imu[:, 1], imu[:, 2:5]
    wi = wi - wi[np.linalg.norm(wi, axis=1) < 0.03].mean(0)
    g = np.arange(ti[0] + 1, ti[-1] - 1, STEP)
    w = np.stack([np.interp(g, ti, wi[:, i]) for i in range(3)], 1)
    rv = R.from_rotvec(w * STEP)
    rots = [R.identity()]
    for x in rv[:-1]:
        rots.append(rots[-1] * x)
    rot = R.concatenate(rots)
    # lidar origin: pelvis position + R_pelvis * FK(waist); world yaw of the pelvis is irrelevant for a synthetic
    # path, so the FK lever is rotated by the gyro-integrated lidar attitude composed back to the pelvis.
    chain = urdf_chain()
    host = g + np.interp(g, et, eo)
    pel = np.stack([np.interp(host, od[:, 0], od[:, 1 + i]) for i in range(3)], 1)
    qw = np.stack([np.interp(host, ls[:, 0], ls[:, 1 + i]) for i in range(3)], 1)
    lever = np.array([fk_origin(chain, dict(zip(WAIST, q))) for q in qw[::20]])
    lever = np.stack([np.interp(np.arange(len(g)), np.arange(0, len(g), 20)[:len(lever)], lever[:, i])
                      for i in range(3)], 1)
    # attitude: start level-ish = the URDF mount at the first sample (lidar upside down, pitched), then the gyro
    mount = R.from_euler('xyz', [np.pi, 0.0511 + np.radians(7), 0])
    rot = mount * rot
    pel_rot = rot * mount.inv()
    pos = pel + pel_rot.apply(lever)
    ker = np.ones(15) / 15
    pos = np.stack([np.convolve(np.pad(pos[:, i], 7, mode='edge'), ker, 'valid') for i in range(3)], 1)
    pos -= pos[0] * [1, 1, 0]  # start at the room centre
    t = (g - g[0]) / speed  # time compression: same path, faster
    return t, rot, pos


def trace(o, d):
    """Distance along unit rays d (n,3) from origins o (n,3) to the first surface (room walls or an obstacle)."""
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / d
        lo, hi = (ROOM[0] - o) * inv, (ROOM[1] - o) * inv
        best = np.nanmin(np.maximum(lo, hi), axis=1)  # exit of the room
        for a, b in OBSTACLES:
            t1, t2 = (np.array(a) - o) * inv, (np.array(b) - o) * inv
            tn, tf = np.nanmax(np.minimum(t1, t2), axis=1), np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tn <= tf) & (tn > 0.05)
            best = np.where(hit & (tn < best), tn, best)
    return best


def pose_at(t, rot, pos, tq):
    i = np.clip(np.round((tq - t[0]) / (t[1] - t[0])).astype(int), 0, len(t) - 1)  # grid step = STEP / speed
    return rot[i], pos[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', nargs='?', default=os.path.join(WS, 'bags', 'run_2026-10-24_2147'))
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--gyro-delay-ms', type=float, default=0.0)
    ap.add_argument('--leg-lag-ms', type=float, default=0.0)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    clouds, imu, od, ls = load(args.bag)
    t, rot, pos = trajectory(imu, od, ls, args.speed)

    # sensors (time base = ground-truth time t)
    wgt = (rot[:-1].inv() * rot[1:]).as_rotvec() / np.diff(t)[:, None]  # body rates
    tw = t[:-1] + np.diff(t) / 2
    gt_i = np.arange(t[0], t[-1], 0.005)
    bias = rng.normal(0, 0.002, 3)
    gyro = np.stack([np.interp(gt_i - args.gyro_delay_ms * 1e-3, tw, wgt[:, i]) for i in range(3)], 1)
    gyro += bias + rng.normal(0, 0.002, gyro.shape)
    leg_t = np.arange(t[0], t[-1], 0.002)
    lr, lp = pose_at(t, rot, pos, leg_t - args.leg_lag_ms * 1e-3)
    lr = lr * R.from_rotvec(rng.normal(0, np.radians(0.05), (len(leg_t), 3)))
    lp = lp + rng.normal(0, 0.002, lp.shape)

    names = ['A raw', 'B gyro rot', 'C gyro rot + leg trans', 'D leg 6-DoF', 'C++ (deskew_check)', 'GT oracle']
    rows, scans = [], []
    base_ns = 1_000_000_000  # sim time 0 on the driver's (int64 ns) clock
    ns = lambda x: base_ns + np.round(np.asarray(x) * 1e9).astype(np.int64)  # noqa: E731
    tmp = tempfile.mkdtemp(prefix='g1_sim_deskew_')
    events = [(int(ns(tg)), f'i {int(ns(tg))} {w[0]!r} {w[1]!r} {w[2]!r}\n') for tg, w in zip(gt_i, gyro)]
    for i, (_, p_real, tp) in enumerate(clouds):  # real scan patterns, replayed at 10 Hz along the trajectory
        t_scan = 0.5 + 0.1 * i
        if t_scan + 0.2 > t[-1]:
            break
        rng_real = np.linalg.norm(p_real, axis=1)
        keep = np.isfinite(rng_real) & (rng_real > 1.0)
        d, tp = p_real[keep] / rng_real[keep, None], tp[keep]
        tp = np.frombuffer(np.float32(tp * 1e9).tobytes(), np.float32).astype(float) * 1e-9  # as stored (f32 ns)
        Rt, ot = pose_at(t, rot, pos, t_scan + tp)
        r = trace(ot, Rt.apply(d))
        ok = np.isfinite(r) & (r < 40)
        d, tp, r, Rt, ot = d[ok], tp[ok], r[ok], Rt[ok], ot[ok]
        hit = ot + Rt.apply(d) * r[:, None]
        span = tp.max()
        RT, oT = pose_at(t, rot, pos, np.array([t_scan + span]))
        gt = RT.inv().apply(hit - oT)  # ground truth in the scan-END frame (FAST-LIO2 convention)
        meas = d * r[:, None]
        noisy = d * (r + rng.normal(0, 0.02, len(r)))[:, None]
        # B: gyro rotation (deskew.hpp algorithm, NumPy twin)
        sel = (gt_i > t_scan - 0.5) & (gt_i < t_scan + 0.3)
        rel_ns = lambda x: ((x - t_scan) * 1e9).astype(np.int64)  # noqa: E731
        B, wmag = reference(meas, tp, 0, rel_ns(gt_i[sel]), gyro[sel])
        Bn, _ = reference(noisy, tp, 0, rel_ns(gt_i[sel]), gyro[sel])
        # leg odometry pose (nearest 500 Hz sample), relative to the scan end
        leg_index = lambda x: np.clip(np.round((np.asarray(x) - leg_t[0]) / 0.002).astype(int), 0, len(leg_t) - 1)  # noqa: E731
        li, lT = leg_index(t_scan + tp), int(leg_index(t_scan + span))
        RlTinv = lr[lT].inv()
        trans = RlTinv.apply(lp[li] - lp[lT])
        C, Cn = B + trans, Bn + trans
        D = (RlTinv * lr[li]).apply(meas) + trans
        G = (RT.inv() * Rt).apply(meas) + RT.inv().apply(ot - oT)
        # the C++: same gyro samples, leg translation every 5 ms (as the bridge samples TF)
        tk = t_scan + np.arange(int(span / TRANS_STEP) + 2) * TRANS_STEP
        tr = RlTinv.apply(lp[leg_index(tk)] - lp[lT])
        src = os.path.join(tmp, f'{i}.bin')
        a = np.zeros(len(meas), DT)
        a['x'], a['y'], a['z'], a['time'] = *meas.T.astype(np.float32), np.float32(tp * 1e9)
        a.tofile(src)
        events.append((int(ns(t_scan + span + 0.001)), f'c {int(ns(t_scan))} {src} {src}.out {TRANS_STEP!r} {len(tr)} '
                       + ' '.join(f'{v!r}' for v in tr.ravel()) + '\n'))
        scans.append((src, gt))
        err = [np.linalg.norm(x - gt, axis=1) for x in (meas, B, C, D, G)]
        vel = np.linalg.norm(RT.inv().apply(ot[-1] - ot[0])) / max(span, 1e-3)
        prox = [half_nn(x, tp) for x in (noisy, Bn, Cn)]
        rows.append([wmag, vel, err, prox])
    events.sort(key=lambda e: e[0])
    exe = os.path.join(get_package_prefix('g1_bringup'), 'lib', 'g1_bringup', 'deskew_check')
    status = subprocess.run([exe], input=''.join(e[1] for e in events), capture_output=True, text=True,
                            check=True).stdout.split('\n')
    assert all(x == 'ok' for x in status[:len(scans)]), status[:5]
    for row, (src, gt) in zip(rows, scans):
        b = np.fromfile(src + '.out', OUT)
        cpp = np.linalg.norm(np.c_[b['x'], b['y'], b['z']] - gt, axis=1)
        e = row[2][:4] + [cpp, row[2][4]]
        row[2:] = [*[np.median(x) for x in e], *[np.percentile(x, 95) for x in e], *row[3]]
        os.remove(src)
        os.remove(src + '.out')
    os.rmdir(tmp)
    rows = np.array(rows)
    print(f'{len(rows)} synthetic scans, speed x{args.speed:g}, gyro delay {args.gyro_delay_ms:g} ms, '
          f'leg lag {args.leg_lag_ms:g} ms; lidar |w| max {rows[:, 0].max():.2f} rad/s, v max {rows[:, 1].max():.2f} m/s')
    groups = [('all', np.ones(len(rows), bool)), ('turning |w|>0.4', rows[:, 0] > 0.4),
              ('fast turn |w|>1', rows[:, 0] > 1.0), ('walking v>0.3,|w|<0.3', (rows[:, 1] > 0.3) & (rows[:, 0] < 0.3))]
    print(f"{'error vs ground truth (cm)':28s}" + ''.join(f'{n:>24s}' for n in names))
    for label, s in groups:
        if s.sum() < 5:
            continue
        med = np.median(rows[s, 2:8], 0) * 100
        p95 = np.median(rows[s, 8:14], 0) * 100
        print(f'{label + f" ({s.sum()})":28s}' + ''.join(f'{m:12.2f} / p95 {q:5.2f}' for m, q in zip(med, p95)))
    print('half-NN proxy (cm, 2 cm range noise): ' + ', '.join(
        f'{label} A {np.median(rows[s, 14]) * 100:.2f} B {np.median(rows[s, 15]) * 100:.2f} '
        f'C {np.median(rows[s, 16]) * 100:.2f}' for label, s in groups[1:2] + groups[3:4] if s.sum() >= 5))


if __name__ == '__main__':
    sys.exit(main())
