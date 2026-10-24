#!/usr/bin/env python3
"""Offline bringup test on a recorded bag: replays it, runs g1.launch.py (rviz:=false) and check_live --floor.

Isolated loopback DDS domain 92. Plays from 20 s into the bag (robot suspended, still), so the floor check compares the lidar floor with the URDF
sole height through the full TF tree (odom TF, joint states, extra frames); see check_live.py for the tolerance.
  scripts/test_bag_bringup.py [bag_dir]
"""
import ctypes
import os
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '92'
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><General><Interfaces>'
                                '<NetworkInterface name="lo" multicast="true"/>'
                                '</Interfaces></General></Domain></CycloneDDS>')

import rclpy  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_live  # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOPICS = ['/lowstate', '/odommodestate', '/utlidar/cloud_livox_mid360', '/utlidar/imu_livox_mid360']


def _die_with_parent():
    ctypes.CDLL('libc.so.6').prctl(1, signal.SIGTERM)


def _start(cmd, log):
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            preexec_fn=_die_with_parent)


def _stop(proc):
    # ros2 launch does not forward SIGTERM to its nodes: signal the whole process group.
    for sig, wait in ((signal.SIGINT, 10), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=wait)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            continue


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else os.path.join(WS, 'bags', 'run_2026-10-24_2052')
    if not os.path.isdir(bag):
        print(f'bag bringup test: skipped (no bag at {bag})')
        return 0
    log = tempfile.NamedTemporaryFile('w+', prefix='g1_bag_test_', suffix='.log', delete=False)
    launch = _start(['ros2', 'launch', 'g1_bringup', 'g1.launch.py', 'rviz:=false'], log)
    time.sleep(3)
    play = _start(['ros2', 'bag', 'play', bag, '--start-offset', '20', '--topics', *TOPICS], log)
    rclpy.init()
    try:
        time.sleep(2)
        ok = check_live.run(warmup=4.0, window=6.0, floor=True)
        ok &= launch.poll() is None
    finally:
        _stop(play)
        _stop(launch)
        rclpy.shutdown()
    text = open(log.name).read()
    if 'Traceback' in text or 'process has died' in text:
        ok = False
        print(text[-3000:])
    print(f"bag bringup test: {'passed' if ok else 'FAILED'} (log {log.name})")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
