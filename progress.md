# Progress

## Current State (Last Updated: 2026-10-24)
F1–F3 (description, TF/joints, lidar + IMU relay) done and verified offline on the recorded bag. F4 needs the robot.

## Log
### 2026-10-24 — initial workspace (bringup)
- What: `g1_description` (upstream g1_29dof_rev_1_0 + meshes, `g1.urdf` with extra frames), `g1_bringup`
  (state_bridge, g1.launch.py, g1.rviz), `setup.zsh`, `init.sh`, `scripts/{check_live,test_bag_bringup}.py`,
  `scripts/play_bag.zsh`, modelled on go2_deployment.
- Found: support.unitree.com blocks scripted access; extrinsics taken from the user's screenshot
  (`docs/unitree_lidar_extrinsics.png`) + upstream URDF `mid360_joint`; both agree with the bag (see CLAUDE.md).
- Found & fixed: /g1/cloud at 0-1 Hz with best-effort readers on loopback → Reliable subscription/readers.
- Found & fixed: double SIGINT on Ctrl-C killed state_bridge inside destroy_node → ignore SIGINT during cleanup.
- Verified: `./init.sh` passes twice in a row; RViz on the bag (`docs/rviz_bringup_bag.png`).
- Next: F4 `./init.sh --robot` once the G1 is connected.

### 2026-10-24 — lidar extrinsics: which one is right
- User: the bag robot is suspended (small gap under the feet), not standing.
- New `scripts/check_mount_imu.py`: gravity in the torso IMU vs the MID-360 IMU (still windows, 113 s) → mount
  pitch 2.93° (= URDF 0.0511 rad), roll −0.15°, residual 0.04°; doc's 2.3° gives 0.65°. Rotation: keep the URDF.
- Floor check reinterpreted: floor +1.0 cm above the soles while hanging → vertical error ≈ 1 cm + gap (F5 open).

### 2026-10-24 — walking bag: extrinsics verified (F5)
- User recorded `bags/run_2026-10-24_2147` (65 s walking).
- `scripts/check_mount_gyro.py`: torso vs MID-360 gyros → pitch 2.911±0.05°, yaw −0.22±0.29° (URDF 2.93°, 0).
- Pelvis IMU through waist FK vs torso IMU (hanging bag): −0.10° about y → the TF chain agrees with the IMU fits.
- `scripts/floor_sectors.py`: clear floor ahead within −0.2…+0.5 cm of the stance foot; other sectors bowl upward
  with distance (no rigid lidar error fits) → environment. Floor tilt alone (±0.5°) can't judge the mount.
- Decision: keep upstream URDF `mid360_joint`; no change to g1.urdf.
- check_live --floor now uses the clearest of 4 sectors (1-2.5 m): -1.6 cm (hanging) / -1.4 cm (walking); init.sh gates both bags.

### 2026-10-24 — stamps: robot clock -> host time (F6)
- User asked whether relaying the cloud only to restamp it is wasteful. Measured bridge CPU (bag replay): cloud relay
  1 %, /lowstate decode 36 %, /odommodestate 12 % → relay kept; CPU work deferred until measured on the robot.
- Found: restamping on receipt put every cloud ~100 ms late (stamp = scan start, per-point time 0..100 ms).
- Fix: `g1_bringup/clock_offset.py` + state_bridge converts lidar/IMU robot stamps; `scripts/test_clock_offset.py`
  (both bags: IMU latency 0.25 ms, cloud 100.3 ms); check_live checks receipt − stamp (cloud 103 ms, IMU 0.4 ms).
- Effect on the walking bag is small (floor scatter 1.21 → 1.06 cm): gentle walking; matters most for turns/mapping.
- check_live prints [CPU] for state_bridge / robot_state_publisher (bag: 46 % / 0.5 %), so `./init.sh --robot` measures it live.

### 2026-10-24 — C++ bridge (F7)
- User: step into C++ first. g1_bringup is now ament_cmake: `StateBridge` component (same topics/params/behaviour),
  `clock_offset.hpp`, `clock_offset_check` (test_clock_offset.py pipes the bags through it: identical results).
- g1.launch: component container with robot_state_publisher. Found: intra-process + transient local (tf_static)
  throws in Humble → no intra-process.
- CPU (walking bag, standalone bridge): 45 % → 5 % of a core; container incl. robot_state_publisher ≈ 6 %.
- check_live [CPU] matches argv[0] (shells mentioning the names were counted). RViz: docs/rviz_bringup_walking.png.
- Python bridge removed (copy in /tmp/claude-1000/py_bridge_backup until reboot; no git yet).

### 2026-10-24 — F4: live robot check passes (+ three fixes)
- Fixes found while doing the same work in go2_deployment: init.sh no longer masks a failing clock-offset test
  (`$(python … | grep)` returned grep's status); robot-mode setup.zsh exports ROS_DOMAIN_ID=0 (an inherited offline
  domain hid every robot topic on the Go2); the bridge logs the offset once a 2 s window is full (the startup burst
  printed a false "clock offset JUMPED").
- This host's Claude Code session inherits G1_DDS=loopback / ROS_DOMAIN_ID=91: robot runs need `G1_DDS=robot`.
- `G1_DDS=robot ./init.sh --robot` passes: all rates (cloud 10.0 Hz, imu 200.0 Hz), all TF chains, cloud receipt −
  stamp 101.0 ms, imu 0.4 ms, g1_container 7.0 % of a core, clock offset +1970.836 s (bags: 1969.2, 1970.3).
