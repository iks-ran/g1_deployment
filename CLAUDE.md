# g1_deployment

ROS 2 Humble workspace for the Unitree G1 (29 DoF, rev 1.0): URDF, TF tree, MID-360 lidar/IMU, RViz. Bringup only
(modelled on `~/workspaces/go2_deployment`).

## Startup Workflow (every session, before writing code)
1. `source ./setup.zsh` (robot) or `G1_DDS=loopback source ./setup.zsh` (offline / bag replay).
2. Read `progress.md` (latest entry) and `feature_list.json` (pick the single `in_progress`, else first `todo`).
3. `./init.sh` (offline, uses the recorded bag) — must pass before any change.

## Layout
- `src/g1_description/` — URDF + meshes from unitreerobotics/unitree_ros `robots/g1_description` @ 15ebe53.
  `urdf/g1_29dof_rev_1_0.urdf` is upstream, untouched. `urdf/g1.urdf` = upstream with `meshes/` →
  `package://g1_description/meshes/` + extra frames appended (`livox_frame`, `livox_imu`, `robot_center`).
  Edit only the appended block (`init.sh` diffs the rest against upstream). `meshes/` = the 35 STLs it references.
- `src/g1_bringup/` (ament_cmake, C++) — `src/state_bridge.cpp`: component `g1_bringup::StateBridge`
  (/lowstate → /joint_states, /odommodestate → TF odom→pelvis + /g1/odom, lidar/IMU relay to /g1/cloud, /g1/imu with
  robot stamps converted to host time; /g1/cloud_deskewed; also a standalone `state_bridge` executable for profiling),
  `include/g1_bringup/deskew.hpp` (deskew to the scan end: gyro rotation + leg-odometry translation),
  `src/deskew_check.cpp` (stdin driver),
  `include/g1_bringup/clock_offset.hpp` (robot→host clock offset), `src/clock_offset_check.cpp` (stdin driver for
  the offset test), `launch/g1.launch.py` (bridge + robot_state_publisher in container `g1_container`),
  `rviz/g1.rviz` (the ONLY RViz config).
- `bags/run_2026-10-24_2052/` — real G1 recording (167 s, robot SUSPENDED: feet a small gap above the floor,
  still). Default bag of the offline gate. `bags/run_2026-10-24_2147/` — 65 s WALKING on the floor (≤0.73 m/s).
  Not in git. `scripts/play_bag.zsh [bag]` replays the topics the bringup uses.
- `scripts/check_live.py` — rates + TF chains + stamp age (receipt − stamp) + `--floor` (lidar floor vs URDF sole);
  prints `[CPU]` of g1_container (or standalone state_bridge / robot_state_publisher), informational.
- `scripts/test_clock_offset.py [bag ...]` — offline (no DDS) test of the C++ clock offset on the bags (via clock_offset_check).
- `scripts/check_mount_imu.py` — lidar mount rotation from torso-IMU vs MID-360-IMU gravity (still windows).
- `scripts/check_mount_gyro.py` — full mount rotation from both gyros (walking bag; the strongest check).
- `scripts/floor_sectors.py` — lidar floor minus lowest foot, per sector × distance (walking bag).
- `scripts/test_deskew.py [bag]` — offline (no DDS) test of the C++ deskew on the walking bag (via deskew_check).
- `scripts/sim_deskew_bench.py [bag] [--speed --gyro-delay-ms --leg-lag-ms]` — synthetic deskew benchmark with ground
  truth (real lidar trajectory + real MID-360 scan pattern ray-traced in a box room; gyro / leg-odometry models).
- `scripts/test_bag_bringup.py [bag]` — offline gate: bag replay + g1.launch + check_live --floor (loopback domain 92);
  init.sh runs it on both bags.
- `scripts/dds_loopback.zsh` — offline DDS vars (lo, domain 91). `scripts/winshot.zsh` — RViz screenshot.
- `docs/unitree_lidar_extrinsics.png` — Unitree's lidar doc (support.unitree.com blocks scripted access).

## Robot interface (measured on the bag)
| topic | type | rate | notes |
|---|---|---|---|
| /lowstate | unitree_hg/LowState | 1050 Hz | motor_state[0..28] = URDF actuated-joint order (L leg, R leg, waist y/r/p, L arm, R arm) |
| /odommodestate | unitree_go/SportModeState | 500 Hz | position + imu_state.quaternion (w,x,y,z) = **pelvis** pose |
| /utlidar/cloud_livox_mid360 | PointCloud2 | 10 Hz | frame `livox_frame`, ~20k pts, fields x y z intensity ring time |
| /utlidar/imu_livox_mid360 | Imu | 200 Hz | EMPTY frame_id, no orientation, acc in **g** |
| /dog_odom | Odometry | 51 Hz | odom → `robot_center` (= pelvis), same pose as /odommodestate |
Also present: /secondary_imu (torso IMU, unitree_hg/IMUState), /ele_clouds, /dog_imu_raw, /lf/* low-rate copies.

## Invariants (learned the hard way)
- `~/workspaces/unitree_ros2` is READ-ONLY. Never write there.
- DDS: with only the robot NIC configured, multicast does not loop back on this host → `setup.zsh` adds a self
  unicast peer + `AllowMulticast=spdp` (same as go2). Default NIC `enx3c18a0d80cff`, override with `G1_IFACE`.
- The robot publishes on DDS domain 0: robot-mode `setup.zsh` exports `ROS_DOMAIN_ID=${G1_ROBOT_DOMAIN:-0}` (a leftover
  offline domain hides every robot topic with no error). `G1_DDS` is read from the environment: a shell that inherited
  `G1_DDS=loopback` stays offline even for `./init.sh --robot` — use `G1_DDS=robot ./init.sh --robot`.
- Robot clock ≠ host clock (host − robot ≈ +1969 s on 2026-10-24, +1.1 s between bags 55 min apart: it drifts).
  Never forward raw robot stamps, and never restamp lidar data on receipt: the cloud stamp is the scan START
  (per-point `time` 0..100 ms), ~100 ms before it arrives. `state_bridge` converts lidar/IMU stamps with
  `ClockOffset` (lower envelope of receipt − stamp of the 200 Hz lidar IMU over 2 s; IMU latency ≈ 0.25 ms); the
  offset is logged once the first 2 s window is full (the startup burst is late), then only on real jumps.
  /lowstate and /odommodestate have no usable stamp (SportModeState.stamp = 0): stamped on receipt.
- Bridge CPU (walking bag replay): Python was 45 % of a core (/lowstate decode 36 %, /odommodestate 12 %); the C++
  component is 5 % (lowstate 3, odom 2, cloud 1, IMU 0); container with robot_state_publisher ≈ 6 %.
- No intra-process comms in g1_container: Humble allows it only for volatile topics; robot_state_publisher's
  /tf_static and /robot_description are transient local → its constructor throws ("allowed only with volatile").
- The robot's cloud is RAW (no motion compensation: verified, deskew with the gyro is optimal at gain 1). Per-point
  `time` is float32 **ns** after the stamp. /g1/cloud stays raw (LIO deskews itself). /g1/cloud_deskewed (only while
  subscribed) follows FAST-LIO2: every point in livox_frame at the scan END, stamp = scan end, fields x y z intensity
  (float32, 16 B). Rotation from the MID-360 gyro (same clock as the points), translation of the lidar origin from leg
  odometry via TF odom → livox_frame (5 ms samples). Waits for gyro + TF to cover the scan end (≤ 50 ms; arrives
  ~30 ms after the scan end), else deskews with what is there. Leg odometry alone is worse (its rotation, 25 ms lag).
  Synthetic ground truth: median/p95 error raw 4.1/13.2 cm → 0.39/0.65 cm (rotation only: 0.82/1.54).
- The robot does no lidar odometry: /odommodestate (= /dog_odom) is proprioceptive (IMU + legs): yaw drifts 0.3°/min
  while hanging (lidar: 0), 1.9° / 3 cm vs lidar ICP over an 11 m walk; its z is held near nominal height.
  SportModeState.imu_state.gyroscope is all ZERO. /slam_info is telemetry JSON, not a pose.
- Timing vs the MID-360 gyro (walking bag, |w| cross-correlation): torso IMU (/secondary_imu) arrives 8 ms BEFORE the
  gyro's stamp says it happened (corr 0.98), /lowstate pelvis IMU and /odommodestate −23/−25 ms. Irrelevant for
  deskew (metric flat over ±20 ms), not yet resolved for cloud-vs-TF alignment.
- Host UDP buffer: net.core.rmem_max = 208 KB (default) < one cloud. Over the robot cable it is fine; on loopback
  (bag gate) a Reliable reader of large clouds loses fragments (5 % of UDP packets) and gets them repaired on the
  next heartbeat → +100/200 ms. check_live checks stamp age on the 10th percentile and reads /g1/cloud_deskewed in
  a separate window (both clouds at once in one process: 7.7 Hz).
- The lidar cloud (~440 KB) must be read with RELIABLE QoS: a best-effort reader loses whole samples on one dropped
  fragment (bag replay on lo: 0 Hz best effort vs 9.3 Hz reliable). RViz and check_live read /g1/cloud Reliable.
- Lidar extrinsics: upstream `mid360_joint` (torso_link, z 0.4284, rpy π, 0.0511, 0) — MID-360 is upside down.
  Unitree's doc says pelvis→lidar z 0.47618, inverted, pitch −2.3° (URDF: 0.4724 at zero waist, 2.93°).
  VERIFIED, keep the URDF (doc's 2.3° is 0.65° off, its z 4 mm off — same mount, less precise):
  gyro fit on the walking bag pitch 2.911±0.05°, yaw −0.22±0.29°, roll π (0.22° from URDF); gravity fit on the
  hanging bag pitch 2.93°, residual 0.04°; pelvis IMU → waist FK vs torso IMU agrees about y to −0.10° (about x
  +0.87°: waist-roll encoder or torso-IMU mounting). Height: walking bag, clear floor ahead: lidar floor −0.2…+0.5 cm
  from the stance foot over 1–3.5 m. Other sectors rise with distance in EVERY direction (bowl): no rigid pose error
  does that (best rigid fit leaves systematic 1.1 cm rms and needs a pitch the IMU checks exclude) → environment.
  Floor-plane tilt through the full chain is only good to ~±0.5° (walking dynamics, floor flatness): don't use it
  to judge the mount.
  MID-360 IMU in the lidar frame: (0.011, 0.02329, −0.04412), no rotation (Unitree doc = Livox manual).
- The waist encoders matter: the torso is pitched ~7° on the bag; the extrinsics only check out through the full
  chain pelvis → waist_yaw/roll/pitch → torso_link → mid360_link.
- Tool sandbox hides the robot NIC and the X display: robot/RViz commands need the sandbox disabled.
  Set `ROS_LOG_DIR` to a writable dir (`~/.ros` is read-only in the sandbox).
- Test nodes: start with PR_SET_PDEATHSIG + kill the process group (`ros2 launch` doesn't forward SIGTERM).
- After adding a NEW package, every open shell must re-source `./setup.zsh`.

## Definition of done
- `./init.sh` passes; if the change touches runtime behaviour and the robot is available, `./init.sh --robot`.
- For visual changes, verify in RViz on the bag (`DISPLAY=:0` + `scripts/winshot.zsh`).
- Update `feature_list.json` (status + evidence) and append to `progress.md`.
- Never send motion commands (`/api/*` requests, `/lowcmd`, `/arm_sdk`) unless the user asks.

## Scope
Bringup only: description, TF, joint states, lidar/IMU relay, RViz. Mapping, navigation, locomotion/control are out
of scope unless the user asks; add them as new `feature_list.json` entries first.

## End of Session
Run `./init.sh`, record evidence in `feature_list.json`, update `session-handoff.md`, append to `progress.md`.
