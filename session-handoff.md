# Session Handoff

- **Last Updated:** 2026-10-24 session 1 (bringup verified on both bags; extrinsics verified; stamps fixed; C++ bridge)
- **Current Objective:** bringup only. F1–F3, F5–F7 done offline; F4 (live robot) open.
- **Blockers:** robot not connected (F4). Lidar extrinsics verified on the walking bag (F5 done). Git: main, not pushed yet (origin = github.com/iks-ran/g1_deployment).
- **On the robot:** `source ./setup.zsh && ./init.sh --robot` prints `[CPU] g1_container` (bag: ~6 %).
- **Recommended Next Step:** connect the G1, `source ./setup.zsh && ./init.sh --robot`; check the NIC name
  (`G1_IFACE`, default copied from go2) and that the live /utlidar cloud QoS is Reliable as in the bag.
