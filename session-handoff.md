# Session Handoff

- **Last Updated:** 2026-10-26 (F8 /g1/cloud_deskewed: gyro rotation + leg-odometry translation, to the scan end)
- **Current Objective:** bringup only. F1–F8 done.
- **Blockers:** none. Git: branch robot-check (merge into main), not pushed yet (origin = github.com/iks-ran/g1_deployment).
- **On the robot:** `G1_DDS=robot ./init.sh --robot` (this host's Claude Code env carries G1_DDS=loopback); live
  container CPU 7.0 %.
- **Host:** net.core.rmem_max is 208 KB: loopback readers of the cloud lose fragments (+100 ms). Needs sudo to fix.
- **Not yet on the robot:** F8 (`G1_DDS=robot ./init.sh --robot` checks /g1/cloud_deskewed rate + stamp).
- **Recommended Next Step:** `G1_DDS=robot ./init.sh --robot` for F8 live. Optional: cut the ~30 ms TF wait of the
  deskewed cloud (odom TF / joint_states throttled to 100 Hz). Optional: `check_live.py --floor` live with the G1
  standing on flat ground (the walking bag already verified it).
