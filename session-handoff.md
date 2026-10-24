# Session Handoff

- **Last Updated:** 2026-10-24 (F4 live robot check passed; three fixes; branch robot-check)
- **Current Objective:** bringup only. F1–F7 done (F4 live on the G1).
- **Blockers:** none. Git: branch robot-check (merge into main), not pushed yet (origin = github.com/iks-ran/g1_deployment).
- **On the robot:** `G1_DDS=robot ./init.sh --robot` (this host's Claude Code env carries G1_DDS=loopback); live
  container CPU 7.0 %.
- **Recommended Next Step:** none open in scope (bringup). Optional: `check_live.py --floor` live with the G1
  standing on flat ground (the walking bag already verified it).
