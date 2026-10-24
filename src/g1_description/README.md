# g1_description

Unitree G1, 29 DoF, rev 1.0, copied from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) `robots/g1_description`
@ 15ebe53f4cd77e7d745ae83b9d7ff90e685e8998 (BSD-3-Clause, `LICENSE`; upstream README: `README.upstream.md`).

- `urdf/g1_29dof_rev_1_0.urdf` — upstream, untouched (relative `meshes/...` paths).
- `urdf/g1.urdf` — the one to use: upstream with `meshes/` → `package://g1_description/meshes/`, plus extra
  frames appended at the end (`livox_frame`, `livox_imu`, `robot_center`). `init.sh` checks that everything
  before the appended block is still the upstream file.
- `meshes/` — only the 35 STL files that `g1_29dof_rev_1_0.urdf` references.

Offline check: `ros2 launch g1_description display.launch.py`.
