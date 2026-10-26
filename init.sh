#!/usr/bin/env zsh
# Build + verify the g1_deployment workspace.
#   ./init.sh           build, then offline checks (URDF, launch files, bringup on the recorded bag)
#   ./init.sh --robot   additionally launch the bringup (no RViz) against the robot and check live topic rates
set -e
WS=${0:A:h}
cd $WS
source $WS/setup.zsh >/dev/null 2>&1 || true
export ROS_LOG_DIR=${ROS_LOG_DIR:-$WS/log/ros}
mkdir -p $ROS_LOG_DIR

echo "== build"
colcon build --symlink-install
source $WS/install/setup.zsh

echo "== URDF"
D=src/g1_description/urdf
check_urdf install/g1_description/share/g1_description/urdf/g1.urdf | head -3
# g1.urdf = upstream (mesh paths -> package://) + frames appended after the marker comment.
diff <(sed 's#filename="meshes/#filename="package://g1_description/meshes/#' $D/g1_29dof_rev_1_0.urdf | sed '$d') \
     <(sed '/Added for g1_deployment/,$d' $D/g1.urdf | sed '$d') >/dev/null \
  || { echo "g1.urdf: body differs from upstream g1_29dof_rev_1_0.urdf (edit only the appended block)"; exit 1 }
echo "g1.urdf body matches upstream g1_29dof_rev_1_0.urdf"
for m in $(grep -o 'package://g1_description/meshes/[^"]*' $D/g1.urdf | sort -u); do
  [[ -f src/g1_description/meshes/${m:t} ]] || { echo "missing mesh ${m:t}"; exit 1 }
done
echo "all meshes present"

echo "== python syntax / launch files"
python3 -m py_compile src/g1_bringup/launch/*.py \
  src/g1_description/launch/*.py scripts/*.py

echo "== clock offset: robot lidar/IMU stamps -> host time (recorded bags, no DDS)"
out=$(python3 scripts/test_clock_offset.py 2>&1) || { echo $out | grep -v INFO; exit 1 }
echo $out | grep -v INFO

echo "== deskew: C++ deskew (gyro rotation + translation, scan end) vs NumPy reference (walking bag, no DDS)"
out=$(python3 scripts/test_deskew.py 2>&1) || { echo $out | grep -v INFO; exit 1 }
echo $out | grep -v INFO

echo "== bringup on the recorded bags (isolated loopback DDS domain; skipped without bags/)"
for b in run_2026-10-24_2052 run_2026-10-24_2147; do   # suspended, walking
  out=$(python3 scripts/test_bag_bringup.py bags/$b) || { echo $out; exit 1 }
  echo "$b:"; echo $out | grep -E 'floor vs URDF|receipt - stamp|^\[CPU' 
  echo $out | tail -1
done
echo "offline checks passed"

if [[ $1 == --robot ]]; then
  echo "== live robot check"
  ros2 daemon stop >/dev/null 2>&1 || true
  setsid ros2 launch g1_bringup g1.launch.py rviz:=false </dev/null > $ROS_LOG_DIR/init_launch.log 2>&1 &
  LPID=$!
  trap 'kill -INT -- -$LPID 2>/dev/null; sleep 2; kill -9 -- -$LPID 2>/dev/null' EXIT
  sleep 5
  python3 scripts/check_live.py
fi
