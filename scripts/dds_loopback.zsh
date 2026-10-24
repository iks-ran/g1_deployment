#!/usr/bin/env zsh
# Offline DDS profile: CycloneDDS on `lo` with multicast on and a fixed offline domain (bag replay, tests).
# g1 shells get it via `G1_DDS=loopback source ./setup.zsh`.
export ROS_DOMAIN_ID=${G1_OFFLINE_DOMAIN:-91}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
