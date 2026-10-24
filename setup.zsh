#!/usr/bin/env zsh
# Environment for g1_deployment. Usage: source ~/workspaces/g1_deployment/setup.zsh
#
# Sources the (read-only) unitree_ros2 environment, then overrides CYCLONEDDS_URI:
# with only the robot NIC configured, multicast discovery does NOT loop back on this host,
# so local nodes (robot_state_publisher, rviz, ...) cannot see each other. Adding this
# host's own IP as a unicast discovery peer fixes that while keeping the robot reachable.
#
# G1_DDS=loopback source ./setup.zsh   offline (robot disconnected, bag replay): CycloneDDS on lo, fixed
#                                       ROS_DOMAIN_ID (scripts/dds_loopback.zsh). Default: robot NIC.
G1_WS=${0:A:h}
G1_IFACE=${G1_IFACE:-enx3c18a0d80cff}

source $HOME/workspaces/unitree_ros2/setup.zsh
if [[ ${G1_DDS:-robot} == loopback ]]; then
  source $G1_WS/scripts/dds_loopback.zsh
else
  # The robot publishes on domain 0. Set it explicitly: a domain left over from loopback mode (or another workspace's
  # offline domain) in the same shell hides every robot topic without any error.
  export ROS_DOMAIN_ID=${G1_ROBOT_DOMAIN:-0}
  G1_HOST_IP=$(ip -4 -o addr show dev $G1_IFACE 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
  if [[ -z $G1_HOST_IP ]]; then
    echo "[g1_deployment] WARNING: interface $G1_IFACE has no IPv4 address; robot not connected? (offline: G1_DDS=loopback)" >&2
    G1_HOST_IP=127.0.0.1
  fi
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$G1_IFACE\" priority=\"default\" multicast=\"default\"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><SPDPInterval>1s</SPDPInterval><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer address=\"$G1_HOST_IP\"/></Peers></Discovery></Domain></CycloneDDS>"
fi

[[ -f $G1_WS/install/setup.zsh ]] && source $G1_WS/install/setup.zsh
