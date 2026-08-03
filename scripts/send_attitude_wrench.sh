#!/usr/bin/env bash
set -euo pipefail

roll_nm="${1:-0}"
pitch_nm="${2:-0}"
yaw_nm="${3:-0}"
wrench_world="${WRENCH_WORLD:-static_water_takeoff}"

export GZ_PARTITION="${GZ_PARTITION:-coaxial_uav_static_water}"

gz topic \
  -t "/world/${wrench_world}/wrench/persistent" \
  -m gz.msgs.EntityWrench \
  -p "entity: {name: 'coaxial_uav::base_link', type: LINK}, wrench: {torque: {x: ${roll_nm}, y: ${pitch_nm}, z: ${yaw_nm}}}"
