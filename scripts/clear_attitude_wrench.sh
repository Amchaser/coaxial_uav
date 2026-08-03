#!/usr/bin/env bash
set -euo pipefail

wrench_world="${WRENCH_WORLD:-static_water_takeoff}"

export GZ_PARTITION="${GZ_PARTITION:-coaxial_uav_static_water}"

gz topic \
  -t "/world/${wrench_world}/wrench/clear" \
  -m gz.msgs.Entity \
  -p "name: 'coaxial_uav::base_link', type: LINK"
