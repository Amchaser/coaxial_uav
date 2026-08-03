#!/usr/bin/env bash
set -euo pipefail

omega="${1:-136.362}"

export GZ_PARTITION="${GZ_PARTITION:-coaxial_uav_static_water}"

gz topic \
  -t /coaxial_uav/gazebo/command/motor_speed \
  -m gz.msgs.Actuators \
  -p "velocity:[${omega}, ${omega}]"

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="${project_root}/data/runtime"
mkdir -p "${runtime_dir}"
printf '{"upper_rad_s":%s,"lower_rad_s":%s,"source":"commanded","updated_unix_s":%s}\n' \
  "${omega}" "${omega}" "$(date +%s)" > "${runtime_dir}/motor_command.json"
