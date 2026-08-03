#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pid_plugin_lib="${project_root}/build/plugins/libCoaxialPidController.so"
pid_plugin_src="${project_root}/plugins/CoaxialPidController.cc"
disturbance_plugin_lib="${project_root}/build/plugins/libNearSurfaceDisturbance.so"
disturbance_plugin_src="${project_root}/plugins/NearSurfaceDisturbance.cc"
aerodynamics_plugin_lib="${project_root}/build/plugins/libAerodynamicEnvironment.so"
aerodynamics_plugin_src="${project_root}/plugins/AerodynamicEnvironment.cc"
rotor_water_plugin_lib="${project_root}/build/plugins/libCoaxialWaterInteraction.so"
rotor_water_plugin_src="${project_root}/plugins/CoaxialWaterInteraction.cc"
landing_target_plugin_lib="${project_root}/build/plugins/libMovingLandingTarget.so"
landing_target_plugin_src="${project_root}/plugins/MovingLandingTarget.cc"

if [[ ! -f "${pid_plugin_lib}" || "${pid_plugin_src}" -nt "${pid_plugin_lib}" ||
      ! -f "${disturbance_plugin_lib}" || "${disturbance_plugin_src}" -nt "${disturbance_plugin_lib}" ||
      ! -f "${aerodynamics_plugin_lib}" || "${aerodynamics_plugin_src}" -nt "${aerodynamics_plugin_lib}" ||
      ! -f "${rotor_water_plugin_lib}" || "${rotor_water_plugin_src}" -nt "${rotor_water_plugin_lib}" ||
      ! -f "${landing_target_plugin_lib}" || "${landing_target_plugin_src}" -nt "${landing_target_plugin_lib}" ]]; then
  "${project_root}/scripts/build_plugins.sh"
fi

export GZ_SIM_RESOURCE_PATH="${project_root}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${project_root}/build/plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_PARTITION="${GZ_PARTITION:-coaxial_uav_static_water}"

runtime_dir="${project_root}/data/runtime"
mkdir -p "${runtime_dir}"
printf 'GZ_PARTITION=%s\n' "${GZ_PARTITION}" > "${runtime_dir}/active_gazebo_partition.env"
printf '{"partition":"%s","world":"static_water_takeoff","updated_unix_s":%s}\n' \
  "${GZ_PARTITION}" "$(date +%s)" > "${runtime_dir}/active_gazebo_partition.json"

exec gz sim "$@" -r "${project_root}/worlds/static_water_takeoff.sdf"
