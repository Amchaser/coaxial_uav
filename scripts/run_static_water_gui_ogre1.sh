#!/usr/bin/env bash
set -euo pipefail

# Try Gazebo's older OGRE renderer. This can work on systems where OGRE2's
# GL3Plus window creation fails.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"

export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export GZ_SIM_RESOURCE_PATH="${project_root}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_PARTITION="${GZ_PARTITION:-coaxial_uav_gui}"

if [[ "${GZ_GUI_ATTACH:-0}" == "1" ]] && gz topic -l 2>/dev/null | grep -qx "/world/static_water_takeoff/stats"; then
  printf 'Existing static_water_takeoff server detected in GZ_PARTITION=%s; starting GUI client only.\n' "${GZ_PARTITION}" >&2
  exec gz sim -g --render-engine-gui ogre "$@"
fi

printf 'Starting isolated static_water_takeoff GUI in GZ_PARTITION=%s.\n' "${GZ_PARTITION}" >&2
exec "${script_dir}/run_static_water.sh" --render-engine-gui ogre "$@"
