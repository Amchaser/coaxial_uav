#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GZ_SIM_RESOURCE_PATH="${project_root}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_PARTITION="${GZ_PARTITION:-coaxial_uav_attitude_test}"

exec gz sim "$@" -r "${project_root}/worlds/attitude_actuator_test.sdf"
