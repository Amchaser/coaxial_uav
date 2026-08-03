#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
publisher_bin="${project_root}/build/tools/gz_config_publisher"
publisher_src="${project_root}/controllers/GazeboConfigPublisher.cc"

if [[ ! -x "${publisher_bin}" || "${publisher_src}" -nt "${publisher_bin}" ]]; then
  "${project_root}/scripts/build_plugins.sh"
fi

has_partition_arg=0
for arg in "$@"; do
  case "${arg}" in
    --partition|--partition=*)
      has_partition_arg=1
      ;;
  esac
done

if [[ "${has_partition_arg}" -eq 0 ]]; then
  active_partition_file="${project_root}/data/runtime/active_gazebo_partition.env"
  if [[ -f "${active_partition_file}" ]]; then
    active_partition="$(sed -n 's/^GZ_PARTITION=//p' "${active_partition_file}" | head -n 1)"
    if [[ -n "${active_partition}" ]]; then
      set -- --partition "${active_partition}" "$@"
    fi
  fi
fi

exec python3 "${project_root}/dashboard/server.py" "$@"
