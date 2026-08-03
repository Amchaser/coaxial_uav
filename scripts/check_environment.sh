#!/usr/bin/env bash
set -euo pipefail

required_commands=(g++ gz pkg-config python3)
for command_name in "${required_commands[@]}"; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    printf 'Missing required command: %s\n' "${command_name}" >&2
    exit 1
  fi
done

required_modules=(gz-sim7 gz-plugin2 gz-transport12 gz-msgs9)
for module_name in "${required_modules[@]}"; do
  if ! pkg-config --exists "${module_name}"; then
    printf 'Missing required pkg-config module: %s\n' "${module_name}" >&2
    exit 1
  fi
done

gz_sim_version="$(gz sim --versions | head -n 1)"
if [[ "${gz_sim_version}" != 7.* ]]; then
  printf 'Expected gz-sim 7.x (Garden), detected %s.\n' "${gz_sim_version}" >&2
  exit 1
fi

printf 'Environment check passed.\n'
printf '  gz-sim: %s\n' "${gz_sim_version}"
printf '  compiler: %s\n' "$(g++ -dumpfullversion -dumpversion)"
printf '  python: %s\n' "$(python3 --version 2>&1)"
