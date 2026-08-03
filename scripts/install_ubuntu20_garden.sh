#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -r /etc/os-release ]]; then
  printf 'Cannot identify the operating system. Ubuntu 20.04 is required.\n' >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "20.04" ]]; then
  printf 'This reproducible setup targets Ubuntu 20.04; detected %s %s.\n' \
    "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  curl \
  gnupg \
  libgl1-mesa-dri \
  lsb-release \
  mesa-utils \
  pkg-config \
  python3

sudo install -d -m 0755 /usr/share/keyrings
sudo curl -fsSL \
  https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

architecture="$(dpkg --print-architecture)"
codename="$(lsb_release -cs)"
repository="deb [arch=${architecture} signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${codename} main"
printf '%s\n' "${repository}" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo apt-get update
sudo apt-get install -y gz-garden libgz-rendering7-ogre1

"${project_root}/scripts/check_environment.sh"
"${project_root}/scripts/build_plugins.sh"

printf 'Environment ready. Start Gazebo with:\n  %s/scripts/run_static_water_gui_ogre1.sh\n' \
  "${project_root}"
