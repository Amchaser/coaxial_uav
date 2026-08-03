#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${project_root}/build/plugins"
tool_dir="${project_root}/build/tools"
mkdir -p "${build_dir}"
mkdir -p "${tool_dir}"

g++ -std=c++17 -fPIC -shared \
  "${project_root}/plugins/CoaxialPidController.cc" \
  -o "${build_dir}/libCoaxialPidController.so" \
  $(pkg-config --cflags --libs gz-sim7 gz-plugin2 gz-transport12 gz-msgs9)

g++ -std=c++17 -fPIC -shared \
  "${project_root}/plugins/NearSurfaceDisturbance.cc" \
  -o "${build_dir}/libNearSurfaceDisturbance.so" \
  $(pkg-config --cflags --libs gz-sim7 gz-plugin2 gz-transport12 gz-msgs9)

g++ -std=c++17 -fPIC -shared \
  "${project_root}/plugins/AerodynamicEnvironment.cc" \
  -o "${build_dir}/libAerodynamicEnvironment.so" \
  $(pkg-config --cflags --libs gz-sim7 gz-plugin2 gz-transport12 gz-msgs9)

g++ -std=c++17 -fPIC -shared \
  "${project_root}/plugins/CoaxialWaterInteraction.cc" \
  -o "${build_dir}/libCoaxialWaterInteraction.so" \
  $(pkg-config --cflags --libs gz-sim7 gz-plugin2 gz-transport12 gz-msgs9)

g++ -std=c++17 -fPIC -shared \
  "${project_root}/plugins/MovingLandingTarget.cc" \
  -o "${build_dir}/libMovingLandingTarget.so" \
  $(pkg-config --cflags --libs gz-sim7 gz-plugin2 gz-transport12 gz-msgs9)

g++ -std=c++17 \
  "${project_root}/controllers/GazeboConfigPublisher.cc" \
  -o "${tool_dir}/gz_config_publisher" \
  $(pkg-config --cflags --libs gz-transport12 gz-msgs9)
