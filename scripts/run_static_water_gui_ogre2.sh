#!/usr/bin/env bash
# GPU 加速版启动脚本：使用 OGRE2 渲染引擎 + WSLg 硬件加速（d3d12）。
# 相比 run_static_water_gui_ogre1.sh（OGRE1 + llvmpipe 软件渲染），
# 在有可用 GPU 的 WSLg 环境下画面更流畅。
# 若 OGRE2 窗口异常，可切回 OGRE1 软件渲染脚本。
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"

# 不强制软件渲染，让 Mesa 使用 WSLg 的 d3d12 硬件后端
unset LIBGL_ALWAYS_SOFTWARE
unset GALLIUM_DRIVER
# 明确用 OGRE2，避免 ~/.bashrc 的 GZ_GUI_RENDER_ENGINE=ogre 干扰
unset GZ_GUI_RENDER_ENGINE

export GZ_SIM_RESOURCE_PATH="${project_root}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_PARTITION="${GZ_PARTITION:-coaxial_uav_gui}"

exec "${script_dir}/run_static_water.sh" --render-engine-gui ogre2 "$@"
