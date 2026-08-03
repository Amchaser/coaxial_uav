#!/usr/bin/env bash
set -euo pipefail

# Prefer WSLg Wayland/EGL over X11/GLX. This is useful when OGRE reports
# GLXWindow currentGLContext errors.
qt_platform_dir="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
if ! compgen -G "${qt_platform_dir}/libqwayland*.so" >/dev/null; then
  printf 'Qt Wayland platform plugin is not installed.\n' >&2
  printf 'Available GUI fallback for this project: scripts/run_static_water_gui_ogre1.sh\n' >&2
  printf 'To add Qt Wayland support on Ubuntu 20.04, install qtwayland5.\n' >&2
  exit 2
fi

unset DISPLAY
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"

exec "$(dirname "${BASH_SOURCE[0]}")/run_static_water.sh" "$@"
