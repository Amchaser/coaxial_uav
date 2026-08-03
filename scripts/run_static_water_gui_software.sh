#!/usr/bin/env bash
set -euo pipefail

# Fallback GUI launcher for WSL / virtualized environments where hardware
# OpenGL 3.3 is unavailable. This uses Mesa llvmpipe software rendering.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-xcb_egl}"

exec "$(dirname "${BASH_SOURCE[0]}")/run_static_water.sh" "$@"
