#!/usr/bin/env bash
# 起降全程视频录制：记录 tlog，并用录像相机把一次起降渲染成 mp4。
# Usage: ./scripts/record_flight.sh --tag <tag> [run_one_flight args...]
#
# 渲染原理（Garden 7.9，无 --record-video）：
#   世界 static_water_takeoff_video.sdf 中挂了一个固定录像相机
#   (recording_camera)，并加载 gz-sim-camera-video-recorder-system。
#   脚本在飞行前通过 gz service 发送 VideoRecord 开始/停止请求，
#   camera-video-recorder 在渲染线程里逐帧编码成 H.264(mp4)。
#   tlog 则由 gz sim --record 同步记录。
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_python="${HOME}/.venv-uav/bin/python"
video_dir="${project_root}/data/videos"
record_service="/recording_camera/record_video"

tag=""
declare -a flight_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag) tag="$2"; shift 2 ;;
    *) flight_args+=("$1"); shift ;;
  esac
done
if [[ -z "${tag}" ]]; then
  echo "usage: $0 --tag <tag> [run_one_flight args...]" >&2
  exit 2
fi

tag_dir="${video_dir}/${tag}"
log_dir="${tag_dir}/tlog"
mp4_path="${tag_dir}/${tag}.mp4"
mkdir -p "${tag_dir}"
rm -rf "${log_dir}" "${mp4_path}" "${tag_dir}"/[0-9]*.mp4 2>/dev/null || true

# 独立分区 + 资源/插件路径；并清掉 ~/.bashrc 里可能强制 ogre1/别的分区的设置
export GZ_PARTITION="coaxial_uav_video"
export GZ_SIM_RESOURCE_PATH="${project_root}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${project_root}/build/plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
unset GZ_GUI_RENDER_ENGINE GZ_GUI_RENDER_ENGINE_GPU LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER 2>/dev/null || true

# 收尾：只杀掉本脚本启动的 gz 进程（其 environ 带 GZ_PARTITION=coaxial_uav_video），
# 不影响其它分区的仿真。gz sim 的 server/gui 子进程会各自建新进程组，
# 所以按分区匹配 /proc/*/environ 更可靠。
kill_recording_gz() {
  local pids pid
  pids="$(pgrep -f 'gz sim' 2>/dev/null || true)"
  for pid in ${pids}; do
    if tr '\0' '\n' < "/proc/${pid}/environ" 2>/dev/null \
        | grep -qx 'GZ_PARTITION=coaxial_uav_video'; then
      kill "$@" "${pid}" 2>/dev/null || true
    fi
  done
}

# 1) 启动仿真（GUI 渲染 + 录像相机 + tlog 记录）。
#    - camera-video-recorder 会把临时视频写到进程 CWD 再 move 到目标路径；
#      跨文件系统（如 /mnt/c、/mnt/d）的 move 会静默失败，因此 cd 到输出
#      目录（ext4），mp4 目标也在同一目录内。
#    - --record-period 0.1 把 tlog 压到约 10Hz，避免单个录制数百 MB。
cd "${tag_dir}"
gz sim --record --record-period 0.1 --record-path "${log_dir}" -r -v 1 \
  "${project_root}/worlds/static_water_takeoff_video.sdf" &
sim_pid=$!
trap 'kill_recording_gz -9 2>/dev/null || true' EXIT

# 2) 等待仿真就绪（录像服务出现）
ready=0
for _ in $(seq 1 90); do
  if gz service -l 2>/dev/null | grep -q "${record_service}"; then
    ready=1
    break
  fi
  sleep 2
done
if [[ "${ready}" != "1" ]]; then
  echo "ERROR: gz sim 未在预期时间内就绪（未发现 ${record_service}）" >&2
  exit 1
fi
sleep 5  # 让相机渲染线程跑起来

# 3) 开始录像（带重试，规避启动期偶发的响应路由失败）
started=0
for _ in 1 2 3; do
  if gz service -s "${record_service}" \
      --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 10000 \
      --req "start: true, format: \"mp4\", save_filename: \"${mp4_path}\"" \
      >/dev/null 2>&1; then
    started=1
    break
  fi
  sleep 3
done
if [[ "${started}" != "1" ]]; then
  echo "WARN: 录像启动请求失败；本次仅记录 tlog。" >&2
fi

# 4) 跑一次起降（同一分区）
set +e
"${venv_python}" "${project_root}/scripts/run_one_flight.py" \
  --partition "${GZ_PARTITION}" --tag "${tag}" "${flight_args[@]}"
flight_rc=$?
set -e

# 5) 停止录像
gz service -s "${record_service}" \
  --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 10000 \
  --req "stop: true" >/dev/null 2>&1 || true

# 6) 等 camera-video-recorder 把临时文件 move 到 mp4_path（异步，最多等 60s）
for _ in $(seq 1 30); do
  if [[ -s "${mp4_path}" ]]; then
    break
  fi
  sleep 2
done

# 7) 停仿真：TERM 让 server 优雅收尾（写 tlog），必要时再 KILL
kill_recording_gz -TERM 2>/dev/null || true
sleep 3
kill_recording_gz -KILL 2>/dev/null || true
trap - EXIT

echo "tlog: ${log_dir}"
if [[ -s "${mp4_path}" ]]; then
  echo "video: ${mp4_path}"
else
  echo "INFO: 未生成 mp4。可用 GUI 播放 tlog 手动渲染："
  echo "  cd ${project_root}"
  echo "  gz sim --playback ${log_dir}"
  echo "在 GUI 右下角打开 VideoRecorder 录像按钮，选择 mp4/ogv 即可（或按记录快捷键）。"
fi
exit "${flight_rc}"
