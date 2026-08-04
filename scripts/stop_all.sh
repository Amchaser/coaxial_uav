#!/usr/bin/env bash
# 停止 coaxial_uav 仿真与网页控制台进程。
# 被 Windows 桌面「停止仿真.bat」调用，也可在 WSL 内直接运行。
# 本脚本自身命令行不包含下列匹配串，避免 pkill 误杀自身。
set -u

echo '正在停止仿真与控制台进程...'
pkill -f 'static_water_takeoff.sdf' 2>/dev/null   # gz sim 主进程
pkill -f 'gz sim server' 2>/dev/null              # 仿真服务端
pkill -f 'gz sim gui' 2>/dev/null                 # 仿真 GUI
pkill -f 'gz_config_publisher' 2>/dev/null        # 参数发布器
pkill -f 'dashboard/server.py' 2>/dev/null        # 网页控制台

sleep 1
left="$(pgrep -fc 'static_water_takeoff.sdf|gz sim server|gz sim gui|gz_config_publisher|dashboard/server.py' 2>/dev/null || true)"
if [ "${left:-0}" -gt 0 ] 2>/dev/null; then
  echo "有 ${left} 个进程未退出，强制结束..."
  pkill -9 -f 'static_water_takeoff.sdf' 2>/dev/null
  pkill -9 -f 'gz sim server' 2>/dev/null
  pkill -9 -f 'gz sim gui' 2>/dev/null
  pkill -9 -f 'gz_config_publisher' 2>/dev/null
  pkill -9 -f 'dashboard/server.py' 2>/dev/null
fi
echo '完成：仿真与控制台进程已停止。'
