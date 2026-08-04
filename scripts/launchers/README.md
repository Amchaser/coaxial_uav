# Windows 桌面启动器（机器相关）

这两个 `.bat` 是 Windows 桌面的一键启动/停止脚本，用于本机 WSL2 (Ubuntu-22.04) 环境：

- `启动仿真.bat`：用 Windows Terminal 开一个窗口、两个标签页，分别启动 Gazebo（OGRE2 硬件加速）和网页控制台，并自动打开浏览器 http://127.0.0.1:5223。
- `停止仿真.bat`：调用 `scripts/stop_all.sh` 停止所有相关进程。

> **注意**：脚本内的 WSL 发行版名（`Ubuntu-22.04`）与项目路径（`/home/cxj/coaxial_uav`）是机器相关的；换机器、换用户名或换发行版时需相应修改。
