# Coaxial UAV Water Landing Simulation

基于 Gazebo Garden 的共轴无人机水上起降仿真与浏览器控制台。项目包含姿态、速度、位置闭环，动态性能测试，以及面向静态或移动承载面的分段自动降落。

> 本项目支持 **Windows 一键启动**（桌面 `.bat` 启动器）与 **GPU 硬件加速渲染**，详见下文「快速开始」与「渲染引擎」。

## 环境要求

- 系统：Ubuntu 20.04 / 22.04（原生或 WSL2）。Windows 用户推荐 WSL2 + Windows Terminal。
- Gazebo Garden / `gz-sim 7`
- Python 3.8+
- 支持 C++17 的 `g++`

已验证的精确依赖版本见 [`config/environment.lock`](config/environment.lock)。

## 快速开始

### 方式一：Windows 一键启动（推荐）

前置条件：已安装 WSL2 + Ubuntu，项目位于 WSL 家目录 `~/coaxial_uav`，并已完成首次安装（见下文「首次安装」）。

1. 双击 `scripts/launchers/` 下的 **`启动仿真.bat`**（建议复制到桌面方便双击）：
   - 自动打开一个 Windows Terminal 窗口，两个标签页：`Gazebo`（仿真）与 `Dashboard`（网页控制台）。
   - 自动在浏览器打开控制台：http://127.0.0.1:5223
2. 双击 **`停止仿真.bat`**：一键停止所有相关进程，避免后台残留占用 CPU。

> ⚠️ `.bat` 内的 WSL 发行版名（`Ubuntu-22.04`）与项目路径（`/home/cxj/coaxial_uav`）是**机器相关**的，换机器、换用户名或换发行版时需对应修改。

### 方式二：手动终端启动

终端 1 —— 启动 Gazebo 仿真（GPU 加速版）：

```bash
./scripts/run_static_water_gui_ogre2.sh
```

终端 2 —— 启动网页控制台：

```bash
./scripts/run_dashboard.sh
```

浏览器打开 **http://127.0.0.1:5223** 即可使用控制台。

停止仿真：运行 `./scripts/stop_all.sh`，或在终端里直接 Ctrl+C。

## 渲染引擎：GPU 加速 vs 软件渲染

项目提供两个 Gazebo 启动脚本，用法完全相同，按需选择：

| 脚本 | 渲染方式 | 适用场景 |
|------|----------|----------|
| `run_static_water_gui_ogre2.sh` | OGRE2 + d3d12 **硬件加速** | WSLg 下可用 RTX 等独立显卡，画面流畅（**推荐**） |
| `run_static_water_gui_ogre1.sh` | OGRE1 + llvmpipe **软件渲染** | 无 GPU，或 OGRE2 出现黑屏 / 贴图异常时使用（兼容性最好） |

切换方法：替换命令里的脚本名即可。桌面「启动仿真.bat」默认使用 GPU 版（ogre2）；如需切回软件版，把 `.bat` 里对应位置的脚本名改成 `run_static_water_gui_ogre1.sh`。

> 判断当前渲染器：查看日志 `~/.gz/auto_default.log` 中的 `Loading plugin [gz-rendering-ogre2]`（GPU）或 `[gz-rendering-ogre1]`（软件）。

## 首次安装（一次性）

- **Ubuntu 20.04**：直接运行 `./scripts/install_ubuntu20_garden.sh`。
- **Ubuntu 22.04**：官方脚本仅支持 20.04，请手动添加 OSRF 源并安装 Gazebo Garden：

```bash
sudo apt-get update && sudo apt-get install -y build-essential curl gnupg libgl1-mesa-dri lsb-release mesa-utils pkg-config python3
sudo install -d -m 0755 /usr/share/keyrings
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update && sudo apt-get install -y gz-garden libgz-rendering7-ogre1
```

安装完成后验证环境并编译插件：

```bash
./scripts/check_environment.sh   # 应输出 Environment check passed.
./scripts/build_plugins.sh
```

## 网页控制台

- 默认地址：http://127.0.0.1:5223
- 端口可在 `dashboard/server.py` 的 `--port` 参数修改（多人共用同一环境时建议各自改端口，避免冲突）。
- 控制台功能：姿态 / 速度 / 位置闭环控制、动态性能测试、静态或移动承载面的分段自动降落，以及各项实时状态显示。

## 说明

该项目用于仿真研究和控制算法验证。将参数迁移到真实飞行器前，仍需依据实际质量、惯量、执行器能力和传感器特性重新辨识，并逐级完成台架与受控飞行测试。

## 第二阶段：批量测试、图表与视频录制

- `scripts/batch_runner.py`：按场景矩阵批量跑起降，产物写入 `data/batch/<tag>/`（meta.json + samples.csv）。
- `analysis/analyze.py`：批量结果聚合统计。
- `analysis/plot_report.py`：报告图表（6 类图），写入 `data/report/`。

### 视频录制：record_flight.sh

一键录制一次起降全程，输出 mp4 视频 + tlog 日志：

```bash
./scripts/record_flight.sh --tag dist_strong --disturbance-preset strong
```

- **产物**：`data/videos/<tag>/<tag>.mp4`（视频）与 `data/videos/<tag>/tlog/`（tlog，SQLite 格式）。
- **原理**：Garden 7.9 不支持 `gz sim --record-video`。世界 `worlds/static_water_takeoff_video.sdf`（基础世界 + 固定录像相机 `recording_camera`）挂了 `gz-sim-camera-video-recorder-system`；脚本在飞行前/后用 `gz service` 发送 `VideoRecord` 开始/停止请求，录像相机在渲染线程逐帧编码成 H.264(mp4)；`gz sim --record --record-period 0.1` 同步记录 tlog。
- **说明**：
  - 录制需要 GPU/渲染线程，仿真以 GUI 模式运行（会弹出 Gazebo 窗口），不用 `--headless-rendering`（对复杂世界会挂起）。
  - 相机位置/分辨率可在 `static_water_takeoff_video.sdf` 的 `recording_camera` 模型中调整。
  - 脚本收尾只杀自己用分区 `coaxial_uav_video` 启动的 gz 进程，不影响其它分区的仿真。
- **GUI 兜底**：若未自动生成 mp4，可手动渲染 tlog：

  ```bash
  gz sim --playback data/videos/<tag>/tlog
  ```

  在 GUI 右下角打开 VideoRecorder 录像按钮，选择 mp4/ogv 保存即可（或使用记录快捷键）。


## 项目进展：起降控制优化与最终验证（2026-08-05）

在第二阶段工具链基础上完成起飞/降落参数优化与全流程验证（Task 11 基线 → Task 12 高度环 → Task 13 降落 → Task 14 最终验证）。

### 优化历程与关键发现

| 阶段 | 问题 | 根因/发现 | 解决 |
|---|---|---|---|
| 基线批量 | 悬停下垂 -0.073m；dist_strong 降落超时；nonideal_on 起飞超时 | 高度环纯 PD（ki=0）；强扰动下波浪摆动破坏 SETTLING 判定窗；非理想性下 30s 稳定窗不足 | 见下两行 |
| 高度环优化 | 悬停下垂超 ±5cm 验收 | kp 是主要杠杆（下垂∝1/kp）；ki 受 integral_limit=0.5 钳制、早窗作用小 | kp 45→90、ki 0→1.0，下垂收敛 -0.041m |
| 降落优化 | dist_strong 无法确认 LANDED | CONTACT_CONFIRM→SETTLING→NEAR_WATER 循环，波浪浮沉破坏 0.5s/0.08m/s 稳定窗 | 调 flare+settling 参数，4/4 种子稳定 LANDED |
| 非理想复测 | nonideal_on 起飞超时 | 纯 PD 悬停均值贴近判定带下沿，叠加噪声/延迟越界 | kp=90 后 1~2s 稳定并正常降落 |

### 最终参数（已写入控制台默认预设 `config/tuning_defaults.json`）

```json
{"height":{"kp":90.0,"ki":1.0},
 "landing":{"descent_rate_m_s":0.3,"touchdown_max_vz_m_s":0.3,"flare_clearance_m":0.15,
            "flare_rate_m_s":0.2,"settling_vertical_speed_limit_m_s":0.15,"settling_time_s":0.35}}
```

### 最终验证结果（6 组 × 5 架次 = 30，全部 LANDED）

| 组 | 成功率 | 触水 vz (m/s) | 落点偏差 (m) | 悬停误差 (m) |
|---|---|---|---|---|
| 标况 | 100% | -0.035 | 0.0036 | -0.033 |
| strong 扰动 | 100% | -0.015 | 0.0030 | -0.041 |
| asymmetric 扰动 | 100% | -0.043 | 0.0037 | -0.044 |
| 非理想性 | 100% | -0.221 | 0.0036 | -0.045 |
| 3m 偏移 | 100% | -0.034 | 0.0045 | -0.035 |
| 移动平台 0.3m/s | 100% | +0.024 | 0.147 | -0.032 |

**验收结论**：起飞成功率 100%、起飞姿态 <1°（标况）、悬停 ±5cm、落点偏差 <0.3m、触水 vz <0.35m/s、侧翻率 0 —— 全部达标。

### 交付物清单（供实验报告与 PPT 参考）

- **实验报告**：`reports/experiment_report.md`（完整实习报告）、`reports/final_validation.md`（验收表）、`reports/takeoff_optimization.md`、`reports/landing_optimization.md`
- **图表**：`data/report/` 下 `success_rate.png`、`landing_scatter.png`、`touchdown_safety.png`、`disturbance_boxplot.png`、`timeseries_fv_baseline_00.png`、`timeseries_fv_dist_strong_00.png`
- **视频**（人工录制，最终参数）：`data/videos/My_videos/`
  - `静水+理想飞机模型+静态目标落点仿真视频.mp4`（68.6 MB）
  - `强扰动+非理想飞机+移动目标点降落仿真视频.mp4`（78.3 MB）
- **数据**：`data/fv_batch/`（30 架次原始 meta.json + samples.csv）、`reports/fv_metrics.csv`（分组指标汇总）

### 本阶段新增/修改文件

- **新增工具链**：`scripts/reset_pose.py`、`scripts/run_one_flight.py`、`scripts/batch_runner.py`、`scripts/batch_scan.py`、`scripts/record_flight.sh`、`analysis/analyze.py`、`analysis/plot_report.py`、`worlds/static_water_takeoff_video.sdf`
- **新增测试**：`tests/`（48 个单元测试）
- **修改配置**：`config/tuning_defaults.json`（控制台默认预设更新为最终调优参数）
- **新增数据/成果**：`data/fv_batch/`、`data/report/`、`data/videos/`
- **说明**：`data/batch/` 为 Task 11-13 历史批量数据（混合配置）；干净的最终验证数据以 `data/fv_batch/` 为准
