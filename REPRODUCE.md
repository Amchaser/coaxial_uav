# 仿真环境复现

本项目的验证环境为 Ubuntu 20.04、Gazebo Garden (`gz-sim 7.9.0`)和 Python 3.8。支持 WSL2 + WSLg，也可在原生 Ubuntu 20.04 上运行。

## 1. 获取项目

```bash
git clone https://github.com/Kianzzz666/coaxial_uav.git
cd coaxial_uav
```

## 2. 安装与编译

在 Ubuntu 20.04 终端执行：

```bash
scripts/install_ubuntu20_garden.sh
```

该脚本安装 Gazebo Garden、OGRE1 软件渲染依赖和 C++ 编译工具，然后编译项目插件。已安装环境可单独检查：

```bash
scripts/check_environment.sh
scripts/build_plugins.sh
```

## 3. 启动仿真

终端 1：

```bash
scripts/run_static_water_gui_ogre1.sh
```

终端 2：

```bash
scripts/run_dashboard.sh
```

控制台默认地址为 `http://127.0.0.1:8765`。如果端口已被占用，服务会打印实际地址。

Gazebo 启动脚本会写入当前 `GZ_PARTITION`，控制台会自动读取，无需手工配置。

## 4. 参数与运行数据

- `config/tuning_defaults.json`：经验证的版本化控制与降落默认参数。
- `data/runtime/`：本机保存参数和运行状态，首次启动时自动创建。
- `data/performance/`：动态测试和自动降落记录，运行时自动创建。

运行数据不纳入 Git。新克隆的项目在没有本机保存参数时，会从 `config/tuning_defaults.json` 初始化；控制台的“恢复默认参数”也使用该文件。

## WSL 图形界面

Windows 11 + WSLg 可直接运行 OGRE1 脚本。如果 Gazebo 窗口无法打开，先确认 WSLg 可用；WSL 的更新命令 `wsl --update` 应在 Windows PowerShell 中执行。
