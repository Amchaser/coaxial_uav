# Coaxial UAV Water Landing Simulation

基于 Gazebo Garden 的共轴无人机水上起降仿真与浏览器控制台。项目包含姿态、速度、位置闭环，动态性能测试，以及面向静态或移动承载面的分段自动降落。

## 环境

- Ubuntu 20.04（原生或 WSL2）
- Gazebo Garden / `gz-sim 7`
- Python 3.8+
- 支持 C++17 的 `g++`

已经验证过的精确依赖版本见 [`config/environment.lock`](config/environment.lock)。

## 快速开始

首次安装依赖并编译：

```bash
./scripts/install_ubuntu20_garden.sh
```

终端 1 启动 Gazebo 图形界面：

```bash
./scripts/run_static_water_gui_ogre1.sh
```

终端 2 启动控制台：

```bash
./scripts/run_dashboard.sh
```

然后打开终端中显示的本地控制台地址。干净克隆会使用 `config/tuning_defaults.json` 中随版本发布的调参结果；个人运行数据、测试曲线和构建产物不会提交到仓库。

完整安装步骤、WSL2 图形界面说明和故障排查见 [`REPRODUCE.md`](REPRODUCE.md)。

## 说明

该项目用于仿真研究和控制算法验证。将参数迁移到真实飞行器前，仍需依据实际质量、惯量、执行器能力和传感器特性重新辨识，并逐级完成台架与受控飞行测试。
