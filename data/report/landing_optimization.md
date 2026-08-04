# Task 13 降落参数优化（L1-L5）结论

## 概览

定点降落参数优化，全部通过 batch_scan.py（已修复 -r）真机仿真运行。核心目标：
1. 修复 `dist_strong` 扰动下着陆无法确认 LANDED 的问题（Task 11 唯一失败场景，CONTACT_CONFIRM/NEAR_WATER 振荡约 20s 后 TIMEOUT）。
2. 在标况下确认/微调下降剖面（低速冲击、低误差、合理时长）。
3. 移动平台定点误差对比基线（0.19m）。

**结论：dist_strong 从「必 TIMEOUT」修复为 4/4 种子稳定 LANDED；标况 touchdown vz 保持在基线水平（约 -0.035）；移动平台定点误差 0.19m → 0.151m。**

## 确认的降落参数键（whitelist + sanitize 范围）

以下键同时通过 `scripts/batch_scan.py` 的 `_LANDING_KEYS` 白名单校验（未知键 → 硬错误）与
`dashboard/server.py::sanitize_landing` 的取值范围钳制：

| 键 | sanitize 范围 | 默认值 | 本次采用 |
|---|---|---|---|
| `landing.descent_rate_m_s` | (0.05, 0.8) | 0.6 | **0.3** |
| `landing.touchdown_max_vz_m_s` | (0.05, 0.5) | 0.25 | **0.3** |
| `landing.flare_clearance_m` | (0.15, 1.0) | 0.35 | **0.15** |
| `landing.flare_rate_m_s` | (0.03, 0.3) | 0.17 | **0.2** |
| `landing.settling_vertical_speed_limit_m_s` | (0.02, 0.3) | 0.08 | **0.15** |
| `landing.settling_time_s` | (0.1, 3.0) | 0.5 | **0.35** |
| `height.kp` / `height.ki` | — | 45 / 0 | **90.0 / 1.0**（Task 12 结论） |

注意：所有降落速率为**正值幅值**（sanitize 钳制为 0.05~0.8），任务描述中的负号仅为符号习惯，本仓
下降/触地速率按幅值处理，负值会被钳到最小值 0.05。

## 参数扫描过程

### L1/L2 — 标况下降剖面（baseline off，height kp90/ki1.0 固定）

`descent_rate_m_s × touchdown_max_vz_m_s`（默认 flare 参数）：

| descent | tdv_max | outcome | duration_s | touchdown_vz | h_err(m) |
|---|---|---|---|---|---|
| 0.3 | 0.3 | LANDED | 11.79 | -0.026 | 0.0036 |
| 0.3 | 0.5 | LANDED | 11.78 | -0.104 | 0.0036 |
| 0.5 | 0.3 | LANDED | 11.73 | -0.112 | 0.0037 |
| 0.5 | 0.5 | LANDED | 11.73 | -0.110 | 0.0037 |
| 0.7 | 0.3 | LANDED | 11.68 | -0.115 | 0.0037 |
| 0.7 | 0.5 | LANDED | 11.68 | -0.116 | 0.0037 |

选择：`descent_rate_m_s=0.3, touchdown_max_vz_m_s=0.3`（最低触地 vz -0.026）。更快的下降（0.5/0.7）
把触地 vz 抬到 -0.11~-0.12；放宽容限（0.5）也只提高实测触地冲击、无确认收益，故保留收紧的 0.3。

### L4 Pass 1 — 强扰动 flare 参数（dist_strong，seed 20260804，desc=0.6 默认）

`flare_clearance_m × flare_rate_m_s`：

| flare_clearance | flare_rate | outcome | duration_s | touchdown_vz | h_err(m) |
|---|---|---|---|---|---|
| 0.15 | 0.25 | LANDED | 11.00 | -0.016 | 0.0029 |
| 0.15 | 0.30 | LANDED | 10.70 | -0.075 | 0.0027 |
| 0.20 | 0.25 | LANDED | 11.17 | -0.032 | 0.0030 |
| 0.20 | 0.30 | LANDED | 11.11 | -0.018 | 0.0030 |

对比基线 `dist_strong`（Task 11 默认参数）：**TIMEOUT**，duration 20.3s，从未进入 LANDED。
4 个组合全部 LANDED → 降低 flare_clearance（0.15）＋ 提高 flare_rate（0.25）即可让强扰动确认通过。
最低触地 vz 组合：fc=0.15, fr=0.25。

### 中间组合（desc0.3/tdv0.3/fc0.15/fr0.25，未改 settling）多种子复验

| seed | outcome | duration_s | touchdown_vz |
|---|---|---|---|
| 20260804 (pre) | LANDED | 14.42 | -0.013 |
| 20260811 | LANDED | 14.43 | -0.014 |
| **20260812** | **TIMEOUT** | **20.65** | -0.011 |
| 20260813 | LANDED | 14.47 | -0.012 |

状态机轨迹显示：循环为 `CONTACT_CONFIRM → SETTLING → NEAR_WATER`，已能进入 SETTLING 但
在波浪浮沉中无法维持 0.5s 稳定窗口（|vz|>0.08 或浮子入水低于阈值）→ 弹回 NEAR_WATER。
→ 说明仅调 flare 不够，还需放宽 SETTLING 判定。

### L4 Pass 2 — 强扰动 settling 参数（dist_strong，**失败种子 20260812**，flare fc0.15/fr0.25 固定）

`settling_vertical_speed_limit_m_s × settling_time_s`：

| settling_vz_limit | settling_time | outcome | duration_s | touchdown_vz | h_err(m) |
|---|---|---|---|---|---|
| 0.15 | 0.25 | LANDED | 11.37 | -0.051 | 0.0032 |
| **0.15** | **0.35** | **LANDED** | **12.82** | **-0.011** | **0.0022** |
| 0.20 | 0.25 | LANDED | 11.37 | -0.050 | 0.0032 |
| 0.20 | 0.35 | LANDED | 12.82 | -0.011 | 0.0021 |

选择：`settling_vertical_speed_limit_m_s=0.15, settling_time_s=0.35`（在失败种子上 4/4 LANDED）。
settling_vz_limit 0.15 vs 0.2 无差异，取更保守的 0.15；settling_time 0.35 比 0.25 触地更柔和。

### flare_rate 精修（fr 0.25 → 0.2）

fr=0.25 在标况（baseline off）触地 vz 高达 **-0.168**（约为基线 -0.037 的 4.5 倍，因 flare 更快）。
测 fr=0.2 是否既保强扰动鲁棒又降标况冲击：

| 场景 | flare_rate | outcome | duration_s | touchdown_vz |
|---|---|---|---|---|
| dist_strong seed 20260812 | 0.2 | LANDED | 11.94 | -0.018 |
| baseline off | 0.2 | LANDED | 11.06 | -0.035 |

fr=0.2 在失败种子上仍 LANDED，且标况触地 vz 回到基线水平（-0.035）。**采用 flare_rate_m_s=0.2**。

## 最终降落参数配置

| 分组 | 参数 | 值 | 说明 |
|---|---|---|---|
| height | kp | **90.0** | Task 12 结论（悬停下垂 -0.041m） |
| height | ki | **1.0** | 同上（kd=35, limit=30 不变） |
| landing | descent_rate_m_s | **0.3** | 低速下降，触地 vz -0.026 |
| landing | touchdown_max_vz_m_s | **0.3** | 确认门限（实测触地 vz ≪ 门限） |
| landing | flare_clearance_m | **0.15** | 低离水面进 flare，强扰动确认关键 |
| landing | flare_rate_m_s | **0.2** | 适度 flare，标况触地不超标 |
| landing | settling_vertical_speed_limit_m_s | **0.15** | 容忍波浪浮沉 |
| landing | settling_time_s | **0.35** | 缩短稳定确认窗，强扰动下可达成 |
| landing | 其余键 | 沿用默认 | surface_mode=water 等 |

## 复验结果（最终配置，flare_rate=0.2）

### dist_strong × 4 种子

| seed | outcome | duration_s | touchdown_vz | h_err(m) |
|---|---|---|---|---|
| 20260804 | LANDED | 11.94 | -0.019 | 0.0032 |
| 20260811 | LANDED | 14.06 | -0.014 | 0.0028 |
| 20260812（原失败） | **LANDED** | 11.94 | -0.018 | 0.0031 |
| 20260813 | LANDED | 11.94 | -0.017 | 0.0032 |

### asymmetric × 3 种子

| seed | outcome | duration_s | touchdown_vz | h_err(m) |
|---|---|---|---|---|
| 20260821 | LANDED | 11.42 | -0.043 | 0.0037 |
| 20260822 | LANDED | 11.42 | -0.044 | 0.0038 |
| 20260823 | LANDED | 11.42 | -0.044 | 0.0037 |

### 标况（baseline off）

LANDED，duration 11.06s，touchdown_vz -0.035，h_err 0.0036（触地 vz 与 Task 11 基线 -0.037 持平）。

## L5 — 移动平台（platform-vx=0.3 m/s）

| 指标 | 基线（Task 11 默认配置） | 最终配置 | 变化 |
|---|---|---|---|
| outcome | LANDED | LANDED | — |
| 定点误差 h_err | 0.190 m | **0.151 m** | **-21%** |
| touchdown_vz | -0.038 | +0.024 | 持平 |
| 触地相对速度（x） | 0.074 m/s | 0.069 m/s | 持平 |
| 最后 3s 相对速度均值 | 0.065 | 0.049 | 略优 |
| duration | 11.3s | 10.7s | 略短 |

移动平台定点误差 0.19m → 0.151m（改善约 21%），触地相对速度约 0.07 m/s（无人机水平速度已匹配
平台 0.3 m/s 运动）。误差主要来自下降阶段对移动目标的跟踪滞后，属平台追踪固有量。

## 数据归档

- 扫描/复验 runs：`data/batch/l4_scan_*`、`l12_scan_*`、`l4p2_scan_*`、`fr020_*`、
  `final2_strong_s*`、`final2_asym_s*`、`platform_final2`（最终配置，fr=0.2），以及中间证据
  `final_strong_r1/r2/r3`、`final_strong_s*`、`final_asym_s*`、`final_baseline_off`、
  `platform_final`（fr=0.25 中间配置）。
- 本结论：`data/report/landing_optimization.md`。
