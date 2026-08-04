# 标况基线批量结论 (Baseline Batch Conclusions)

- 日期: 2026-08-04
- 批次规模: 21 场景（10× baseline_repeat00..09 + 5× dist_{off,calm,mild,strong,asymmetric} + 4× offset_{0,1,2,3}.0m + nonideal_on + platform_0.3ms）
- 汇总: SUMMARY: 19/21 LANDED
- 世界: static_water_takeoff @ GZ_PARTITION=coaxial_uav_static_water（headless）

## 分组结果表（来自 batch_summary.md）

| 分组 | n | 成功 | 成功率 | 触水 vz(m/s) | 落点偏差(m) | 最大横滚(deg) | 中止原因 |
|---|---|---|---|---|---|---|---|
| baseline | 10 | 10 | 1.00 | -0.0378±0.0013 | 0.0037±0.0011 | 8.6336±0.0483 |  |
| dist_asymmetric | 1 | 1 | 1.00 | -0.0373±0.0 | 0.0015±0.0 | 8.7593±0.0 |  |
| dist_calm | 1 | 1 | 1.00 | -0.0429±0.0 | 0.0043±0.0 | 8.6085±0.0 |  |
| dist_mild | 1 | 1 | 1.00 | -0.0392±0.0 | 0.0037±0.0 | 8.5784±0.0 |  |
| dist_off | 2 | 2 | 1.00 | -0.0372±0.0004 | 0.0032±0.0003 | 8.6034±0.0045 |  |
| dist_strong | 1 | 0 | 0.00 | — | — | — |  |
| nonideal | 1 | 0 | 0.00 | — | — | — |  |
| offset_1.0m | 1 | 1 | 1.00 | -0.0393±0.0 | 0.0109±0.0 | 8.6524±0.0 |  |
| offset_2.0m | 1 | 1 | 1.00 | -0.0367±0.0 | 0.0043±0.0 | 8.5879±0.0 |  |
| offset_3.0m | 1 | 1 | 1.00 | -0.0371±0.0 | 0.0014±0.0 | 8.7196±0.0 |  |
| platform | 1 | 1 | 1.00 | -0.0382±0.0 | 0.19±0.0 | 9.2212±0.0 |  |

## 关键观察

- **成功组**: 所有 baseline、dist_{off,calm,mild,asymmetric}、offset_{1,2,3}.0m、platform_0.3ms 全部 LANDED；基线 10/10 重复一致。
- **失败/超时**:
  - **dist_strong → TIMEOUT**: 起飞稳定正常（stabilize 2.43s，hover 误差 -0.067m）；降落状态机推进到 CONTACT_CONFIRM / NEAR_WATER 后持续振荡约 20s（状态计数 CONTACT_CONFIRM 134、NEAR_WATER 116），未能进入 LANDED，控制状态流随之不可用而中止。触水 vz=-0.0226、落点偏差 0.0075m（本身达标，仅最终 LANDED 确认未达成）。
  - **nonideal_on → TAKEOFF_TIMEOUT**: 已知问题。加入非理想性（推力偏差/配平等）后 30s 稳定窗口内无法在 target_z=0.8±0.08m 带内连续 5 次采样（hover_error=null，stabilize 16.26s 未达标）。需放宽稳定窗口或加 ki。
- **悬停下垂**: 高度环为纯 PD（ki=0），基线 hover_error 稳定在 **-0.073~-0.074m**（悬停在 ~0.726m，低于目标 0.8m），所有 LANDED 场景一致。这是已知的系统性稳态偏差，不影响触水判定。
- **触水安全**: 所有 LANDED 场景 touchdown vz 均约 **-0.036~-0.043 m/s**，远小于触水安全阈值（~0.5 m/s），触水柔和、一致。
- **落点精度**: 静态场景落点偏差 ~1.5~11 mm，offset 3.0m 反而最小（0.0014m）；仅 platform_0.3ms 因平台以 0.3 m/s 移动，最终偏差放大到 **0.19m**（无人机成功追踪移动平台着陆）。
- **姿态包线**: 降落过程最大横滚一致在 ~8.6~9.2 deg（platform 组最大 9.22 deg），无异常翻滚；起飞阶段姿态 <0.3 deg。
