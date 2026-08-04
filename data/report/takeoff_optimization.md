# Task 12 — 起飞优化 / 高度环参数调优结论

日期: 2026-08-04。世界 `static_water_takeoff`，GZ_PARTITION=coaxial_uav_static_water（持久 headless sim）。
目标高度 0.8m，验收 |hover_error_m| ≤ 0.05m、overshoot ≤ 0.05m、最大姿态 ≤ 5°。
基线: 高度环纯 PD (kp=45, ki=0, kd=35, limit=30)，hover droop ≈ **-0.073m**（验收之外）；`nonideal_on` TAKEOFF_TIMEOUT。

## 扫描方法与路径

- **batch_scan.py 实测路径不可用（已定位根因，未修改生产代码）**：`batch_scan.py` 每个组合用
  `gz sim -s <world>`（**缺 `-r`**）启动独立分区 sim，而 `gz sim` 缺 `-r` 时服务端**保持暂停**
  （`gz sim --help` 明确 `-r  Run simulation on start`）。实测 `/clock` 在无 `-r` 时 `real {} / sim {}`
  恒为空（sim 时间不前进），有 `-r` 时正常前进。`GazeboPluginController` 只发 config/motor 话题、
  不发送 play 命令，故每个组合都会 TAKEOFF_TIMEOUT。→ 回退到已验证的持久 sim + 顺序 `run_one_flight.py`。
- 持久 headless sim 实测实时率 ~0.53x（sim 96.8s / real 182s），故 30s 墙钟稳定窗口 ≈ 16s sim，
  对正常起飞（~2s）裕量充足。

## 参数表（combo → hover_error / overshoot / 最大姿态 / stabilize）

第一轮 grid: ki∈{0.5,1,2} × kp∈{45,60}（非理想关）

| combo | hover_error_m | overshoot_m | max_roll_deg | max_pitch_deg | stabilize_s |
|---|---|---|---|---|---|
| height_ki0.5_kp45 | -0.0744 | 0.0 | 0.055 | 0.047 | 2.717 |
| height_ki1.0_kp45 | -0.0737 | 0.0 | 0.057 | 0.026 | 2.729 |
| height_ki2.0_kp45 | -0.0736 | 0.0 | 0.047 | 0.043 | 2.729 |
| height_ki0.5_kp60 | -0.0620 | 0.0 | 0.072 | 0.035 | 1.988 |
| height_ki1.0_kp60 | -0.0640 | 0.0 | 0.085 | 0.158 | 1.972 |
| height_ki2.0_kp60 | -0.0637 | 0.0 | 0.115 | 0.066 | 1.972 |

第二轮（ki=1.0，提高 kp 作为主要杠杆）:

| combo | hover_error_m | overshoot_m | max_roll_deg | max_pitch_deg | stabilize_s |
|---|---|---|---|---|---|
| height_ki1.0_kp75 | -0.0544 | 0.0 | 0.127 | 0.064 | 1.669 |
| **height_ki1.0_kp90** | **-0.0415** | **0.0** | **0.227** | **0.044** | **1.617** |
| height_ki1.0_kp110 | -0.0180 | 0.0127 | 0.109 | 0.109 | 1.617 |

重复性校验:

| combo | hover_error_m | overshoot_m | stabilize_s |
|---|---|---|---|
| height_ki1.0_kp90_r1 | -0.0422 | 0.0 | 1.613 |

## 关键发现

1. **ki 对 hover_error 指标几乎无效**：kp 相同族内，ki=0.5/1.0/2.0 的 hover_error 完全一致
   （kp45 族 ≈ -0.074，kp60 族 ≈ -0.062）。原因有二：
   - 插件高度积分被 `height_integral_limit=0.5` 钳制（SDF 值；`_publish_config` 对 height 只发布
     kp/ki/kd/limit，**不发 integral_limit**，故 config-json 无法放大积分上限）。ki=2.0 时积分最大
     推力增量 = 2.0×0.5 = 1.0N，远小于悬停所需 P 项（kp45 时 45×0.074≈3.3N）。
   - 指标在**稳定窗口早期**（~2s，升空后 5 个连续带内样本）测量，积分尚未蓄积到位。
2. **kp 是唯一有效杠杆**：hover droop 随 kp 升高而降低（45→-0.074，60→-0.062，75→-0.054，
   90→-0.042），近似 droop ≈ 2.16/kp + 0.026。机理是早窗指标捕捉到的是“入带高度”：
   更高 kp 使爬升更快、入带时更高。
3. **kp=90 出现轻微抬升**（kp110 才有可测 overshoot 0.013m），kp90 全程 overshoot=0、无振荡
   （landing 阶段 z 平滑收敛至 0.80±0.004m）。max attitude 全程 < 0.3°。

## 选定参数（最优组合）

**height: kp = 90.0, ki = 1.0（kd=35, limit=30 不变）**

- 理想（非理想关）: hover_error **-0.042m**（验收内），overshoot 0.0，max attitude 0.23°，
  stabilize 1.62s（重复 -0.0422m 稳定）。
- 非理想开: 见下节。

## 非理想复测（`--nonidealities`，kp=90 / ki=1.0）

| tag | outcome | hover_error_m | overshoot_m | stabilize_s | max_roll/pitch_deg |
|---|---|---|---|---|---|
| nonideal_on（基线旧值） | **TAKEOFF_TIMEOUT** | null | 0.0 | 16.26 | 0.275/0.274 |
| nonideal_fix | **LANDED** | **-0.0362** | 0.0 | 2.347 | 0.239/0.220 |
| nonideal_fix_r1 | LANDED | -0.0348 | 0.0 | 2.351 | 0.228/0.181 |

结论：旧 `nonideal_on` 的 30s 稳定超时**已解决**。原失败原因是纯 PD 下悬停均值 ~0.726m 紧贴
±0.08m 判定带下沿，叠加非理想噪声/延迟后在带内外摆动，5 次连续带内永不达成；kp 提高到 90 后
悬停均值抬到 ~0.76m+，噪声摆动不再越界，1~2s 即稳定，并正常降落（落点水平误差 ~4.6mm）。

## Task 13 交接（降落运行是否需要携带本高度配置）

**需要**。理由：
- 选定高度环（kp=90, ki=1.0）在理想与非理想下均显著改善悬停精度，且对降落无副作用——
  降落阶段监控数据显示最终落点水平误差 ~2.6mm（理想）/ ~4.6mm（非理想），触水 vz、姿态均达标，
  与基线 kp=45 的降落表现相当或更优。
- 降落序列包含 high_hover → approach 等悬停环节，更高的悬停精度降低进场起始误差，
  有利于 landing 状态机通过 height_tolerance 等检查。
- 若 Task 13 需对比“原配置 vs 优化配置”的降落表现，请用 `--config-json '{"height":{"kp":90.0,"ki":1.0}}'`
  传入（`persist=False`，不会污染 data/runtime/tuning_config.json，本次已确认持久化文件未被改动）。
