const history = [];
const attitudeWindowS = 60;
const maxHistory = 1000;

const $ = (id) => document.getElementById(id);
const fmt = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : "--";
const toDegrees = (radians) => Number(radians) * 180 / Math.PI;
const rounded = (value, digits = 6) => {
  if (!Number.isFinite(value)) return value;
  const factor = 10 ** digits;
  return Math.round(value * factor) / factor;
};
let formLoaded = false;
let activePidTab = "attitude";
let lastTestAxis = null;
let activeWorkspace = "flight";
let latestDashboardData = null;
let configDirty = false;
let lastSavedConfig = null;
let activeControlSection = "basic";
const hiddenChartSeries = new Set();
let latestCompletedTest = null;
let previousCompletedTest = null;
let unsavedNavigationIndex = 0;
const chartPointerX = {
  attitude: NaN,
  test: NaN,
};
const motorSourceLabels = {
  pid_plugin: "PID 控制器",
  joint_state: "仿真关节反馈",
  commanded: "转速命令缓存",
  controller: "软件控制器",
  plugin_stop: "控制器已停止",
  unavailable: "暂无数据",
};
const testChartFallbackColors = {
  target: "#30363b",
  filteredTarget: "#74558d",
  response: "#2463a7",
  output: "#a36217",
  stepMarker: "#858c93",
  attitudeRoll: "#2463a7",
  attitudeRollTarget: "#7fa6ce",
  attitudePitch: "#a33a48",
  attitudePitchTarget: "#d48b94",
  attitudeYaw: "#8b6914",
  attitudeYawTarget: "#c7aa62",
};
const configScalars = [
  "target_z_m",
  "target_vx_m_s",
  "target_vy_m_s",
  "velocity_accel_limit_m_s2",
  "target_x_m",
  "target_y_m",
  "position_velocity_limit_m_s",
  "hover_omega_rad_s",
  "max_omega_rad_s",
  "yaw_large_signal_kp",
  "yaw_large_signal_kd",
  "rate_hz",
];
const angleConfigFields = {
  target_roll_rad: "target_roll_deg",
  target_pitch_rad: "target_pitch_deg",
  target_yaw_rad: "target_yaw_deg",
  velocity_tilt_limit_rad: "velocity_tilt_limit_deg",
  attitude_setpoint_rate_limit_rad_s: "attitude_setpoint_rate_limit_deg_s",
  yaw_schedule_start_rad: "yaw_schedule_start_deg",
  yaw_schedule_end_rad: "yaw_schedule_end_deg",
};
const pidAxes = ["height", "roll", "pitch", "yaw"];
const pidKeys = ["kp", "ki", "kd", "limit"];
const velocityPidAxes = ["velocity_x", "velocity_y"];
const velocityPidKeys = ["kp", "ki", "limit", "integral_limit"];
const positionPidAxes = ["position_x", "position_y"];
const positionPidKeys = ["kp", "ki", "kd", "limit", "integral_limit"];
const aerodynamicKeys = [
  "air_density_kg_m3",
  "drag_area_x_m2",
  "drag_area_y_m2",
  "drag_area_z_m2",
  "angular_damping_roll_nm_s",
  "angular_damping_pitch_nm_s",
  "angular_damping_yaw_nm_s",
  "wind_x_m_s",
  "wind_y_m_s",
  "wind_z_m_s",
  "gust_rms_m_s",
  "gust_correlation_time_s",
  "mass_scale",
  "inertia_scale_roll",
  "inertia_scale_pitch",
  "inertia_scale_yaw",
  "cg_offset_x_m",
  "cg_offset_y_m",
  "cg_offset_z_m",
  "seed",
];
const rotorWaterKeys = [
  "coaxial_inflow_time_constant_s",
  "water_density_kg_m3",
  "water_level_z_m",
  "float_virtual_draft_m",
  "water_linear_drag_x_n_s_m",
  "water_linear_drag_y_n_s_m",
  "water_linear_drag_z_n_s_m",
  "water_quadratic_drag_x",
  "water_quadratic_drag_y",
  "water_quadratic_drag_z",
  "water_current_x_m_s",
  "water_current_y_m_s",
  "water_current_z_m_s",
  "water_slamming_gain_n_s_m",
];
const landingKeys = [
  "platform_top_offset_m",
  "target_x_m",
  "target_y_m",
  "target_vx_m_s",
  "target_vy_m_s",
  "target_status_timeout_s",
  "target_speed_limit_m_s",
  "high_hover_z_m",
  "approach_speed_m_s",
  "cruise_speed_m_s",
  "position_tolerance_m",
  "descent_rate_m_s",
  "flare_clearance_m",
  "flare_rate_m_s",
  "touchdown_max_vz_m_s",
  "contact_confirm_s",
  "spool_down_s",
  "departure_horizontal_speed_limit_m_s",
  "departure_clearance_margin_m",
  "near_horizontal_speed_limit_m_s",
  "moving_target_correction_reserve_m_s",
  "approach_braking_accel_m_s2",
  "abort_position_error_m",
  "near_max_descent_speed_m_s",
  "go_around_height_m",
  "departure_stable_time_s",
  "align_stable_time_s",
  "hover_stable_time_s",
  "approach_relative_speed_tolerance_m_s",
  "align_relative_speed_tolerance_m_s",
  "hover_relative_speed_tolerance_m_s",
  "departure_horizontal_speed_tolerance_m_s",
  "height_tolerance_m",
  "approach_vertical_speed_tolerance_m_s",
  "precision_vertical_speed_tolerance_m_s",
  "near_overspeed_grace_s",
  "contact_submerged_fraction",
  "settling_vertical_speed_limit_m_s",
  "settling_time_s",
  "contact_loss_grace_s",
  "go_around_height_tolerance_m",
  "go_around_vertical_speed_tolerance_m_s",
  "flare_transition_margin_m",
];
const landingAngleConfigFields = {
  target_yaw_rad: "target_yaw_deg",
  target_yaw_rate_rad_s: "target_yaw_rate_deg_s",
  yaw_tolerance_rad: "yaw_tolerance_deg",
  departure_tilt_limit_rad: "departure_tilt_limit_deg",
  approach_tilt_limit_rad: "approach_tilt_limit_deg",
  near_tilt_limit_rad: "near_tilt_limit_deg",
  warning_tilt_rad: "warning_tilt_deg",
  abort_tilt_rad: "abort_tilt_deg",
  approach_abort_tilt_rad: "approach_abort_tilt_deg",
  yaw_rate_tolerance_rad_s: "yaw_rate_tolerance_deg_s",
  contact_tilt_rate_limit_rad_s: "contact_tilt_rate_limit_deg_s",
  settling_tilt_rate_limit_rad_s: "settling_tilt_rate_limit_deg_s",
  go_around_tilt_tolerance_rad: "go_around_tilt_tolerance_deg",
};
const landingStrategyPresets = {
  smooth: {
    label: "平稳",
    description: "降低下降与触水速度，适合扰动或载荷敏感场景",
    values: {
      high_hover_z_m: 1.8,
      approach_speed_m_s: 0.55,
      cruise_speed_m_s: 1.5,
      position_tolerance_m: 0.12,
      yaw_tolerance_deg: 4,
      descent_rate_m_s: 0.25,
      flare_clearance_m: 0.5,
      flare_rate_m_s: 0.08,
      touchdown_max_vz_m_s: 0.15,
      contact_confirm_s: 0.4,
      spool_down_s: 1.8,
    },
  },
  standard: {
    label: "标准",
    description: "平衡下降时间与触水平稳性",
    values: {
      high_hover_z_m: 1.8,
      approach_speed_m_s: 0.8,
      cruise_speed_m_s: 2.5,
      position_tolerance_m: 0.15,
      yaw_tolerance_deg: 5,
      descent_rate_m_s: 0.35,
      flare_clearance_m: 0.4,
      flare_rate_m_s: 0.12,
      touchdown_max_vz_m_s: 0.2,
      contact_confirm_s: 0.3,
      spool_down_s: 1.5,
    },
  },
  fast: {
    label: "快速",
    description: "缩短进场和下降时间，保留近水减速与触水保护",
    values: {
      high_hover_z_m: 1.6,
      approach_speed_m_s: 0.95,
      cruise_speed_m_s: 2.5,
      position_tolerance_m: 0.2,
      yaw_tolerance_deg: 7,
      descent_rate_m_s: 0.45,
      flare_clearance_m: 0.35,
      flare_rate_m_s: 0.16,
      touchdown_max_vz_m_s: 0.25,
      contact_confirm_s: 0.25,
      spool_down_s: 1.2,
    },
  },
};
const landingStrategyProfileKeys = Object.keys(
  landingStrategyPresets.standard.values
);
let landingStrategyProfiles = Object.fromEntries(
  Object.entries(landingStrategyPresets).map(([name, preset]) => (
    [name, {...preset.values}]
  ))
);
let activeLandingStrategy = "standard";
const landingStateLabels = {
  IDLE: "待命",
  CLIMB: "脱水爬升",
  STABILIZE: "稳定",
  APPROACH: "定位",
  ALIGN: "对准",
  HIGH_HOVER: "准备下降",
  SLOW_DESCENT: "下降",
  NEAR_WATER: "近水缓冲",
  CONTACT_CONFIRM: "触水确认",
  SETTLING: "稳定浮态",
  SPOOL_DOWN: "电机退转",
  LANDED: "已完成",
  GO_AROUND: "复飞",
  ABORTED: "已中止",
};

function formatLandingAbortReason(status, landing = {}) {
  const reason = String(
    status.landing_abort_reason ?? landing.abort_reason ?? ""
  );
  if (!reason) return "控制器未提供中止原因，请查看技术详情";
  const triggerState = String(
    status.landing_abort_trigger_state ?? landing.abort_trigger_state ?? ""
  );
  const stage = landingStateLabels[triggerState] || triggerState || "自动降落";
  const measured = Number(
    status.landing_abort_measured_value ?? landing.abort_measured_value
  );
  const limit = Number(
    status.landing_abort_limit_value ?? landing.abort_limit_value
  );
  const hasValues = Number.isFinite(measured) && Number.isFinite(limit)
    && limit > 0;
  if (reason === "platform unavailable") {
    return `${stage}阶段未检测到可用实体平台；请检查甲板模型和目标插件状态`;
  }
  if (reason === "geometry unavailable") {
    return `${stage}阶段未取得机体或承载面几何描述；请重启 Gazebo 并检查模型插件`;
  }
  if (reason === "target speed limit") {
    return hasValues
      ? `${stage}阶段漂移目标速度 ${fmt(measured, 2)} m/s，超过允许上限 ${fmt(limit, 2)} m/s；请降低目标速度或重新验证安全限值`
      : `${stage}阶段漂移目标速度超过安全上限；请降低目标速度`;
  }
  if (reason === "target status lost") {
    return hasValues
      ? `${stage}阶段目标状态已中断 ${fmt(measured, 2)} s，超过 ${fmt(limit, 2)} s 超时阈值；请检查目标插件和仿真实时因子`
      : `${stage}阶段目标状态超时；请检查目标插件和 Gazebo 连接`;
  }
  if (reason === "attitude limit") {
    return hasValues
      ? `${stage}阶段最大倾角 ${fmt(toDegrees(measured), 1)} deg，超过 ${fmt(toDegrees(limit), 1)} deg 安全限值；控制器已执行复飞`
      : `${stage}阶段姿态超过安全限值；控制器已执行复飞`;
  }
  if (reason === "position error") {
    return hasValues
      ? `${stage}阶段水平落点误差 ${fmt(measured, 2)} m，超过 ${fmt(limit, 2)} m 复飞阈值；请检查目标跟踪和制动参数`
      : `${stage}阶段水平落点误差超过复飞阈值；请检查目标跟踪`;
  }
  if (reason === "descent speed") {
    return hasValues
      ? `${stage}阶段下降速度 ${fmt(measured, 2)} m/s，超过 ${fmt(limit, 2)} m/s 近表面上限；请降低下降速度或增大缓冲高度`
      : `${stage}阶段下降速度超过近表面安全上限；请检查下降参数`;
  }
  return `${stage}阶段触发未知保护（${reason}）；请查看技术详情`;
}

function setMetric(id, value, digits = 2) {
  $(id).textContent = fmt(value, digits);
}

function clockText(seconds) {
  if (!Number.isFinite(seconds)) return "--";
  const total = Math.max(0, Math.floor(seconds));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return `${h}h ${String(m).padStart(2, "0")}m ${String(s).padStart(2, "0")}s`;
}

function syncLandingOverview() {
  document.querySelectorAll("[data-mirror-source]").forEach((target) => {
    const source = $(target.dataset.mirrorSource);
    if (!source) return;
    target.textContent = source.textContent;
    target.className = source.className;
    target.title = source.title;
  });
}

function initializeLandingOverview() {
  const observer = new MutationObserver(syncLandingOverview);
  [
    document.querySelector(".flight-status-strip[data-workspace='flight']"),
    document.querySelector(".telemetry[data-workspace='flight']"),
  ].filter(Boolean).forEach((source) => observer.observe(source, {
    attributes: true,
    childList: true,
    characterData: true,
    subtree: true,
  }));
  syncLandingOverview();
}

function updateState(data) {
  latestDashboardData = data;
  const badge = $("connection");
  if (!data.ok) {
    badge.textContent = "DISCONNECTED";
    badge.classList.remove("ok");
    const topicTimeout = String(data.error || "").includes("timed out");
    $("status").textContent = topicTimeout
      ? "Gazebo 位姿话题超时，请确认仿真仍在运行"
      : (data.error || "Gazebo 数据不可用");
    $("statusGazebo").textContent = "话题超时";
    $("statusGazebo").className = "error";
    $("statusGazebo").title = data.error || "Gazebo 数据不可用";
    return;
  }

  badge.textContent = "CONNECTED";
  badge.classList.add("ok");
  $("status").textContent = "Gazebo transport 实时采样中";
  $("statusGazebo").textContent = `已连接 · RTF ${fmt(data.stats.real_time_factor, 2)}`;
  $("statusGazebo").className = "success";
  $("statusGazebo").title = "Gazebo transport 数据正常";
  setMetric("z", data.position.z, 3);
  setMetric("roll", data.attitude.roll_deg, 2);
  setMetric("pitch", data.attitude.pitch_deg, 2);
  setMetric("yaw", data.attitude.yaw_deg, 2);
  const pluginStatus = data.motors.plugin_status || {};
  setMetric("velocityX", Number(pluginStatus.world_vx_m_s), 2);
  setMetric("velocityY", Number(pluginStatus.world_vy_m_s), 2);
  setMetric("simTime", data.stats.sim_time_s, 1);
  $("simClock").textContent = clockText(data.stats.sim_time_s);
  setMetric("rtf", data.stats.real_time_factor, 2);
  setMetric("upperMotor", data.motors.upper_rad_s, 2);
  setMetric("lowerMotor", data.motors.lower_rad_s, 2);
  const rotorWater = data.rotor_water || {};
  setMetric("coaxialLoss", Number(rotorWater.coaxial_loss_fraction) * 100, 1);
  $("waterContact").textContent =
    typeof rotorWater.water_contact === "boolean"
      ? (rotorWater.water_contact ? "接触" : "离水")
      : "--";
  $("buoyancy").textContent = `浮力 ${fmt(Number(rotorWater.buoyancy_n), 1)} N`;
  const motorSource = data.motors.source || "unavailable";
  $("motorSource").textContent = motorSourceLabels[motorSource] || "其他数据源";
  $("motorSource").title = `内部标识: ${motorSource}`;
  $("motorRpm").textContent = `${fmt(data.motors.upper_rpm, 0)} / ${fmt(data.motors.lower_rpm, 0)} RPM`;
  updateController(data.controller);
  updatePerformance(data.performance);
  updateLanding(
    data.landing || {}, pluginStatus, data.performance || {}, rotorWater
  );

  appendAttitudeHistory(data);
  drawAttitude();
  drawMap(data);
}

function setWorkspace(workspace) {
  activeWorkspace = workspace;
  document.querySelectorAll("[data-workspace]").forEach((element) => {
    element.hidden = element.dataset.workspace !== workspace;
  });
  document.querySelectorAll("[data-workspace-tab]").forEach((button) => {
    button.classList.toggle(
      "active", button.dataset.workspaceTab === workspace
    );
  });
  if (latestDashboardData) {
    requestAnimationFrame(() => {
      if (workspace === "landing") {
        drawLanding(latestDashboardData.landing || {});
      } else if (workspace === "test") {
        drawTestResponse(latestDashboardData.performance || {});
        drawTestPositionMap(latestDashboardData.performance || {});
      }
      drawMap(latestDashboardData);
    });
  }
}

function appendAttitudeHistory(data) {
  const simTime = Number(data?.stats?.sim_time_s);
  if (!Number.isFinite(simTime)) return;

  const previousTime = history.length
    ? Number(history[history.length - 1]?.stats?.sim_time_s)
    : NaN;
  if (Number.isFinite(previousTime) && simTime < previousTime - 1e-6) {
    history.length = 0;
  } else if (Number.isFinite(previousTime) && Math.abs(simTime - previousTime) <= 1e-6) {
    history[history.length - 1] = data;
    return;
  }

  history.push(data);
  const cutoff = simTime - attitudeWindowS;
  while (history.length > 1 && Number(history[0].stats.sim_time_s) < cutoff) {
    history.shift();
  }
  while (history.length > maxHistory) history.shift();
}

function loadControllerForm(controller) {
  if (!formLoaded) {
    loadForm(controller.config);
    formLoaded = true;
    $("startTest").disabled = false;
    $("restoreDefaults").disabled = false;
  }
}

function updateController(controller) {
  if (!controller) return;
  const state = $("controllerState");
  state.textContent = controller.running ? "RUNNING" : "STOPPED";
  state.classList.toggle("running", Boolean(controller.running));
  $("statusControlSummary").textContent = controller.running ? "运行中" : "已停止";
  $("statusControlSummary").className = controller.running ? "success" : "";
  loadControllerForm(controller);
  $("controllerOutput").textContent = JSON.stringify(controller.last || {}, null, 2);
}

async function initializeControllerForm() {
  try {
    const response = await fetch("/control", {cache: "no-store"});
    if (!response.ok) return;
    const controller = await response.json();
    if (controller?.config) loadControllerForm(controller);
  } catch {
    // The EventSource path retries and can initialize the form later.
  }
}

function updatePerformance(test) {
  if (!test) return;
  const state = $("testState");
  const runningStates = {
    "waiting for stable baseline": "STABILIZING",
    "braking horizontal motion": "RECOVERY",
    "restoring baseline": "RECOVERY",
    "baseline sampling": "BASELINE",
    "step sampling": "STEP",
    "dynamic performance test cleanup in progress": "STOPPING",
  };
  state.textContent = test.running
    ? `${runningStates[test.message] || "STARTING"} ${test.repeat_index || 1}/${test.repeat_count || 1}`
    : String(test.mode || "IDLE").toUpperCase();
  state.classList.toggle("running", Boolean(test.running));
  updateTestWorkflowView(test);
  $("startTest").disabled = !formLoaded || Boolean(test.running);
  $("restoreDefaults").disabled = !formLoaded || Boolean(test.running);
  $("startControl").disabled = Boolean(test.running);
  $("saveConfig").disabled = Boolean(test.running);

  const attitudeAxis = ["roll", "pitch", "yaw"].includes(test.axis);
  const metrics = test.metrics || {};
  setMetric("testOvershoot", metrics.overshoot_percent, 1);
  setMetric("testRiseTime", metrics.rise_time_s, 2);
  setMetric("testSettlingTime", metrics.settling_time_s, 2);
  setMetric(
    "testSteadyError",
    attitudeAxis ? toDegrees(metrics.steady_state_error) : metrics.steady_state_error,
    4,
  );
  const saturation = metrics.saturation_breakdown || {};
  setMetric("testPositionSaturation",
    Number(saturation.planning?.position_velocity_ratio) * 100, 1);
  setMetric("testAccelSaturation",
    Number(saturation.planning?.velocity_acceleration_ratio) * 100, 1);
  setMetric("testTorqueSaturation",
    Number(saturation.actuator?.attitude_torque_ratio) * 100, 1);
  setMetric("testMotorSaturation",
    Number(saturation.actuator?.motor_speed_ratio) * 100, 1);
  setMetric("testSamples", metrics.sample_count, 0);
  setMetric("testSampleRate", metrics.measured_sample_rate_hz, 1);
  setMetric("testBandwidth", metrics.bandwidth_hz_estimate, 2);
  setMetric("testPhaseMargin", metrics.phase_margin_deg_estimate, 1);
  setMetric("testDampingRatio", metrics.damping_ratio_estimate, 2);
  const repeatStats = test.repeat_statistics || {};
  setMetric("testRepeatCount", Array.isArray(test.repetitions) ? test.repetitions.length : NaN, 0);
  setMetric("testRiseStddev", repeatStats.rise_time_s?.stddev, 3);
  setMetric("testOvershootStddev", repeatStats.overshoot_percent?.stddev, 3);
  setMetric("testSettlingStddev", repeatStats.settling_time_s?.stddev, 3);
  updateTestComparison(test, metrics);

  const displayMetrics = {
    ...metrics,
    steady_state_error: attitudeAxis
      ? toDegrees(metrics.steady_state_error)
      : metrics.steady_state_error,
    steady_state_error_unit: attitudeAxis
      ? "deg"
      : (["vx", "vy"].includes(test.axis) ? "m/s" : "m"),
  };
  const displayRepetitions = Array.isArray(test.repetitions)
    ? test.repetitions.map((run) => {
      const {samples: runSamples, ...runSummary} = run;
      return {
        ...runSummary,
        chart_sample_count: Array.isArray(runSamples) ? runSamples.length : 0,
        initial_target: attitudeAxis ? toDegrees(run.initial_target) : run.initial_target,
        final_target: attitudeAxis ? toDegrees(run.final_target) : run.final_target,
        target_unit: attitudeAxis ? "deg" : test.requested_step_unit,
        metrics: {
          ...run.metrics,
          steady_state_error: attitudeAxis
            ? toDegrees(run.metrics?.steady_state_error)
            : run.metrics?.steady_state_error,
          steady_state_error_unit: attitudeAxis
            ? "deg"
            : (["vx", "vy"].includes(test.axis) ? "m/s" : "m"),
        },
      };
    })
    : test.repetitions;
  const latestSample = Array.isArray(test.samples) && test.samples.length
    ? test.samples[test.samples.length - 1]
    : null;
  const displayLatestSample = latestSample && attitudeAxis
    ? {
      ...latestSample,
      value: toDegrees(latestSample.value),
      target: toDegrees(latestSample.target),
      filtered_target: toDegrees(latestSample.filtered_target),
      value_unit: "deg",
    }
    : latestSample;
  const summary = {
    mode: test.mode,
    message: test.message,
    axis: test.axis,
    requested_step: test.requested_step,
    requested_step_unit: test.requested_step_unit,
    applied_step: attitudeAxis ? toDegrees(test.applied_step) : test.applied_step,
    applied_step_unit: attitudeAxis ? "deg" : test.applied_step_unit,
    repeat_count: test.repeat_count,
    repeat_statistics: test.repeat_statistics,
    repetitions: displayRepetitions,
    controller_state_restored: test.controller_state_restored,
    controller_restore_error: test.restore_error,
    saved_path: test.saved_path,
    data_csv_path: test.data_csv_path,
    metrics: displayMetrics,
    latest_sample: displayLatestSample,
  };
  $("testOutput").textContent = JSON.stringify(summary, null, 2);
  drawTestResponse(test);
  drawTestPositionMap(test);
}

function metricDeltaText(current, previous, digits, unit) {
  const delta = Number(current) - Number(previous);
  if (!Number.isFinite(delta)) return {text: "暂无可比数据", className: ""};
  const direction = delta > 0 ? "增加" : (delta < 0 ? "减少" : "无变化");
  return {
    text: delta === 0
      ? direction
      : `${direction} ${fmt(Math.abs(delta), digits)} ${unit}`,
    className: "",
  };
}

function updateTestComparison(test, metrics) {
  const completedKey = String(test.saved_path || test.data_csv_path || "");
  const hasMetrics = Number.isFinite(Number(metrics.overshoot_percent));
  if (!test.running && completedKey && hasMetrics
      && completedKey !== latestCompletedTest?.key) {
    const requestedStep = Number(test.requested_step ?? test.applied_step);
    const sameStep = Number.isFinite(requestedStep)
      && Math.abs(requestedStep - Number(latestCompletedTest?.requestedStep)) < 1e-9;
    previousCompletedTest = latestCompletedTest?.axis === test.axis && sameStep
      ? latestCompletedTest
      : null;
    latestCompletedTest = {
      key: completedKey,
      axis: test.axis,
      requestedStep,
      metrics: JSON.parse(JSON.stringify(metrics)),
    };
  }

  const previous = previousCompletedTest?.metrics;
  $("testPreviousOvershoot").textContent = previous
    ? `${fmt(Number(previous.overshoot_percent), 1)} %`
    : "--";
  $("testPreviousRiseTime").textContent = previous
    ? `${fmt(Number(previous.rise_time_s), 2)} s`
    : "--";
  $("testPreviousSettlingTime").textContent = previous
    ? `${fmt(Number(previous.settling_time_s), 2)} s`
    : "--";
  const comparisons = [
    ["testDeltaOvershoot", metrics.overshoot_percent, previous?.overshoot_percent, 1, "%"],
    ["testDeltaRiseTime", metrics.rise_time_s, previous?.rise_time_s, 2, "s"],
    ["testDeltaSettlingTime", metrics.settling_time_s, previous?.settling_time_s, 2, "s"],
  ];
  comparisons.forEach(([id, current, old, digits, unit]) => {
    const result = metricDeltaText(current, old, digits, unit);
    $(id).textContent = result.text;
    $(id).className = result.className;
  });
  $("testCompareState").textContent = previous
    ? `${String(test.axis || "").toUpperCase()} 已比较`
    : (latestCompletedTest ? "等待下一次" : "无基线");
}

function updateTestWorkflowView(test = {}) {
  const pidEdited = Boolean($("testModal").querySelector("[data-test-pid] .is-edited"));
  const anyEdited = Boolean($("testModal").querySelector(".is-edited"));
  const hasResult = Boolean(test.metrics && (test.saved_path || test.data_csv_path));
  const runningLabels = {
    "waiting for stable baseline": "正在等待稳定基线",
    "braking horizontal motion": "正在抑制水平运动",
    "restoring baseline": "正在恢复基线状态",
    "baseline sampling": "正在记录基线",
    "step sampling": "正在记录阶跃响应",
    "dynamic performance test cleanup in progress": "正在恢复测试前控制状态",
  };
  const message = test.running
    ? (runningLabels[test.message] || "正在准备动态测试")
    : (hasResult && !anyEdited
      ? "测试完成，可查看曲线、指标和上次结果差值"
      : (pidEdited
        ? "PID 已修改，确认后可启动当前通道测试"
        : "选择测试通道、阶跃幅值和重复次数"));
  $("testUserMessage").textContent = message;
  $("testUserMessage").className = "action-message";
}

function updateLanding(landing, status, performance, rotorWater = {}) {
  const running = Boolean(landing.running);
  const stateName = String(status.landing_state || landing.state || "IDLE");
  const state = $("landingState");
  state.textContent = landingStateLabels[stateName] || stateName;
  state.title = `控制器状态：${stateName}`;
  state.classList.toggle("running", running || Boolean(status.landing_active));
  const localLandingUnavailable = new Set([
    "CONTACT_CONFIRM", "SETTLING", "SPOOL_DOWN", "LANDED",
  ]).has(stateName);
  $("localLanding").disabled = !running || localLandingUnavailable;
  $("saveLandingConfig").disabled = running;
  updateLandingFieldRevertButtons();
  $("startTest").disabled = !formLoaded || running || Boolean(performance.running);
  $("startControl").disabled = $("startControl").disabled || running;
  $("saveConfig").disabled = $("saveConfig").disabled || running;
  setMetric(
    "landingClearance",
    Math.max(0, Number(status.float_clearance_m)),
    3,
  );
  setMetric("landingVz", Number(status.z_rate_m_s), 3);
  setMetric(
    "landingImpact",
    Number(status.landing_peak_impact_n ?? landing.peak_impact_n),
    1,
  );
  setMetric(
    "landingHorizontalError",
    Number(status.landing_horizontal_error_m ?? landing.final_horizontal_error_m),
    3,
  );
  setMetric("landingHorizontalSpeed", Math.hypot(
    Number(status.world_vx_m_s),
    Number(status.world_vy_m_s),
  ), 2);
  setMetric(
    "landingActiveSpeedLimit",
    Number(status.position_velocity_limit_m_s),
    2,
  );
  setMetric(
    "landingYawError",
    toDegrees(Number(status.landing_yaw_error_rad)),
    1,
  );
  const selectedMovingTarget = Boolean($("landing_moving_target_enabled").checked);
  const movingTarget = selectedMovingTarget
    || Boolean(status.landing_moving_target_enabled);
  const targetStatusKnown =
    typeof status.landing_target_healthy === "boolean";
  const targetHealthy = targetStatusKnown
    && Boolean(status.landing_target_healthy);
  $("landingTargetHealth").textContent = movingTarget
    ? (targetStatusKnown ? (targetHealthy ? "正常" : "失效") : "等待状态")
    : "静态";
  $("landingTargetVelocity").textContent = movingTarget
    ? `${fmt(Math.hypot(
      Number(status.landing_target_vx_m_s),
      Number(status.landing_target_vy_m_s),
    ), 2)} m/s`
    : "m/s";
  const currentX = Number(latestDashboardData?.position?.x);
  const currentY = Number(latestDashboardData?.position?.y);
  const targetX = number("landing_target_x_m");
  const targetY = number("landing_target_y_m");
  const landingDistance = Math.hypot(targetX - currentX, targetY - currentY);
  const targetStatus = latestDashboardData?.landing_target || {};
  const targetYaw = number("landing_target_yaw_deg") * Math.PI / 180;
  const targetDx = currentX - targetX;
  const targetDy = currentY - targetY;
  const targetCos = Math.cos(targetYaw);
  const targetSin = Math.sin(targetYaw);
  const platformLocalX = targetCos * targetDx + targetSin * targetDy;
  const platformLocalY = -targetSin * targetDx + targetCos * targetDy;
  const floatBottomOffset = Number(rotorWater.float_bottom_offset_m);
  const floatHalfLength = Number(
    rotorWater.float_footprint_half_length_m
  );
  const floatHalfWidth = Number(rotorWater.float_footprint_half_width_m);
  const platformSignedClearance = Number(latestDashboardData?.position?.z)
    - floatBottomOffset
    - (
      Number(latestDashboardData?.rotor_water?.water_level_z_m ?? 0)
      + number("landing_platform_top_offset_m")
    );
  const platformInitialOverlap =
    Math.abs(platformLocalX)
      <= Number(targetStatus.platform_half_length_m) + floatHalfLength
    && Math.abs(platformLocalY)
      <= Number(targetStatus.platform_half_width_m) + floatHalfWidth
    && platformSignedClearance
      < Number(targetStatus.initial_overlap_min_clearance_m);
  setMetric("landingDistance", landingDistance, 2);
  const nearSpeed = number("landing_approach_speed_m_s");
  const cruiseSpeed = number("landing_cruise_speed_m_s");
  const speedProfile = $("landingSpeedProfile");
  const adaptiveProfileLoaded =
    status.landing_speed_profile === "adaptive_distance_v1";
  const controllerConnected = Boolean(latestDashboardData?.ok);
  const controllerStatusAvailable = Object.keys(status).length > 0;
  $("landingSpeedPlan").textContent = Number.isFinite(nearSpeed)
    && Number.isFinite(cruiseSpeed)
    ? `自适应 ${fmt(nearSpeed, 1)} - ${fmt(cruiseSpeed, 1)} m/s`
    : "自适应 -- m/s";
  speedProfile.classList.toggle(
    "legacy",
    controllerConnected && controllerStatusAvailable && !adaptiveProfileLoaded,
  );
  if (controllerConnected && !controllerStatusAvailable) {
    $("landingSpeedPlanDetail").textContent =
      "正在等待 PID 控制器状态，不影响 Gazebo 位姿连接";
  } else if (controllerConnected && !adaptiveProfileLoaded) {
    $("landingSpeedPlanDetail").textContent =
      "当前 Gazebo 未加载自适应插件，请重启仿真";
  } else if (running && adaptiveProfileLoaded && stateName === "APPROACH") {
    $("landingSpeedPlanDetail").textContent = movingTarget
      ? `目标前馈 ${fmt(Math.hypot(
        Number(status.landing_target_vx_m_s),
        Number(status.landing_target_vy_m_s),
      ), 2)} m/s，并保留相对位置修正；当前上限 ${fmt(
        Number(status.position_velocity_limit_m_s), 2,
      )} m/s`
      : `按剩余距离调速，当前上限 ${fmt(
        Number(status.position_velocity_limit_m_s), 2,
      )} m/s`;
  } else if (running && adaptiveProfileLoaded && movingTarget) {
    $("landingSpeedPlanDetail").textContent =
      `目标前馈 ${fmt(Math.hypot(
        Number(status.landing_target_vx_m_s),
        Number(status.landing_target_vy_m_s),
      ), 2)} m/s，并保留相对位置修正；当前上限 ${fmt(
        Number(status.position_velocity_limit_m_s), 2,
      )} m/s`;
  } else if (adaptiveProfileLoaded) {
    $("landingSpeedPlanDetail").textContent =
      "远距巡航，接近落点后按制动距离自动减速";
  } else {
    $("landingSpeedPlanDetail").textContent =
      "连接仿真后检查自适应速度控制器";
  }
  updateLandingReadiness({
    running,
    performanceRunning: Boolean(performance.running),
    selectedMovingTarget,
    targetStatusKnown,
    targetHealthy,
    landingDistance,
    movingWaterCouplingReady:
      rotorWater.moving_target_current_coupling_available === true
      || rotorWater.moving_target_current_coupled === true
      || typeof rotorWater.landing_surface_mode === "string",
    selectedSurfaceMode: $("landing_surface_mode").value,
    vehicleGeometryReady:
      rotorWater.vehicle_geometry_version === "float_geometry_v1",
    platformReady: status.landing_platform_available === true
      && targetStatus.platform_height_config_version === "configurable_v1"
      && targetStatus.surface_geometry_version === "solid_deck_geometry_v1",
    platformInitialOverlap,
  });
  const touchdownStates = new Set([
    "CONTACT_CONFIRM", "SETTLING", "SPOOL_DOWN", "LANDED",
  ]);
  const hasTouchdown = touchdownStates.has(stateName)
    || Number(landing.touchdown_horizontal_error_m) > 0;
  setMetric("landingTouchdownError", hasTouchdown
    ? Number(status.landing_touchdown_horizontal_error_m
      ?? landing.touchdown_horizontal_error_m)
    : NaN, 3);
  setMetric("landingTouchdownSpeed", hasTouchdown
    ? Number(status.landing_touchdown_relative_speed_m_s
      ?? landing.touchdown_relative_speed_m_s)
    : NaN, 3);
  setMetric("landingTouchdownYawError", hasTouchdown
    ? toDegrees(Number(status.landing_touchdown_yaw_error_rad))
    : NaN, 2);
  setMetric("landingContactDelay", hasTouchdown
    ? Number(status.landing_dual_contact_delay_s
      ?? landing.dual_contact_delay_s)
    : NaN, 3);
  const left = Number(status.left_float_submerged_fraction);
  const right = Number(status.right_float_submerged_fraction);
  const platformMode = $("landing_surface_mode").value === "platform";
  $("landingImmersion").textContent = platformMode
    ? (status.landing_platform_contact === true ? "已接触" : "未接触")
    : (Number.isFinite(left) && Number.isFinite(right)
      ? `${fmt(left * 100, 1)} / ${fmt(right * 100, 1)}`
      : "--");
  updateLandingStepper(stateName);
  const landingHint = $("landingActionHint");
  const abortDetail = formatLandingAbortReason(status, landing);
  const landingEditedCount = document.querySelectorAll(
    ".landing-console .is-edited"
  ).length;
  landingHint.textContent = running
    ? (stateName === "GO_AROUND"
      ? `正在安全复飞：${abortDetail}`
      : "任务参数已冻结，可切换为当前位置自动降落")
    : (landingEditedCount
      ? `${landingEditedCount} 项任务参数未保存，点击定位`
      : (stateName === "LANDED"
      ? `任务完成，${platformMode ? "着陆" : "触水"}精度指标已保存`
      : (stateName === "ABORTED"
        ? `任务已中止：${abortDetail}`
        : "参数将在任务启动时保存并冻结")));
  landingHint.disabled = running || !landingEditedCount;
  landingHint.classList.toggle("dirty", Boolean(landingEditedCount) && !running);
  drawLanding(landing);
}

const landingStepOrder = [
  "locate", "descent", "flare", "contact", "complete",
];
const landingStateStep = {
  CLIMB: "locate",
  STABILIZE: "locate",
  APPROACH: "locate",
  ALIGN: "locate",
  HIGH_HOVER: "locate",
  SLOW_DESCENT: "descent",
  NEAR_WATER: "flare",
  CONTACT_CONFIRM: "contact",
  SETTLING: "contact",
  SPOOL_DOWN: "complete",
  LANDED: "complete",
};

function updateLandingStepper(stateName) {
  const stepper = $("landingStepper");
  const activeStep = landingStateStep[stateName];
  const activeIndex = landingStepOrder.indexOf(activeStep);
  stepper.classList.toggle(
    "alert", ["GO_AROUND", "ABORTED"].includes(stateName)
  );
  stepper.querySelectorAll("[data-landing-step]").forEach((item) => {
    const index = landingStepOrder.indexOf(item.dataset.landingStep);
    item.classList.toggle("active", index === activeIndex);
    item.classList.toggle(
      "complete",
      stateName === "LANDED" || (activeIndex >= 0 && index < activeIndex),
    );
  });
}

async function startLanding() {
  const confirmed = window.confirm(
    "系统将自动完成定位、减速、分段下降和电机退转。确认落点及周围区域安全？",
  );
  if (!confirmed) return;
  const saved = await saveConfig();
  if (saved.error || !saved.config) {
    $("landingActionHint").textContent = "任务参数保存失败，未启动降落";
    $("landingActionHint").classList.add("dirty");
    return;
  }
  const result = await postJson("/landing/start");
  if (result.error) {
    $("controllerOutput").textContent = result.error;
    $("landingActionHint").textContent = `启动失败：${result.error}`;
  }
}

async function requestLocalLanding() {
  const confirmed = window.confirm(
    "将取消原落点跟踪，并以无人机当前位置作为新落点继续自动下降。是否继续？",
  );
  if (!confirmed) return;
  const result = await postJson("/landing/local");
  if (result.error) {
    $("controllerOutput").textContent = result.error;
    $("landingActionHint").textContent = `原地降落请求失败：${result.error}`;
  } else {
    $("landingActionHint").textContent = "已切换为当前位置自动降落";
  }
}

function setInput(id, value) {
  const el = $(id);
  if (el) el.value = value;
}

function syncMovingTargetState() {
  const enabled = Boolean($("landing_moving_target_enabled")?.checked);
  $("landingTargetPositionHeading").textContent = enabled ? "起始点" : "落点";
  $("landingTargetXLabel").textContent = enabled ? "起始点 X m" : "降落点 X m";
  $("landingTargetYLabel").textContent = enabled ? "起始点 Y m" : "降落点 Y m";
  document.querySelectorAll("[data-moving-target]").forEach((element) => {
    element.hidden = !enabled;
  });
  document.querySelectorAll("[data-target-mode]").forEach((button) => {
    button.classList.toggle(
      "active", button.dataset.targetMode === (enabled ? "moving" : "static")
    );
  });
}

function syncLandingSurfaceState() {
  const mode = $("landing_surface_mode")?.value === "platform"
    ? "platform" : "water";
  const platformMode = mode === "platform";
  document.querySelectorAll("[data-surface-mode]").forEach((button) => {
    button.classList.toggle("active", button.dataset.surfaceMode === mode);
  });
  document.querySelectorAll("[data-water-surface-only]").forEach((element) => {
    element.hidden = platformMode;
  });
  document.querySelectorAll("[data-platform-surface-only]").forEach((element) => {
    element.hidden = !platformMode;
  });
  landingStateLabels.CLIMB = platformMode ? "离台爬升" : "脱水爬升";
  landingStateLabels.NEAR_WATER = platformMode ? "近台缓冲" : "近水缓冲";
  landingStateLabels.CONTACT_CONFIRM = platformMode ? "甲板接触确认" : "触水确认";
  landingStateLabels.SETTLING = platformMode ? "甲板稳定" : "稳定浮态";
  $("landingFlareStepLabel").textContent = platformMode
    ? "近台缓冲" : "近水缓冲";
  $("landingContactStepLabel").textContent = platformMode
    ? "接触确认" : "触水确认";
  $("landingContactQualityTitle").textContent = platformMode
    ? "着陆质量" : "触水质量";
  $("landingContactDelayLabel").textContent = platformMode
    ? "接触确认延迟" : "双侧接触延迟";
  $("landingImmersionLabel").textContent = platformMode
    ? "甲板接触" : "左右浸没率";
  $("landingImmersionUnit").textContent = platformMode ? "" : "%";
}

function updateLandingReadiness({
  running,
  performanceRunning,
  selectedMovingTarget,
  targetStatusKnown,
  targetHealthy,
  landingDistance,
  movingWaterCouplingReady,
  selectedSurfaceMode,
  vehicleGeometryReady,
  platformReady,
  platformInitialOverlap,
}) {
  const readiness = $("landingReadiness");
  const state = $("landingReadinessState");
  const detail = $("landingReadinessDetail");
  const connected = Boolean(latestDashboardData?.ok);
  const roll = Math.abs(Number(latestDashboardData?.attitude?.roll_deg));
  const pitch = Math.abs(Number(latestDashboardData?.attitude?.pitch_deg));
  const attitudeSafe = Number.isFinite(roll) && Number.isFinite(pitch)
    && Math.max(roll, pitch) <= 20;
  const targetStatusReady = !selectedMovingTarget || targetStatusKnown;
  const targetValid = Number.isFinite(landingDistance)
    && targetStatusReady
    && (!selectedMovingTarget || targetHealthy);
  const platformMode = selectedSurfaceMode === "platform";
  const couplingReady = platformMode
    ? platformReady
    : (!selectedMovingTarget || movingWaterCouplingReady);
  const ready = connected && formLoaded && targetValid
    && vehicleGeometryReady && couplingReady && attitudeSafe
    && !performanceRunning
    && !(platformMode && platformInitialOverlap);

  readiness.classList.toggle("ready", ready && !running);
  readiness.classList.toggle("blocked", !ready && connected && !running);
  if (running) {
    state.textContent = "自动降落进行中";
    detail.textContent = "任务参数已冻结，可切换为当前位置自动降落";
  } else if (!connected) {
    state.textContent = "等待飞行数据";
    detail.textContent = "连接仿真后检查目标与飞行状态";
  } else if (performanceRunning) {
    state.textContent = "动态测试占用中";
    detail.textContent = "停止动态测试后才能开始自动降落";
  } else if (!vehicleGeometryReady) {
    state.textContent = "需重启仿真";
    detail.textContent = "当前 Gazebo 未加载统一机体几何描述，请重启仿真";
  } else if (platformMode && !platformReady) {
    state.textContent = "需重启仿真";
    detail.textContent = "当前 Gazebo 未加载可调高度甲板模型，请重启仿真";
  } else if (platformMode && platformInitialOverlap) {
    state.textContent = selectedMovingTarget ? "调整平台起始点" : "调整平台落点";
    detail.textContent = selectedMovingTarget
      ? "平台会与当前无人机重叠，请移动起始点或先起飞"
      : "平台会与当前无人机重叠，请移动落点或先起飞";
  } else if (!platformMode && selectedMovingTarget && !movingWaterCouplingReady) {
    state.textContent = "需重启仿真";
    detail.textContent = "当前水动力插件不支持移动目标触水，请重启 Gazebo";
  } else if (selectedMovingTarget && !targetStatusKnown) {
    state.textContent = "等待控制器状态";
    detail.textContent = "尚未收到漂移目标健康度，暂不判定目标失效";
  } else if (!targetValid) {
    state.textContent = selectedMovingTarget ? "漂移目标不可用" : "落点无效";
    detail.textContent = selectedMovingTarget
      ? "检查目标状态和速度安全限值"
      : "检查降落点 X、Y 数值";
  } else if (!attitudeSafe) {
    state.textContent = "当前姿态不适合降落";
    detail.textContent = `横滚 ${fmt(roll, 1)} deg，俯仰 ${fmt(pitch, 1)} deg`;
  } else {
    state.textContent = "可以开始";
    const surfaceLabel = platformMode ? "实体平台" : "水面落点";
    const heightDetail = platformMode
      ? ` · 台面高于水面 ${fmt(number("landing_platform_top_offset_m"), 2)} m`
      : "";
    detail.textContent = `${surfaceLabel}${heightDetail} · ${selectedMovingTarget ? "漂移目标正常" : "静态落点"} · 距离 ${fmt(landingDistance, 2)} m`;
  }
  $("startLanding").disabled = running || !ready;
  $("useCurrentLandingPoint").disabled = running || !connected;
}

function strategyMatches(values) {
  return Object.entries(values).every(([key, value]) => (
    Math.abs(number(`landing_${key}`) - value) <= 1e-6
  ));
}

function readLandingStrategyProfile() {
  return Object.fromEntries(landingStrategyProfileKeys.map(key => (
    [key, number(`landing_${key}`)]
  )));
}

function deserializeLandingStrategyProfile(profile, fallback) {
  const values = {...fallback};
  for (const key of landingStrategyProfileKeys) {
    if (key === "yaw_tolerance_deg") {
      const radians = Number(profile?.yaw_tolerance_rad);
      if (Number.isFinite(radians)) values[key] = toDegrees(radians);
    } else {
      const value = Number(profile?.[key]);
      if (Number.isFinite(value)) values[key] = value;
    }
  }
  return values;
}

function serializeLandingStrategyProfile(values) {
  const profile = {};
  for (const [key, value] of Object.entries(values)) {
    if (key === "yaw_tolerance_deg") {
      profile.yaw_tolerance_rad = Number(value) * Math.PI / 180;
    } else {
      profile[key] = Number(value);
    }
  }
  return profile;
}

function loadLandingStrategyProfiles(landing) {
  landingStrategyProfiles = Object.fromEntries(
    Object.entries(landingStrategyPresets).map(([name, preset]) => (
      [name, deserializeLandingStrategyProfile(
        landing?.strategy_profiles?.[name], preset.values
      )]
    ))
  );
  activeLandingStrategy = landingStrategyProfiles[landing?.selected_strategy]
    ? landing.selected_strategy
    : "standard";
}

function landingStrategyIsCustomized(name) {
  const profile = landingStrategyProfiles[name];
  const defaults = landingStrategyPresets[name]?.values;
  return Boolean(profile && defaults) && Object.entries(defaults).some(
    ([key, value]) => Math.abs(Number(profile[key]) - Number(value)) > 1e-6
  );
}

function savedLandingStrategyProfile(name, savedLanding) {
  const fallback = landingStrategyPresets[name]?.values || {};
  const savedProfile = savedLanding?.strategy_profiles?.[name];
  if (savedProfile) {
    return deserializeLandingStrategyProfile(savedProfile, fallback);
  }
  if (name === savedLanding?.selected_strategy) {
    return deserializeLandingStrategyProfile(savedLanding, fallback);
  }
  return {...fallback};
}

function landingStrategyProfilesMatch(left, right) {
  return landingStrategyProfileKeys.every(key => (
    Number.isFinite(Number(left?.[key]))
      && Number.isFinite(Number(right?.[key]))
      && Math.abs(Number(left[key]) - Number(right[key])) <= 1e-6
  ));
}

function showLandingStrategy(name, modified = false) {
  const preset = landingStrategyPresets[name];
  const customized = landingStrategyIsCustomized(name);
  document.querySelectorAll("[data-landing-strategy]").forEach(button => {
    button.classList.toggle(
      "active", button.dataset.landingStrategy === activeLandingStrategy
    );
  });
  $("landingStrategyState").textContent = preset
    ? `${preset.label}${customized ? " · 自定义" : ""}${modified ? " · 已修改" : ""}`
    : "自定义";
  $("landingStrategyDescription").textContent = modified
    ? "高级参数已修改，保存后将更新当前策略"
    : (customized
      ? "使用该策略最后保存的高级参数"
      : (preset?.description || "高级参数已单独调整"));
}

function syncLandingStrategyState() {
  const profile = landingStrategyProfiles[activeLandingStrategy];
  showLandingStrategy(
    activeLandingStrategy,
    Boolean(profile) && !strategyMatches(profile),
  );
}

function updateLandingStrategyEditedState() {
  const saved = lastSavedConfig?.landing;
  if (!saved) {
    setConfigDirty(true);
    return;
  }
  landingStrategyProfiles[activeLandingStrategy] =
    readLandingStrategyProfile();
  const savedActiveProfile = savedLandingStrategyProfile(
    activeLandingStrategy, saved
  );
  for (const key of landingStrategyProfileKeys) {
    const input = $(`landing_${key}`);
    const savedValue = Number(savedActiveProfile[key]);
    input.classList.toggle(
      "is-edited",
      !Number.isFinite(savedValue)
        || Math.abs(Number(input.value) - savedValue) > 1e-6,
    );
  }
  for (const key of landingKeys) {
    if (landingStrategyProfileKeys.includes(key)) continue;
    const input = $(`landing_${key}`);
    const savedValue = Number(saved[key]);
    input.classList.toggle(
      "is-edited",
      !Number.isFinite(savedValue)
        || Math.abs(Number(input.value) - savedValue) > 1e-6,
    );
  }
  for (const [configKey, inputKey] of Object.entries(landingAngleConfigFields)) {
    if (landingStrategyProfileKeys.includes(inputKey)) continue;
    const input = $(`landing_${inputKey}`);
    const savedValue = toDegrees(saved[configKey]);
    input.classList.toggle(
      "is-edited",
      !Number.isFinite(savedValue)
        || Math.abs(Number(input.value) - savedValue) > 1e-6,
    );
  }
  const movingTargetEnabled = $("landing_moving_target_enabled");
  movingTargetEnabled.classList.toggle(
    "is-edited",
    movingTargetEnabled.checked !== Boolean(saved.moving_target_enabled),
  );
  const surfaceMode = $("landing_surface_mode");
  surfaceMode.classList.toggle(
    "is-edited", surfaceMode.value !== String(saved.surface_mode || "water"),
  );
  document.querySelectorAll("[data-landing-strategy]").forEach(button => {
    const name = button.dataset.landingStrategy;
    const profileModified = !landingStrategyProfilesMatch(
      landingStrategyProfiles[name],
      savedLandingStrategyProfile(name, saved),
    );
    button.classList.toggle(
      "is-edited",
      profileModified || (
        name === activeLandingStrategy
        && activeLandingStrategy !== saved.selected_strategy
      ),
    );
  });
  const dirty = Boolean(document.querySelector(
    ".tuner .is-edited, .landing-console .is-edited"
  ));
  setConfigDirty(dirty);
}

function applyLandingStrategy(name) {
  const profile = landingStrategyProfiles[name];
  if (!profile) return;
  if (landingStrategyProfiles[activeLandingStrategy]) {
    landingStrategyProfiles[activeLandingStrategy] = readLandingStrategyProfile();
  }
  activeLandingStrategy = name;
  for (const [key, value] of Object.entries(profile)) {
    const input = $(`landing_${key}`);
    if (Math.abs(Number(input.value) - value) > 1e-6) {
      input.value = value;
      input.classList.add("is-edited");
    }
  }
  showLandingStrategy(name, false);
  updateLandingStrategyEditedState();
}

function useCurrentLandingPoint() {
  const data = latestDashboardData;
  if (!data?.ok) return;
  const values = {
    landing_target_x_m: rounded(Number(data.position.x), 3),
    landing_target_y_m: rounded(Number(data.position.y), 3),
    landing_target_yaw_deg: rounded(Number(data.attitude.yaw_deg), 1),
  };
  for (const [id, value] of Object.entries(values)) {
    $(id).value = value;
    $(id).classList.add("is-edited");
  }
  const moving = $("landing_moving_target_enabled");
  if (moving.checked) {
    moving.checked = false;
    moving.classList.add("is-edited");
    syncMovingTargetState();
  }
  setConfigDirty(true);
  updateLanding(
    data.landing || {},
    data.motors?.plugin_status || {},
    data.performance || {},
  );
}

function number(id) {
  return Number($(id).value);
}

function textValue(id) {
  return $(id).value;
}

function inputId(prefix, id) {
  return prefix ? `${prefix}_${id}` : id;
}

function formRoot(prefix = "") {
  return prefix ? $("testModal") : document.querySelector(".tuner");
}

function setPidTab(tab) {
  activePidTab = tab;
  document.querySelectorAll("[data-pid-tab]").forEach((button) => {
    const active = button.dataset.pidTab === tab;
    button.classList.toggle("active", active);
    button.setAttribute("aria-selected", String(active));
  });
  document.querySelectorAll("[data-pid-group]").forEach((group) => {
    group.hidden = group.dataset.pidGroup !== tab;
  });
}

function setConfigDirty(dirty) {
  configDirty = dirty;
  if (!dirty) {
    unsavedNavigationIndex = 0;
    document.querySelectorAll(
      ".tuner .is-edited, .landing-console .is-edited"
    ).forEach(element => element.classList.remove("is-edited"));
  }
  const state = $("configDirty");
  const editedCount = document.querySelectorAll(
    ".tuner .is-edited, .landing-console .is-edited"
  ).length;
  const visibleCount = dirty ? Math.max(1, editedCount) : 0;
  if (state) {
    state.textContent = dirty ? `${visibleCount} 项未保存` : "参数已保存";
    state.classList.toggle("dirty", dirty);
    state.disabled = !dirty;
  }
  $("discardConfig").disabled = !dirty;
  updateLandingFieldRevertButtons();
  $("statusUnsavedSummary").textContent = dirty
    ? `${visibleCount} 项修改未生效`
    : "已保存 · 与控制器一致";
  $("statusUnsavedSummary").className = dirty ? "warning" : "success";
  const landingHint = $("landingActionHint");
  if (landingHint && !latestDashboardData?.landing?.running) {
    const landingEditedCount = document.querySelectorAll(
      ".landing-console .is-edited"
    ).length;
    landingHint.textContent = landingEditedCount
      ? `${landingEditedCount} 项任务参数未保存，点击定位`
      : "参数将在任务启动时保存并冻结";
    landingHint.classList.toggle("dirty", Boolean(landingEditedCount));
    landingHint.disabled = !landingEditedCount;
  }
}

function focusUnsavedParameter(scope = "all") {
  const selector = scope === "landing"
    ? ".landing-console .is-edited"
    : ".tuner .is-edited, .landing-console .is-edited";
  const targets = [...document.querySelectorAll(selector)];
  if (!targets.length) return;
  const target = targets[unsavedNavigationIndex % targets.length];
  unsavedNavigationIndex = (unsavedNavigationIndex + 1) % targets.length;
  const inLanding = Boolean(target.closest(".landing-console"));
  setWorkspace(inLanding ? "landing" : "flight");
  if (!inLanding) {
    const panel = target.closest("[data-control-section]");
    if (panel) setControlSection(panel.dataset.controlSection);
    const pidGroup = target.closest("[data-pid-group]");
    if (pidGroup) setPidTab(pidGroup.dataset.pidGroup);
  }
  target.closest("details")?.setAttribute("open", "");
  const hiddenByMode = target.closest("[hidden]");
  let focusTarget = target;
  if (target.id === "landing_surface_mode") {
    focusTarget = $("landingSurfaceMode").querySelector("button.active");
  } else if (hiddenByMode && inLanding) {
    focusTarget = $("landingTargetMode").querySelector("button.active");
  } else if (hiddenByMode?.matches("[data-horizontal-modes]")) {
    focusTarget = $("horizontal_control_mode");
  }
  const highlight = focusTarget.closest("label, .landing-config-section") || focusTarget;
  requestAnimationFrame(() => {
    highlight.scrollIntoView({behavior: "smooth", block: "center"});
    focusTarget.focus({preventScroll: true});
    highlight.classList.remove("parameter-focus");
    requestAnimationFrame(() => highlight.classList.add("parameter-focus"));
    setTimeout(() => highlight.classList.remove("parameter-focus"), 1000);
  });
}

function syncHorizontalModeState(prefix = "") {
  const mode = textValue(inputId(prefix, "horizontal_control_mode"));
  const velocityEnabled = mode === "velocity";
  const positionEnabled = mode === "position";
  for (const key of ["target_vx_m_s", "target_vy_m_s"]) {
    $(inputId(prefix, key)).disabled = !velocityEnabled || positionEnabled;
  }
  for (const key of ["target_x_m", "target_y_m"]) {
    $(inputId(prefix, key)).disabled = !positionEnabled;
  }
  const root = formRoot(prefix);
  root?.querySelectorAll("[data-horizontal-modes]").forEach((element) => {
    const modes = String(element.dataset.horizontalModes || "").split(/\s+/);
    element.hidden = !modes.includes(mode);
  });
  if (!prefix) {
    setPidTab(mode);
    const labels = {attitude: "姿态", velocity: "速度", position: "位置"};
    $("statusModeSummary").textContent = labels[mode] || mode;
  }
}

function loadConfigInto(prefix, config) {
  for (const key of configScalars) {
    setInput(inputId(prefix, key), config[key]);
  }
  for (const [configKey, inputKey] of Object.entries(angleConfigFields)) {
    setInput(inputId(prefix, inputKey), rounded(toDegrees(config[configKey])));
  }
  for (const axis of pidAxes) {
    for (const key of pidKeys) {
      setInput(inputId(prefix, `${axis}_${key}`), config[axis][key]);
    }
  }
  for (const axis of velocityPidAxes) {
    for (const key of velocityPidKeys) {
      setInput(inputId(prefix, `${axis}_${key}`), config[axis][key]);
    }
  }
  for (const axis of positionPidAxes) {
    for (const key of positionPidKeys) {
      setInput(inputId(prefix, `${axis}_${key}`), config[axis][key]);
    }
  }
  setInput(
    inputId(prefix, "horizontal_control_mode"),
    config.position_control_enabled ? "position"
      : (config.velocity_control_enabled ? "velocity" : "attitude"),
  );
  syncHorizontalModeState(prefix);
  const disturbance = config.disturbance || {};
  const enabled = $(inputId(prefix, "disturbance_enabled"));
  if (enabled) enabled.checked = Boolean(disturbance.enabled);
  setInput(inputId(prefix, "disturbance_preset"), disturbance.preset || "off");
  setInput(inputId(prefix, "disturbance_seed"), disturbance.seed ?? 20260726);
  const nonidealities = config.nonidealities || {};
  const nonidealEnabled = $(inputId(prefix, "nonidealities_enabled"));
  if (nonidealEnabled) nonidealEnabled.checked = Boolean(nonidealities.enabled);
  setInput(inputId(prefix, "attitude_noise_std_deg"),
    rounded(toDegrees(nonidealities.attitude_noise_std_rad)));
  setInput(inputId(prefix, "gyro_noise_std_deg_s"),
    rounded(toDegrees(nonidealities.gyro_noise_std_rad_s)));
  setInput(inputId(prefix, "attitude_bias_std_deg"),
    rounded(toDegrees(nonidealities.attitude_bias_std_rad)));
  setInput(inputId(prefix, "gyro_bias_std_deg_s"),
    rounded(toDegrees(nonidealities.gyro_bias_std_rad_s)));
  setInput(inputId(prefix, "position_noise_std_m"), nonidealities.position_noise_std_m);
  setInput(inputId(prefix, "velocity_noise_std_m_s"), nonidealities.velocity_noise_std_m_s);
  setInput(inputId(prefix, "control_delay_ms"), Number(nonidealities.control_delay_s) * 1000);
  setInput(inputId(prefix, "motor_time_constant_ms"),
    Number(nonidealities.motor_time_constant_s) * 1000);
  setInput(inputId(prefix, "motor_rate_limit_rad_s2"), nonidealities.motor_rate_limit_rad_s2);
  setInput(inputId(prefix, "motor_effectiveness"), nonidealities.motor_effectiveness);
  setInput(inputId(prefix, "nonidealities_seed"), nonidealities.seed ?? 20260726);
  const aerodynamics = config.aerodynamics || {};
  const aerodynamicsEnabled = $(inputId(prefix, "aerodynamics_enabled"));
  if (aerodynamicsEnabled) aerodynamicsEnabled.checked = Boolean(aerodynamics.enabled);
  for (const key of aerodynamicKeys) {
    const inputKey = key === "seed" ? "aerodynamics_seed" : key;
    setInput(inputId(prefix, inputKey), aerodynamics[key]);
  }
  const rotorWater = config.rotor_water || {};
  const rotorInterferenceEnabled = $(
    inputId(prefix, "rotor_interference_enabled")
  );
  if (rotorInterferenceEnabled) {
    rotorInterferenceEnabled.checked = Boolean(
      rotorWater.rotor_interference_enabled
    );
  }
  const hydrodynamicsEnabled = $(inputId(prefix, "hydrodynamics_enabled"));
  if (hydrodynamicsEnabled) {
    hydrodynamicsEnabled.checked = Boolean(rotorWater.hydrodynamics_enabled);
  }
  setInput(
    inputId(prefix, "coaxial_max_thrust_loss_percent"),
    rounded(Number(rotorWater.coaxial_max_thrust_loss) * 100),
  );
  for (const key of rotorWaterKeys) {
    setInput(inputId(prefix, key), rotorWater[key]);
  }
  const landing = config.landing || {};
  const movingTargetEnabled = $(
    inputId(prefix, "landing_moving_target_enabled")
  );
  if (movingTargetEnabled) {
    movingTargetEnabled.checked = Boolean(landing.moving_target_enabled);
  }
  const surfaceMode = $(inputId(prefix, "landing_surface_mode"));
  if (surfaceMode) surfaceMode.value = landing.surface_mode === "platform"
    ? "platform" : "water";
  for (const key of landingKeys) {
    setInput(inputId(prefix, `landing_${key}`), landing[key]);
  }
  for (const [configKey, inputKey] of Object.entries(landingAngleConfigFields)) {
    setInput(
      inputId(prefix, `landing_${inputKey}`),
      rounded(toDegrees(landing[configKey])),
    );
  }
  if (!prefix) {
    loadLandingStrategyProfiles(landing);
    syncMovingTargetState();
    syncLandingSurfaceState();
  }
}

function loadForm(config) {
  lastSavedConfig = JSON.parse(JSON.stringify(config));
  loadConfigInto("", config);
  loadConfigInto("modal", config);
  syncLandingStrategyState();
  $("testModal").querySelectorAll(".is-edited").forEach(
    element => element.classList.remove("is-edited")
  );
  setConfigDirty(false);
}

function discardConfigChanges() {
  if (!lastSavedConfig) return;
  loadForm(JSON.parse(JSON.stringify(lastSavedConfig)));
}

function setControlSection(section) {
  activeControlSection = section;
  document.querySelectorAll("[data-control-section-tab]").forEach(button => {
    const active = button.dataset.controlSectionTab === section;
    button.classList.toggle("active", active);
    button.setAttribute("aria-selected", String(active));
  });
  document.querySelectorAll("[data-control-section]").forEach(panel => {
    panel.hidden = panel.dataset.controlSection !== section;
  });
}

function initializeFlightWorkbench() {
  const panels = document.querySelector(".panels[data-workspace='flight']");
  const tuner = document.querySelector(".tuner[data-workspace='flight']");
  if (!panels || !tuner) return;
  const workbench = document.createElement("section");
  workbench.className = "flight-workbench";
  workbench.dataset.workspace = "flight";
  panels.before(workbench);
  delete panels.dataset.workspace;
  delete tuner.dataset.workspace;
  workbench.append(tuner, panels);
}

function enhanceInputUnits() {
  const unitPattern = /^(.*\S)\s+([%A-Za-z°²³·()/.-]+)$/;
  document.querySelectorAll("label > input[type='number']").forEach(input => {
    if (input.parentElement?.classList.contains("input-with-unit")) return;
    const label = input.closest("label");
    const textNode = [...label.childNodes].find(node => (
      node.nodeType === Node.TEXT_NODE && node.textContent.trim()
    ));
    if (!textNode || label.querySelector(":scope > span")) return;
    const match = textNode.textContent.trim().match(unitPattern);
    if (!match) return;
    textNode.textContent = `${match[1]} `;
    const wrapper = document.createElement("span");
    wrapper.className = "input-with-unit";
    const unit = document.createElement("span");
    unit.className = "field-unit";
    unit.textContent = match[2];
    input.before(wrapper);
    wrapper.append(input, unit);
  });
}

function savedLandingFieldValue(input) {
  const saved = lastSavedConfig?.landing;
  if (!saved || !input.id.startsWith("landing_")) return undefined;
  const inputKey = input.id.slice("landing_".length);
  if (landingStrategyProfileKeys.includes(inputKey)) {
    return savedLandingStrategyProfile(activeLandingStrategy, saved)[inputKey];
  }
  const angleEntry = Object.entries(landingAngleConfigFields).find(
    ([, mappedInputKey]) => mappedInputKey === inputKey
  );
  if (angleEntry) return rounded(toDegrees(saved[angleEntry[0]]));
  return saved[inputKey];
}

function updateLandingFieldRevertButtons() {
  const running = Boolean(latestDashboardData?.landing?.running);
  document.querySelectorAll(".landing-field-revert").forEach(button => {
    const input = $(button.dataset.inputId);
    button.disabled = running || !input?.classList.contains("is-edited");
  });
}

function initializeLandingFieldReverts() {
  const settings = document.querySelector(".landing-advanced-settings");
  if (!settings) return;
  settings.querySelectorAll(".landing-fields input").forEach(input => {
    const label = input.closest("label");
    if (!label || label.parentElement?.classList.contains("landing-field-item")) {
      return;
    }
    const item = document.createElement("div");
    item.className = "landing-field-item";
    for (const attribute of ["data-water-surface-only", "data-platform-surface-only"]) {
      if (!label.hasAttribute(attribute)) continue;
      item.setAttribute(attribute, "");
      label.removeAttribute(attribute);
    }
    const button = document.createElement("button");
    button.type = "button";
    button.className = "landing-field-revert";
    button.dataset.inputId = input.id;
    button.setAttribute("aria-controls", input.id);
    button.setAttribute("aria-label", `撤回${label.textContent.trim()}的修改`);
    button.title = "恢复此项最近保存的值";
    button.textContent = "↩";
    label.before(item);
    item.append(label, button);
    button.addEventListener("click", () => {
      if (button.disabled) return;
      const savedValue = savedLandingFieldValue(input);
      if (savedValue === undefined || savedValue === null) return;
      input.value = savedValue;
      syncLandingStrategyState();
      updateLandingStrategyEditedState();
      input.focus({preventScroll: true});
    });
  });
  updateLandingFieldRevertButtons();
}

function enhanceChartHeaders() {
  document.querySelectorAll(".chart-legend").forEach(legend => {
    const canvas = legend.previousElementSibling;
    if (!(canvas instanceof HTMLCanvasElement)) return;
    const heading = canvas.previousElementSibling;
    if (!heading || !heading.matches("h2, h3")) return;
    const wrapper = document.createElement("div");
    wrapper.className = "chart-heading";
    heading.before(wrapper);
    wrapper.append(heading, legend);
  });
}

function exportCanvasPng(canvas, title) {
  const exportCanvas = document.createElement("canvas");
  exportCanvas.width = canvas.width;
  exportCanvas.height = canvas.height;
  const ctx = exportCanvas.getContext("2d");
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, exportCanvas.width, exportCanvas.height);
  ctx.drawImage(canvas, 0, 0);
  exportCanvas.toBlob((blob) => {
    if (!blob) return;
    const stamp = new Date().toISOString().replace(/[:.]/g, "-");
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = `coaxial-uav-${title}-${stamp}.png`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }, "image/png");
}

function enhanceChartExports() {
  const exportNames = {
    attitudeChart: "attitude",
    xyMap: "position",
    landingMap: "landing-trajectory",
    landingHeightChart: "landing-height",
    landingMotionChart: "landing-motion",
    testResponseChart: "dynamic-response",
    testPositionMap: "dynamic-position",
  };
  document.querySelectorAll("canvas").forEach((canvas) => {
    let header = canvas.previousElementSibling;
    if (header?.matches("h2, h3")) {
      const wrapper = document.createElement("div");
      wrapper.className = "chart-heading";
      header.before(wrapper);
      wrapper.append(header);
      header = wrapper;
    }
    if (!header?.matches(".chart-heading, .section-heading")) return;
    if (header.querySelector(".chart-export-button")) return;
    const button = document.createElement("button");
    button.type = "button";
    button.className = "chart-export-button";
    button.textContent = "保存 PNG";
    button.title = "保存当前图像为 PNG";
    button.setAttribute("aria-label", "保存当前图像为 PNG");
    button.addEventListener("click", () => {
      exportCanvasPng(canvas, exportNames[canvas.id] || canvas.id || "chart");
      button.textContent = "已保存";
      setTimeout(() => { button.textContent = "保存 PNG"; }, 1200);
    });
    header.append(button);
  });
}

function collectConfigFrom(prefix = "") {
  const horizontalMode = textValue(inputId(prefix, "horizontal_control_mode"));
  const config = {
    target_z_m: number(inputId(prefix, "target_z_m")),
    target_roll_rad: number(inputId(prefix, "target_roll_deg")) * Math.PI / 180,
    target_pitch_rad: number(inputId(prefix, "target_pitch_deg")) * Math.PI / 180,
    target_yaw_rad: number(inputId(prefix, "target_yaw_deg")) * Math.PI / 180,
    velocity_control_enabled: horizontalMode === "velocity",
    target_vx_m_s: number(inputId(prefix, "target_vx_m_s")),
    target_vy_m_s: number(inputId(prefix, "target_vy_m_s")),
    velocity_tilt_limit_rad: number(inputId(prefix, "velocity_tilt_limit_deg")) * Math.PI / 180,
    velocity_accel_limit_m_s2: number(inputId(prefix, "velocity_accel_limit_m_s2")),
    position_control_enabled: horizontalMode === "position",
    target_x_m: number(inputId(prefix, "target_x_m")),
    target_y_m: number(inputId(prefix, "target_y_m")),
    position_velocity_limit_m_s: number(inputId(prefix, "position_velocity_limit_m_s")),
    hover_omega_rad_s: number(inputId(prefix, "hover_omega_rad_s")),
    max_omega_rad_s: number(inputId(prefix, "max_omega_rad_s")),
    attitude_setpoint_rate_limit_rad_s:
      number(inputId(prefix, "attitude_setpoint_rate_limit_deg_s")) * Math.PI / 180,
    yaw_large_signal_kp: number(inputId(prefix, "yaw_large_signal_kp")),
    yaw_large_signal_kd: number(inputId(prefix, "yaw_large_signal_kd")),
    yaw_schedule_start_rad: number(inputId(prefix, "yaw_schedule_start_deg")) * Math.PI / 180,
    yaw_schedule_end_rad: number(inputId(prefix, "yaw_schedule_end_deg")) * Math.PI / 180,
    rate_hz: number(inputId(prefix, "rate_hz")),
    disturbance: {
      enabled: Boolean($(inputId(prefix, "disturbance_enabled")).checked),
      preset: textValue(inputId(prefix, "disturbance_preset")),
      seed: number(inputId(prefix, "disturbance_seed")),
    },
    nonidealities: {
      enabled: Boolean($(inputId(prefix, "nonidealities_enabled")).checked),
      attitude_noise_std_rad:
        number(inputId(prefix, "attitude_noise_std_deg")) * Math.PI / 180,
      gyro_noise_std_rad_s:
        number(inputId(prefix, "gyro_noise_std_deg_s")) * Math.PI / 180,
      attitude_bias_std_rad:
        number(inputId(prefix, "attitude_bias_std_deg")) * Math.PI / 180,
      gyro_bias_std_rad_s:
        number(inputId(prefix, "gyro_bias_std_deg_s")) * Math.PI / 180,
      position_noise_std_m: number(inputId(prefix, "position_noise_std_m")),
      velocity_noise_std_m_s: number(inputId(prefix, "velocity_noise_std_m_s")),
      control_delay_s: number(inputId(prefix, "control_delay_ms")) / 1000,
      motor_time_constant_s: number(inputId(prefix, "motor_time_constant_ms")) / 1000,
      motor_rate_limit_rad_s2: number(inputId(prefix, "motor_rate_limit_rad_s2")),
      motor_effectiveness: number(inputId(prefix, "motor_effectiveness")),
      seed: number(inputId(prefix, "nonidealities_seed")),
    },
    aerodynamics: {
      enabled: Boolean($(inputId(prefix, "aerodynamics_enabled")).checked),
    },
    rotor_water: {
      rotor_interference_enabled: Boolean(
        $(inputId(prefix, "rotor_interference_enabled")).checked
      ),
      coaxial_max_thrust_loss:
        number(inputId(prefix, "coaxial_max_thrust_loss_percent")) / 100,
      hydrodynamics_enabled: Boolean(
        $(inputId(prefix, "hydrodynamics_enabled")).checked
      ),
    },
    landing: {},
  };
  for (const key of aerodynamicKeys) {
    const inputKey = key === "seed" ? "aerodynamics_seed" : key;
    config.aerodynamics[key] = number(inputId(prefix, inputKey));
  }
  for (const key of rotorWaterKeys) {
    config.rotor_water[key] = number(inputId(prefix, key));
  }
  for (const key of landingKeys) {
    const id = inputId(prefix, `landing_${key}`);
    const input = $(id);
    if (input) config.landing[key] = number(id);
  }
  const movingTargetEnabled = $(
    inputId(prefix, "landing_moving_target_enabled")
  );
  if (movingTargetEnabled) {
    config.landing.moving_target_enabled =
      Boolean(movingTargetEnabled.checked);
  }
  const surfaceMode = $(inputId(prefix, "landing_surface_mode"));
  if (surfaceMode) config.landing.surface_mode =
    surfaceMode.value === "platform" ? "platform" : "water";
  for (const [configKey, inputKey] of Object.entries(landingAngleConfigFields)) {
    const id = inputId(prefix, `landing_${inputKey}`);
    const input = $(id);
    if (input) config.landing[configKey] = number(id) * Math.PI / 180;
  }
  if (!prefix) {
    landingStrategyProfiles[activeLandingStrategy] = readLandingStrategyProfile();
    config.landing.selected_strategy = activeLandingStrategy;
    config.landing.strategy_profiles = Object.fromEntries(
      Object.entries(landingStrategyProfiles).map(([name, values]) => (
        [name, serializeLandingStrategyProfile(values)]
      ))
    );
  }
  for (const axis of pidAxes) {
    config[axis] = {};
    for (const key of pidKeys) {
      config[axis][key] = number(inputId(prefix, `${axis}_${key}`));
    }
  }
  for (const axis of velocityPidAxes) {
    config[axis] = {};
    for (const key of velocityPidKeys) {
      config[axis][key] = number(inputId(prefix, `${axis}_${key}`));
    }
  }
  for (const axis of positionPidAxes) {
    config[axis] = {kd: 0};
    for (const key of positionPidKeys) {
      config[axis][key] = number(inputId(prefix, `${axis}_${key}`));
    }
  }
  return config;
}

function collectConfig() {
  return collectConfigFrom("");
}

async function postJson(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload),
  });
  return await response.json();
}

async function saveConfig() {
  try {
    const result = await postJson("/control/config", collectConfig());
    if (result.config) {
      loadForm(result.config);
      setConfigDirty(false);
    } else {
      $("statusUnsavedSummary").textContent = `保存失败：${result.error || "控制器未接受参数"}`;
      $("statusUnsavedSummary").className = "error";
    }
    return result;
  } catch (error) {
    $("statusUnsavedSummary").textContent = `保存失败：${error.message}`;
    $("statusUnsavedSummary").className = "error";
    return {error: error.message};
  }
}

async function saveLandingConfig() {
  const result = await saveConfig();
  if (result.config) {
    const label = landingStrategyPresets[activeLandingStrategy]?.label || "当前";
    $("landingActionHint").textContent = `${label}策略参数已保存`;
  }
}

async function restoreDefaults() {
  const confirmed = window.confirm(
    "恢复默认参数会停止闭环并清零输出，同时重置目标、PID、限幅和工况设置。是否继续？",
  );
  if (!confirmed) return;
  const button = $("restoreDefaults");
  button.disabled = true;
  try {
    const result = await postJson("/control/defaults");
    if (!result.ok) {
      $("controllerOutput").textContent = result.error || "恢复默认参数失败";
      $("statusUnsavedSummary").textContent = `恢复失败：${result.error || "未知错误"}`;
      $("statusUnsavedSummary").className = "error";
      return;
    }
    loadForm(result.config);
    formLoaded = true;
    $("controllerOutput").textContent = JSON.stringify({
      ...(result.last || {}),
      backup: result.backup,
    }, null, 2);
  } catch (error) {
    $("controllerOutput").textContent = `恢复默认参数失败: ${error.message}`;
    $("statusUnsavedSummary").textContent = `恢复失败：${error.message}`;
    $("statusUnsavedSummary").className = "error";
  } finally {
    button.disabled = false;
  }
}

async function startControl() {
  const saved = await saveConfig();
  if (saved.error || !saved.config) return;
  const result = await postJson("/control/start");
  if (result.error) {
    $("statusControlSummary").textContent = `启动失败：${result.error}`;
    $("statusControlSummary").className = "error";
  }
}

async function stopControl() {
  const result = await postJson("/control/stop");
  if (result.error) {
    $("statusControlSummary").textContent = `停止失败：${result.error}`;
    $("statusControlSummary").className = "error";
  }
}

function syncModalConfigToMain() {
  const config = collectConfigFrom("modal");
  inheritMainSystemCalibration(config, collectConfig());
  loadConfigInto("", config);
  setConfigDirty(true);
}

function inheritMainSystemCalibration(config, source = lastSavedConfig) {
  if (!source) return config;
  for (const key of ["hover_omega_rad_s", "max_omega_rad_s", "rate_hz"]) {
    config[key] = source[key];
  }
  config.attitude_setpoint_rate_limit_rad_s =
    source.attitude_setpoint_rate_limit_rad_s;
  return config;
}

function collectTestConfig() {
  return {
    axis: textValue("test_axis"),
    step: number("test_step"),
    baseline_s: number("test_baseline_s"),
    duration_s: number("test_duration_s"),
    repeat_count: number("test_repeat_count"),
    sample_period_s: number("test_sample_period_s"),
    settling_band_percent: number("test_settling_band_percent"),
  };
}

async function startTest() {
  if (!formLoaded) return;
  const test = collectTestConfig();
  if (!Number.isFinite(test.step) || Math.abs(test.step) < 1e-9) {
    $("testOutput").textContent = "阶跃幅值必须是非零数值";
    $("testUserMessage").textContent = "阶跃幅值必须是非零数值";
    $("testUserMessage").className = "action-message error";
    return;
  }
  const config = inheritMainSystemCalibration(collectConfigFrom("modal"));
  const result = await postJson("/test/start", {
    config,
    test,
  });
  if (!result.error) {
    $("testModal").querySelectorAll(".is-edited").forEach(
      element => element.classList.remove("is-edited")
    );
    updateTestWorkflowView({running: true, message: "waiting for stable baseline"});
  } else {
    $("testUserMessage").textContent = `启动失败：${result.error}`;
    $("testUserMessage").className = "action-message error";
  }
}

async function stopTest() {
  const result = await postJson("/test/stop");
  if (result.error) {
    $("testUserMessage").textContent = `停止失败：${result.error}`;
    $("testUserMessage").className = "action-message error";
  }
}

function testAxisUnits(axis) {
  if (axis === "z") {
    return {value: "m", output: "rad/s"};
  }
  if (axis === "vx" || axis === "vy") {
    return {value: "m/s", output: "m/s²"};
  }
  if (axis === "x" || axis === "y") {
    return {value: "m", output: "m/s"};
  }
  return {value: "deg", output: "Nm"};
}

function updateTestAxisUnit() {
  const axis = textValue("test_axis");
  const velocityAxis = ["vx", "vy"].includes(axis);
  const positionAxis = ["x", "y"].includes(axis);
  const maxStep = positionAxis ? 10 : (velocityAxis ? 5 : (axis === "yaw" ? 180 : 45));
  $("modal_horizontal_control_mode").value =
    positionAxis ? "position" : (velocityAxis ? "velocity" : "attitude");
  syncHorizontalModeState("modal");
  const relatedPid = {
    roll: ["roll"],
    pitch: ["pitch"],
    yaw: ["yaw"],
    vx: ["velocity_x", "pitch"],
    vy: ["velocity_y", "roll"],
    x: ["position_x", "velocity_x", "pitch"],
    y: ["position_y", "velocity_y", "roll"],
  }[axis] || [];
  document.querySelectorAll("[data-test-pid]").forEach((group) => {
    group.hidden = !relatedPid.includes(group.dataset.testPid);
  });
  $("modalYawScheduleGroup").hidden = axis !== "yaw";
  $("testStepUnit").textContent = positionAxis ? "m" : (velocityAxis ? "m/s" : "deg");
  $("test_step").min = -maxStep;
  $("test_step").max = maxStep;
  $("testSteadyUnit").textContent = positionAxis ? "m" : (velocityAxis ? "m/s" : "deg");
  if (lastTestAxis !== axis) {
    $("test_step").value = positionAxis ? 2 : (velocityAxis ? 1 : 15);
  }
  $("test_settling_band_percent").value = (velocityAxis || positionAxis) ? 5 : 2;
  lastTestAxis = axis;
  updateTestWorkflowView(latestDashboardData?.performance || {});
}

function niceLimit(values, fallback = 1) {
  const finite = values.filter(Number.isFinite).map(Math.abs);
  const maxValue = Math.max(fallback, ...finite);
  const exponent = Math.floor(Math.log10(maxValue));
  const base = Math.pow(10, exponent);
  return Math.ceil(maxValue / base) * base;
}

function chartColor(name) {
  const cssName = `--chart-${name.replace(/[A-Z]/g, letter => `-${letter.toLowerCase()}`)}`;
  return getComputedStyle(document.documentElement).getPropertyValue(cssName).trim()
    || testChartFallbackColors[name];
}

function uiColor(name, fallback) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

function drawLine(ctx, points, xOf, yOf, color, lineDash = [], lineWidth = 2, opacity = 1) {
  if (!points.length) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.globalAlpha = opacity;
  ctx.setLineDash(lineDash);
  ctx.beginPath();
  let drawing = false;
  points.forEach((point) => {
    const x = xOf(point);
    const y = yOf(point);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      drawing = false;
      return;
    }
    if (!drawing) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
    drawing = true;
  });
  ctx.stroke();
  ctx.restore();
}

function drawStepLine(ctx, points, xOf, yOf, color, lineDash = [7, 5], lineWidth = 1.5) {
  if (!points.length) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.setLineDash(lineDash);
  ctx.beginPath();
  let previousY = null;
  let drawing = false;
  for (const point of points) {
    const x = xOf(point);
    const y = yOf(point);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      drawing = false;
      previousY = null;
      continue;
    }
    if (!drawing) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, previousY);
      ctx.lineTo(x, y);
    }
    drawing = true;
    previousY = y;
  }
  ctx.stroke();
  ctx.restore();
}

function drawChartTooltip(ctx, pointerX, plot, lines) {
  if (!Number.isFinite(pointerX) || pointerX < plot.left || pointerX > plot.right) return;
  ctx.save();
  ctx.strokeStyle = uiColor("--axis", "#3e464d");
  ctx.globalAlpha = 0.58;
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.moveTo(pointerX, plot.top);
  ctx.lineTo(pointerX, plot.bottom);
  ctx.stroke();
  ctx.restore();

  ctx.save();
  ctx.font = "12px system-ui";
  const paddingX = 10;
  const lineHeight = 18;
  const boxWidth = Math.max(
    132,
    ...lines.map(line => ctx.measureText(line).width + 2 * paddingX),
  );
  const boxHeight = lines.length * lineHeight + 10;
  let boxX = pointerX + 10;
  if (boxX + boxWidth > plot.right) boxX = pointerX - boxWidth - 10;
  boxX = Math.max(plot.left + 4, Math.min(boxX, plot.right - boxWidth - 4));
  const boxY = plot.top + 8;
  ctx.globalAlpha = 0.96;
  ctx.fillStyle = uiColor("--panel", "#ffffff");
  ctx.strokeStyle = uiColor("--line-strong", "#c1c8ce");
  ctx.fillRect(boxX, boxY, boxWidth, boxHeight);
  ctx.strokeRect(boxX + 0.5, boxY + 0.5, boxWidth - 1, boxHeight - 1);
  ctx.globalAlpha = 1;
  ctx.fillStyle = uiColor("--text", "#20252a");
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  lines.forEach((line, index) => {
    ctx.fillText(line, boxX + paddingX, boxY + 8 + lineHeight * (index + 0.5));
  });
  ctx.restore();
}

function normalizeTestSamples(rawSamples) {
  const samples = rawSamples
    .filter(sample => Number.isFinite(Number(sample.sim_time_s)))
    .slice()
    .sort((a, b) => Number(a.sim_time_s) - Number(b.sim_time_s));
  if (!samples.length) return [];

  const startSimTime = Number(samples[0].sim_time_s);
  return samples.map(sample => ({
    ...sample,
    plot_t_s: Math.max(0, Number(sample.sim_time_s) - startSimTime),
  }));
}

function meanResponseSeries(seriesList, tMax, pointCount = 240) {
  if (seriesList.length < 2) return [];
  const interpolate = (series, time) => {
    if (!series.length || time < series[0].plot_t_s || time > series[series.length - 1].plot_t_s) {
      return NaN;
    }
    let low = 0;
    let high = series.length - 1;
    while (high - low > 1) {
      const middle = Math.floor((low + high) / 2);
      if (series[middle].plot_t_s <= time) low = middle;
      else high = middle;
    }
    const a = series[low];
    const b = series[high];
    const span = b.plot_t_s - a.plot_t_s;
    if (span <= 1e-9) return Number(a.value);
    const ratio = (time - a.plot_t_s) / span;
    return Number(a.value) + (Number(b.value) - Number(a.value)) * ratio;
  };
  const mean = [];
  for (let index = 0; index < pointCount; index++) {
    const plotTime = (tMax * index) / (pointCount - 1);
    const values = seriesList.map(series => interpolate(series, plotTime)).filter(Number.isFinite);
    if (values.length) {
      mean.push({
        plot_t_s: plotTime,
        value: values.reduce((sum, value) => sum + value, 0) / values.length,
      });
    }
  }
  return mean;
}

function prepareCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const fallbackWidth = Number(canvas.dataset.logicalWidth) || canvas.width;
  const fallbackHeight = Number(canvas.dataset.logicalHeight) || canvas.height;
  const width = Math.max(1, Math.round(rect.width || fallbackWidth));
  const height = Math.max(1, Math.round(rect.height || fallbackHeight));
  const dpr = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
  const pixelWidth = Math.round(width * dpr);
  const pixelHeight = Math.round(height * dpr);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  canvas.dataset.logicalWidth = String(width);
  canvas.dataset.logicalHeight = String(height);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return {ctx, w: width, h: height};
}

function drawTestResponse(test) {
  const canvas = $("testResponseChart");
  if (!canvas) return;
  const {ctx, w, h} = prepareCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  const samples = normalizeTestSamples(Array.isArray(test.samples) ? test.samples : []);
  const plot = {left: 58, right: w - 58, top: 24, bottom: h - 42};
  const plotW = plot.right - plot.left;
  const plotH = plot.bottom - plot.top;

  ctx.font = "12px system-ui";
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.strokeStyle = uiColor("--line", "#d8dce0");
  ctx.lineWidth = 1;
  ctx.textBaseline = "middle";

  for (let i = 0; i <= 4; i++) {
    const y = plot.top + (i / 4) * plotH;
    ctx.beginPath();
    ctx.moveTo(plot.left, y);
    ctx.lineTo(plot.right, y);
    ctx.stroke();
  }
  for (let i = 0; i <= 4; i++) {
    const x = plot.left + (i / 4) * plotW;
    ctx.beginPath();
    ctx.moveTo(x, plot.top);
    ctx.lineTo(x, plot.bottom);
    ctx.stroke();
  }

  ctx.strokeStyle = uiColor("--axis", "#353b41");
  ctx.beginPath();
  ctx.moveTo(plot.left, plot.bottom);
  ctx.lineTo(plot.right, plot.bottom);
  ctx.moveTo(plot.left, plot.top);
  ctx.lineTo(plot.left, plot.bottom);
  ctx.moveTo(plot.right, plot.top);
  ctx.lineTo(plot.right, plot.bottom);
  ctx.stroke();

  if (!samples.length) {
    const axisSelect = $("test_axis");
    const axisLabel = axisSelect.options[axisSelect.selectedIndex]?.text || "当前通道";
    ctx.textAlign = "center";
    ctx.fillStyle = uiColor("--text", "#20252a");
    ctx.font = "15px system-ui";
    ctx.fillText("尚无响应数据", w / 2, h / 2 - 10);
    ctx.fillStyle = uiColor("--muted", "#66717b");
    ctx.font = "12px system-ui";
    ctx.fillText(`${axisLabel} · ${$("testState").textContent}`, w / 2, h / 2 + 16);
    return;
  }

  const axis = samples[samples.length - 1].axis || test.axis || textValue("test_axis");
  const attitudeAxis = ["roll", "pitch", "yaw"].includes(axis);
  const repetitionSeries = (Array.isArray(test.repetitions) ? test.repetitions : [])
    .map(run => normalizeTestSamples(Array.isArray(run.samples) ? run.samples : []))
    .filter(series => series.length);
  if (attitudeAxis) {
    for (const series of [samples, ...repetitionSeries]) {
      for (const sample of series) {
      sample.value = toDegrees(sample.value);
      sample.target = toDegrees(sample.target);
      sample.filtered_target = toDegrees(sample.filtered_target);
      }
    }
  }
  const units = testAxisUnits(axis);
  const configuredDuration = Number(test.chart_duration_s)
    || (Number(test.baseline_s) + Number(test.duration_s));
  const sampledDuration = Math.max(
    0,
    ...samples.map(s => s.plot_t_s),
    ...repetitionSeries.flatMap(series => series.map(s => s.plot_t_s)),
  );
  const tMax = Math.max(
    1,
    sampledDuration,
    test.running && Number.isFinite(configuredDuration) ? configuredDuration : 0,
  );
  const valueLimit = niceLimit(
    [
      ...samples.flatMap(s => [Number(s.value), Number(s.target), Number(s.filtered_target)]),
      ...repetitionSeries.flatMap(series => series.map(s => Number(s.value))),
    ],
    axis === "z" ? 0.1 : 0.01,
  );
  const outputLimit = niceLimit(samples.map(s => Number(s.output)), axis === "z" ? 10 : 0.1);
  const xOf = (sample) => plot.left + (sample.plot_t_s / tMax) * plotW;
  const yValue = (value) => plot.top + ((valueLimit - value) / (2 * valueLimit)) * plotH;
  const yOutput = (value) => plot.top + ((outputLimit - value) / (2 * outputLimit)) * plotH;

  ctx.textAlign = "right";
  for (let i = 0; i <= 4; i++) {
    const value = valueLimit - (2 * valueLimit * i) / 4;
    const y = plot.top + (i / 4) * plotH;
    ctx.fillText(fmt(value, axis === "z" ? 2 : 3), plot.left - 8, y);
  }
  ctx.textAlign = "left";
  for (let i = 0; i <= 4; i++) {
    const value = outputLimit - (2 * outputLimit * i) / 4;
    const y = plot.top + (i / 4) * plotH;
    ctx.fillText(fmt(value, axis === "z" ? 0 : 2), plot.right + 8, y);
  }
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let i = 0; i <= 4; i++) {
    const x = plot.left + (i / 4) * plotW;
    ctx.fillText(`${fmt((tMax * i) / 4, 1)}s`, x, plot.bottom + 10);
  }
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillText(`响应 / ${units.value}`, plot.left, 16);
  ctx.textAlign = "right";
  ctx.fillText(`输出 / ${units.output}`, plot.right, 16);

  const stepSample = samples.find((sample, index) => (
    sample.phase === "step" && (index === 0 || samples[index - 1].phase !== "step")
  ));
  if (stepSample) {
    const stepX = xOf(stepSample);
    ctx.save();
    ctx.strokeStyle = chartColor("stepMarker");
    ctx.fillStyle = chartColor("stepMarker");
    ctx.setLineDash([3, 5]);
    ctx.beginPath();
    ctx.moveTo(stepX, plot.top);
    ctx.lineTo(stepX, plot.bottom);
    ctx.stroke();
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText("STEP", Math.min(stepX + 5, plot.right - 32), plot.top + 5);
    ctx.restore();
  }

  if (!hiddenChartSeries.has("filteredTarget")) {
    drawLine(
      ctx,
      samples.filter(s => Number.isFinite(Number(s.filtered_target))),
      xOf,
      s => yValue(Number(s.filtered_target)),
      chartColor("filteredTarget"),
      [2, 4],
      2,
    );
  }
  if (repetitionSeries.length > 1) {
    if (!hiddenChartSeries.has("responseRuns")) {
      repetitionSeries.forEach((series) => {
        drawLine(
          ctx,
          series,
          xOf,
          s => yValue(Number(s.value)),
          chartColor("response"),
          [],
          1.4,
          0.28,
        );
      });
    }
    if (!hiddenChartSeries.has("response")) {
      const meanSeries = meanResponseSeries(repetitionSeries, tMax);
      drawLine(ctx, meanSeries, xOf, s => yValue(Number(s.value)), chartColor("response"), [], 3);
    }
  } else if (!hiddenChartSeries.has("response")) {
    drawLine(ctx, samples, xOf, s => yValue(Number(s.value)), chartColor("response"), [], 2.5);
  }
  if (!hiddenChartSeries.has("output")) {
    drawLine(ctx, samples, xOf, s => yOutput(Number(s.output)), chartColor("output"), [], 2);
  }
  if (!hiddenChartSeries.has("target")) {
    drawLine(ctx, samples, xOf, s => yValue(Number(s.target)), chartColor("target"), [8, 5], 1.5);
  }
  if (Number.isFinite(chartPointerX.test)) {
    const hoverTime = Math.max(
      0,
      Math.min(tMax, ((chartPointerX.test - plot.left) / plotW) * tMax),
    );
    const nearest = samples.reduce((best, sample) => (
      Math.abs(sample.plot_t_s - hoverTime) < Math.abs(best.plot_t_s - hoverTime)
        ? sample
        : best
    ));
    drawChartTooltip(ctx, chartPointerX.test, plot, [
      `t  ${fmt(nearest.plot_t_s, 3)} s`,
      `响应  ${fmt(Number(nearest.value), 3)} ${units.value}`,
      `目标  ${fmt(Number(nearest.target), 3)} ${units.value}`,
      `输出  ${fmt(Number(nearest.output), 3)} ${units.output}`,
    ]);
  }
}

function drawAxes(ctx, w, h) {
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = uiColor("--line", "#d8dce0");
  ctx.lineWidth = 1;
  for (let i = 1; i < 5; i++) {
    const y = (h * i) / 5;
    ctx.beginPath();
    ctx.moveTo(36, y);
    ctx.lineTo(w - 12, y);
    ctx.stroke();
  }
}

function drawAttitude() {
  const canvas = $("attitudeChart");
  const {ctx, w, h} = prepareCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  const series = [
    ["roll_deg", "target_roll_rad", "attitudeRoll", "attitudeRollTarget"],
    ["pitch_deg", "target_pitch_rad", "attitudePitch", "attitudePitchTarget"],
    ["yaw_deg", "target_yaw_rad", "attitudeYaw", "attitudeYawTarget"],
  ];
  const plot = {left: 48, right: w - 14, top: 20, bottom: h - 34};
  const plotW = plot.right - plot.left;
  const plotH = plot.bottom - plot.top;
  const samples = history.filter(sample => Number.isFinite(Number(sample?.stats?.sim_time_s)));
  const t1 = samples.length ? Number(samples[samples.length - 1].stats.sim_time_s) : 0;
  const t0 = t1 - attitudeWindowS;
  const targetDegrees = (sample, key) => {
    const radians = Number(sample?.controller?.config?.[key]);
    return Number.isFinite(radians) ? radians * 180 / Math.PI : 0;
  };
  const plottedMagnitudes = samples.flatMap(sample => series.flatMap(([valueKey, targetKey]) => [
    Math.abs(Number(sample.attitude[valueKey])),
    Math.abs(targetDegrees(sample, targetKey)),
  ])).filter(Number.isFinite);
  const maxAbs = Math.max(5, ...plottedMagnitudes);
  const yMax = Math.ceil(maxAbs / 5) * 5;
  const xOf = sample => (
    plot.left + ((Number(sample.stats.sim_time_s) - t0) / attitudeWindowS) * plotW
  );
  const yOf = value => plot.top + ((yMax - value) / (2 * yMax)) * plotH;

  ctx.font = "12px system-ui";
  ctx.lineWidth = 1;
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.strokeStyle = uiColor("--line", "#d8dce0");
  ctx.textBaseline = "middle";

  for (let i = 0; i <= 4; i++) {
    const value = yMax - (2 * yMax * i) / 4;
    const y = plot.top + (i / 4) * plotH;
    ctx.beginPath();
    ctx.moveTo(plot.left, y);
    ctx.lineTo(plot.right, y);
    ctx.stroke();
    ctx.textAlign = "right";
    ctx.fillText(`${fmt(value, 0)}°`, plot.left - 8, y);
  }

  ctx.textBaseline = "top";
  for (let i = 0; i <= 4; i++) {
    const x = plot.left + (i / 4) * plotW;
    const relativeTime = -attitudeWindowS + (attitudeWindowS * i) / 4;
    ctx.beginPath();
    ctx.moveTo(x, plot.top);
    ctx.lineTo(x, plot.bottom);
    ctx.stroke();
    ctx.textAlign = "center";
    ctx.fillText(relativeTime === 0 ? "0s" : `${fmt(relativeTime, 0)}s`, x, plot.bottom + 8);
  }

  ctx.strokeStyle = uiColor("--axis", "#353b41");
  ctx.beginPath();
  ctx.moveTo(plot.left, plot.bottom);
  ctx.lineTo(plot.right, plot.bottom);
  ctx.moveTo(plot.left, plot.top);
  ctx.lineTo(plot.left, plot.bottom);
  ctx.stroke();

  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillText("姿态角 / deg", plot.left, 14);

  for (const [valueKey, , colorName] of series) {
    if (hiddenChartSeries.has(colorName)) continue;
    drawLine(
      ctx,
      samples,
      xOf,
      sample => yOf(Number(sample.attitude[valueKey])),
      chartColor(colorName),
      [],
      2.25,
    );
  }

  for (const [, targetKey, , targetColorName] of series) {
    if (hiddenChartSeries.has(targetColorName)) continue;
    if (!samples.some(sample => Math.abs(targetDegrees(sample, targetKey)) >= 1e-6)) continue;
    drawStepLine(
      ctx,
      samples,
      xOf,
      sample => yOf(targetDegrees(sample, targetKey)),
      chartColor(targetColorName),
    );
  }
  if (samples.length && Number.isFinite(chartPointerX.attitude)) {
    const hoverTime = t0 + Math.max(
      0,
      Math.min(
        attitudeWindowS,
        ((chartPointerX.attitude - plot.left) / plotW) * attitudeWindowS,
      ),
    );
    const nearest = samples.reduce((best, sample) => (
      Math.abs(Number(sample.stats.sim_time_s) - hoverTime)
        < Math.abs(Number(best.stats.sim_time_s) - hoverTime)
        ? sample
        : best
    ));
    drawChartTooltip(ctx, chartPointerX.attitude, plot, [
      `相对时间  ${fmt(Number(nearest.stats.sim_time_s) - t1, 2)} s`,
      `横滚  ${fmt(Number(nearest.attitude.roll_deg), 2)} deg`,
      `俯仰  ${fmt(Number(nearest.attitude.pitch_deg), 2)} deg`,
      `偏航  ${fmt(Number(nearest.attitude.yaw_deg), 2)} deg`,
    ]);
  }
}

function drawTestPositionMap(test) {
  const canvas = $("testPositionMap");
  if (!canvas) return;
  const {ctx, w, h} = prepareCanvas(canvas);
  const samples = Array.isArray(test.samples) ? test.samples : [];
  const actual = samples.map(sample => ({
    x: Number(sample.world_x_m),
    y: Number(sample.world_y_m),
  })).filter(point => Number.isFinite(point.x) && Number.isFinite(point.y));
  const plot = {left: 38, right: w - 18, top: 24, bottom: h - 34};
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = uiColor("--line", "#d8dce0");
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.font = "11px system-ui";
  if (!actual.length) {
    for (let index = 0; index <= 4; index++) {
      const x = plot.left + (index / 4) * (plot.right - plot.left);
      const y = plot.top + (index / 4) * (plot.bottom - plot.top);
      ctx.beginPath();
      ctx.moveTo(x, plot.top);
      ctx.lineTo(x, plot.bottom);
      ctx.moveTo(plot.left, y);
      ctx.lineTo(plot.right, y);
      ctx.stroke();
    }
    ctx.textAlign = "center";
    ctx.fillText("暂无平面轨迹", w / 2, h / 2);
    return;
  }

  const axis = String(test.axis || samples[samples.length - 1]?.axis || "");
  const start = actual[0];
  const target = ["x", "y"].includes(axis)
    ? samples.map(sample => ({
      x: axis === "x" ? Number(sample.target) : start.x,
      y: axis === "y" ? Number(sample.target) : start.y,
    })).filter(point => Number.isFinite(point.x) && Number.isFinite(point.y))
    : [];
  const points = [...actual, ...target];
  const minX = Math.min(...points.map(point => point.x));
  const maxX = Math.max(...points.map(point => point.x));
  const minY = Math.min(...points.map(point => point.y));
  const maxY = Math.max(...points.map(point => point.y));
  const span = Math.max(0.5, maxX - minX, maxY - minY);
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const scale = Math.min(
    (plot.right - plot.left) / (span * 1.18),
    (plot.bottom - plot.top) / (span * 1.18),
  );
  const xOf = value => (plot.left + plot.right) / 2 + (value - centerX) * scale;
  const yOf = value => (plot.top + plot.bottom) / 2 - (value - centerY) * scale;
  for (let index = 0; index <= 4; index++) {
    const offset = (index - 2) * span / 4;
    const x = xOf(centerX + offset);
    const y = yOf(centerY + offset);
    ctx.beginPath();
    ctx.moveTo(x, plot.top);
    ctx.lineTo(x, plot.bottom);
    ctx.moveTo(plot.left, y);
    ctx.lineTo(plot.right, y);
    ctx.stroke();
  }
  if (target.length) {
    drawLine(
      ctx, target,
      point => xOf(point.x), point => yOf(point.y),
      chartColor("target"), [5, 4], 1.8,
    );
  }
  drawLine(
    ctx, actual,
    point => xOf(point.x), point => yOf(point.y),
    chartColor("response"), [], 2.3,
  );
  ctx.fillStyle = uiColor("--success", "#26734d");
  ctx.beginPath();
  ctx.arc(xOf(start.x), yOf(start.y), 4, 0, Math.PI * 2);
  ctx.fill();
  const end = actual[actual.length - 1];
  ctx.fillStyle = uiColor("--map-point", "#a36217");
  ctx.fillRect(xOf(end.x) - 4, yOf(end.y) - 4, 8, 8);
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.textAlign = "left";
  ctx.fillText("x / m", plot.left, 14);
  ctx.textAlign = "right";
  ctx.fillText("y / m", plot.right, 14);
  ctx.textAlign = "left";
  ctx.fillText(`起点 ${fmt(start.x, 2)}, ${fmt(start.y, 2)}`, plot.left, h - 10);
  ctx.textAlign = "right";
  ctx.fillText(`终点 ${fmt(end.x, 2)}, ${fmt(end.y, 2)}`, plot.right, h - 10);
}

function drawLandingGrid(ctx, plot, tMax, leftLabels, rightLabels = []) {
  const plotW = plot.right - plot.left;
  const plotH = plot.bottom - plot.top;
  ctx.font = "12px system-ui";
  ctx.strokeStyle = uiColor("--line", "#d8dce0");
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.lineWidth = 1;
  ctx.textBaseline = "middle";
  for (let i = 0; i <= 4; i++) {
    const y = plot.top + (i / 4) * plotH;
    ctx.beginPath();
    ctx.moveTo(plot.left, y);
    ctx.lineTo(plot.right, y);
    ctx.stroke();
    ctx.textAlign = "right";
    ctx.fillText(leftLabels[i], plot.left - 8, y);
    if (rightLabels.length) {
      ctx.textAlign = "left";
      ctx.fillText(rightLabels[i], plot.right + 8, y);
    }
  }
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let i = 0; i <= 4; i++) {
    const x = plot.left + (i / 4) * plotW;
    ctx.beginPath();
    ctx.moveTo(x, plot.top);
    ctx.lineTo(x, plot.bottom);
    ctx.stroke();
    ctx.fillText(`${fmt(tMax * i / 4, 1)}s`, x, plot.bottom + 9);
  }
}

function drawLandingStateMarkers(ctx, samples, xOf, plot) {
  const labels = {
    APPROACH: "进场",
    ALIGN: "对准",
    SLOW_DESCENT: "下降",
    NEAR_WATER: "缓冲",
    CONTACT_CONFIRM: "触水",
    SPOOL_DOWN: "收桨",
  };
  const markers = [];
  const seenStates = new Set();
  let previous = null;
  for (const sample of samples) {
    const state = String(sample.state || "");
    if (state === previous || !labels[state] || seenStates.has(state)) {
      previous = state;
      continue;
    }
    markers.push({state, label: labels[state], x: xOf(sample)});
    seenStates.add(state);
    previous = state;
  }

  const lanes = Array.from({length: 4}, () => []);
  const placements = [];
  ctx.font = "10px system-ui";
  for (const marker of markers) {
    const width = Math.ceil(ctx.measureText(marker.label).width);
    let best = null;
    for (let lane = 0; lane < lanes.length; lane++) {
      const preferredPositions = [marker.x + 5, marker.x - width - 5];
      lanes[lane].forEach(interval => preferredPositions.push(
        interval.left - width - 6,
        interval.right + 6,
      ));
      for (const preferredLeft of preferredPositions) {
        const left = Math.max(
          plot.left + 2,
          Math.min(preferredLeft, plot.right - width - 2),
        );
        const overlap = lanes[lane].reduce((total, interval) => (
          total + Math.max(0, Math.min(left + width, interval.right)
            - Math.max(left, interval.left) + 5)
        ), 0);
        const distance = Math.abs(left + width / 2 - marker.x);
        const score = overlap * 10000 + distance + lane * 0.01;
        if (!best || score < best.score) best = {lane, left, width, score};
      }
    }
    lanes[best.lane].push({left: best.left, right: best.left + best.width});
    placements.push({...marker, ...best, top: 4 + best.lane * 13});
  }

  for (const marker of placements) {
    ctx.save();
    ctx.strokeStyle = uiColor("--muted", "#687078");
    ctx.globalAlpha = 0.72;
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    ctx.moveTo(marker.x, plot.top);
    ctx.lineTo(marker.x, plot.bottom);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 0.94;
    ctx.fillStyle = uiColor("--panel", "#ffffff");
    ctx.fillRect(marker.left - 2, marker.top - 1, marker.width + 4, 12);
    ctx.globalAlpha = 1;
    ctx.fillStyle = uiColor("--muted", "#687078");
    ctx.font = "10px system-ui";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText(marker.label, marker.left, marker.top);
    ctx.restore();
  }
  return placements.map(marker => ({
    label: marker.label,
    left: marker.left,
    right: marker.left + marker.width,
    top: marker.top,
    bottom: marker.top + 11,
    lane: marker.lane,
  }));
}

function drawLandingHeightChart(samples, tMax) {
  const canvas = $("landingHeightChart");
  if (!canvas) return;
  const {ctx, w, h} = prepareCanvas(canvas);
  const plot = {left: 54, right: w - 18, top: 60, bottom: h - 38};
  const plotW = plot.right - plot.left;
  const plotH = plot.bottom - plot.top;
  ctx.clearRect(0, 0, w, h);
  if (!samples.length) {
    ctx.fillStyle = uiColor("--muted", "#687078");
    ctx.font = "12px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("等待降落任务数据...", w / 2, h / 2);
    return;
  }
  const zValues = samples.flatMap(s => [
    Number(s.z_ref_m), Number(s.z_m),
    Math.max(0, Number(s.float_clearance_m)),
  ]).filter(Number.isFinite);
  const zMin = Math.min(0, ...zValues);
  const zMaxRaw = Math.max(0.5, ...zValues);
  const zSpan = Math.max(0.5, zMaxRaw - zMin);
  const zMax = zMaxRaw + zSpan * 0.08;
  const zFloor = zMin - zSpan * 0.04;
  const xOf = s => plot.left + (Number(s.t_s) / tMax) * plotW;
  const yHeight = value =>
    plot.bottom - ((Number(value) - zFloor) / (zMax - zFloor)) * plotH;
  const labels = Array.from({length: 5}, (_, i) =>
    fmt(zMax - ((zMax - zFloor) * i) / 4, 2)
  );
  drawLandingGrid(ctx, plot, tMax, labels);
  canvas.dataset.eventMarkerLayout = JSON.stringify(
    drawLandingStateMarkers(ctx, samples, xOf, plot),
  );
  drawLine(ctx, samples, xOf, s => yHeight(Number(s.z_ref_m)),
    chartColor("attitudePitchTarget"), [6, 4], 2);
  drawLine(ctx, samples, xOf, s => yHeight(Number(s.z_m)),
    chartColor("attitudePitch"), [], 2.5);
  drawLine(ctx, samples, xOf,
    s => yHeight(Math.max(0, Number(s.float_clearance_m))),
    chartColor("attitudeRollTarget"), [], 1.8);
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.fillText("高度 / m", plot.left, plot.top - 5);
}

function drawLandingMotionChart(samples, tMax) {
  const canvas = $("landingMotionChart");
  if (!canvas) return;
  const {ctx, w, h} = prepareCanvas(canvas);
  const plot = {left: 54, right: w - 54, top: 60, bottom: h - 38};
  const plotW = plot.right - plot.left;
  const plotH = plot.bottom - plot.top;
  ctx.clearRect(0, 0, w, h);
  if (!samples.length) {
    ctx.fillStyle = uiColor("--muted", "#687078");
    ctx.font = "12px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("等待降落任务数据...", w / 2, h / 2);
    return;
  }
  const velocityLimit = niceLimit(
    samples.map(s => Number(s.vz_m_s)),
    0.2,
  );
  const maxOmega = Math.max(
    1,
    Number(latestDashboardData?.controller?.config?.max_omega_rad_s) || 0,
    ...samples.map(s => Number(s.motor_omega_rad_s) || 0),
  );
  const xOf = s => plot.left + (Number(s.t_s) / tMax) * plotW;
  const yVelocity = value =>
    plot.top + ((velocityLimit - Number(value)) / (2 * velocityLimit)) * plotH;
  const yMotor = value =>
    plot.bottom - Math.max(0, Math.min(1, Number(value) / maxOmega)) * plotH;
  const leftLabels = Array.from({length: 5}, (_, i) =>
    fmt(velocityLimit - (2 * velocityLimit * i) / 4, 2)
  );
  const rightLabels = Array.from({length: 5}, (_, i) => `${100 - i * 25}%`);
  drawLandingGrid(ctx, plot, tMax, leftLabels, rightLabels);
  canvas.dataset.eventMarkerLayout = JSON.stringify(
    drawLandingStateMarkers(ctx, samples, xOf, plot),
  );
  drawLine(ctx, samples, xOf, s => yVelocity(Number(s.vz_m_s)),
    chartColor("attitudeRoll"), [], 2.3);
  drawLine(ctx, samples, xOf, s => yMotor(Number(s.motor_omega_rad_s)),
    chartColor("attitudeYaw"), [], 2);
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.textBaseline = "alphabetic";
  ctx.textAlign = "left";
  ctx.fillText("垂向速度 / m/s", plot.left, plot.top - 5);
  ctx.textAlign = "right";
  ctx.fillText("执行器 / %", plot.right, plot.top - 5);
}

function drawLanding(landing) {
  const samples = Array.isArray(landing.samples) ? landing.samples : [];
  const tMax = Math.max(1, ...samples.map(s => Number(s.t_s) || 0));
  drawLandingHeightChart(samples, tMax);
  drawLandingMotionChart(samples, tMax);
}

function drawPositionMap(canvas, data, detailed = false) {
  if (!canvas) return;
  const {ctx, w, h} = prepareCanvas(canvas);
  const pluginStatus = data.motors?.plugin_status || {};
  const landingConfig = data.controller?.config?.landing || {};
  const landingSamples = Array.isArray(data.landing?.samples)
    ? data.landing.samples
    : [];
  const targetX = Number(
    pluginStatus.landing_target_x_m ?? landingConfig.target_x_m ?? 0
  );
  const targetY = Number(
    pluginStatus.landing_target_y_m ?? landingConfig.target_y_m ?? 0
  );
  const targetYaw = Number(
    pluginStatus.landing_target_yaw_rad ?? landingConfig.target_yaw_rad ?? 0
  );
  const actualTrail = detailed
    ? landingSamples.map(s => ({x: Number(s.x_m), y: Number(s.y_m)}))
    : history.map(sample => ({
      x: Number(sample.position?.x),
      y: Number(sample.position?.y),
    }));
  const targetTrail = detailed
    ? landingSamples.map(s => ({x: Number(s.x_ref_m), y: Number(s.y_ref_m)}))
    : history.map(sample => {
      const status = sample.motors?.plugin_status || {};
      return {
        x: Number(status.landing_target_x_m),
        y: Number(status.landing_target_y_m),
      };
    });
  const points = [
    ...actualTrail,
    ...targetTrail,
    {x: Number(data.position.x), y: Number(data.position.y)},
    {x: targetX, y: targetY},
  ].filter(point => Number.isFinite(point.x) && Number.isFinite(point.y));
  const minX = Math.min(...points.map(point => point.x), -0.3);
  const maxX = Math.max(...points.map(point => point.x), 0.3);
  const minY = Math.min(...points.map(point => point.y), -0.3);
  const maxY = Math.max(...points.map(point => point.y), 0.3);
  const span = Math.max(1.2, maxX - minX, maxY - minY);
  const padding = detailed ? 44 : 30;
  const scale = Math.min(
    (w - 2 * padding) / (span * 1.22),
    (h - 2 * padding) / (span * 1.22),
  );
  const worldCx = (minX + maxX) / 2;
  const worldCy = (minY + maxY) / 2;
  const cx = w / 2 - worldCx * scale;
  const cy = h / 2 + worldCy * scale;
  const xOf = value => cx + value * scale;
  const yOf = value => cy - value * scale;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = uiColor("--line", "#d8dce0");
  ctx.lineWidth = 1;
  for (let i = -4; i <= 4; i++) {
    const offset = i * span / 8;
    ctx.beginPath();
    ctx.moveTo(xOf(worldCx + offset), padding);
    ctx.lineTo(xOf(worldCx + offset), h - padding);
    ctx.moveTo(padding, yOf(worldCy + offset));
    ctx.lineTo(w - padding, yOf(worldCy + offset));
    ctx.stroke();
  }
  ctx.strokeStyle = uiColor("--axis", "#353b41");
  ctx.beginPath();
  ctx.moveTo(xOf(0), padding);
  ctx.lineTo(xOf(0), h - padding);
  ctx.moveTo(padding, yOf(0));
  ctx.lineTo(w - padding, yOf(0));
  ctx.stroke();
  const tolerance = Number(landingConfig.position_tolerance_m) || 0.15;
  ctx.strokeStyle = uiColor("--map-ring", "#26734d");
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.arc(xOf(targetX), yOf(targetY), tolerance * scale, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);
  drawLine(
    ctx,
    targetTrail,
    point => xOf(point.x),
    point => yOf(point.y),
    chartColor("filteredTarget"),
    [5, 4],
    1.8,
  );
  drawLine(
    ctx,
    actualTrail,
    point => xOf(point.x),
    point => yOf(point.y),
    chartColor("response"),
    [],
    detailed ? 2.6 : 1.8,
  );
  const x = xOf(Number(data.position.x));
  const y = yOf(Number(data.position.y));
  ctx.fillStyle = uiColor("--map-point", "#a36217");
  ctx.beginPath();
  ctx.arc(x, y, 6, 0, Math.PI * 2);
  ctx.fill();
  const touchdown = landingSamples.find(sample => sample.state === "CONTACT_CONFIRM");
  if (detailed && touchdown) {
    ctx.fillStyle = chartColor("attitudeYaw");
    ctx.fillRect(xOf(Number(touchdown.x_m)) - 4, yOf(Number(touchdown.y_m)) - 4, 8, 8);
  }
  const tx = xOf(targetX);
  const ty = yOf(targetY);
  ctx.strokeStyle = chartColor("target");
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(tx, ty, 8, 0, Math.PI * 2);
  ctx.moveTo(tx - 11, ty);
  ctx.lineTo(tx + 11, ty);
  ctx.moveTo(tx, ty - 11);
  ctx.lineTo(tx, ty + 11);
  ctx.moveTo(tx, ty);
  ctx.lineTo(
    tx + 18 * Math.cos(targetYaw),
    ty - 18 * Math.sin(targetYaw),
  );
  ctx.stroke();
  ctx.fillStyle = uiColor("--muted", "#687078");
  ctx.font = "12px system-ui";
  ctx.fillText(`x ${fmt(data.position.x, 2)} m`, 12, 22);
  ctx.fillText(`y ${fmt(data.position.y, 2)} m`, 12, 40);
  ctx.textAlign = "right";
  ctx.fillText(
    `落点 ${fmt(targetX, 1)}, ${fmt(targetY, 1)} m / ${fmt(toDegrees(targetYaw), 0)} deg`,
    w - 12,
    22,
  );
  ctx.textAlign = "left";
}

function drawMap(data) {
  drawPositionMap($("xyMap"), data);
  drawPositionMap($("landingMap"), data, true);
}

initializeFlightWorkbench();
enhanceInputUnits();
initializeLandingFieldReverts();
enhanceChartHeaders();
enhanceChartExports();
initializeLandingOverview();

const events = new EventSource("/events");
events.onmessage = (event) => updateState(JSON.parse(event.data));
events.onerror = () => {
  $("connection").textContent = "DISCONNECTED";
  $("connection").classList.remove("ok");
  $("status").textContent = "仪表盘服务连接中断";
  $("statusGazebo").textContent = "控制台服务断开";
  $("statusGazebo").className = "error";
};

$("saveConfig").addEventListener("click", saveConfig);
$("discardConfig").addEventListener("click", discardConfigChanges);
$("configDirty").addEventListener("click", () => focusUnsavedParameter("all"));
$("saveLandingConfig").addEventListener("click", saveLandingConfig);
$("landingActionHint").addEventListener("click", () => focusUnsavedParameter("landing"));
$("restoreDefaults").addEventListener("click", restoreDefaults);
$("startControl").addEventListener("click", startControl);
$("stopControl").addEventListener("click", stopControl);
$("startLanding").addEventListener("click", startLanding);
$("localLanding").addEventListener("click", requestLocalLanding);
$("useCurrentLandingPoint").addEventListener("click", useCurrentLandingPoint);
$("landingStrategy").addEventListener("click", (event) => {
  const button = event.target.closest("[data-landing-strategy]");
  if (button) applyLandingStrategy(button.dataset.landingStrategy);
});
$("landing_moving_target_enabled").addEventListener(
  "change", syncMovingTargetState
);
$("landingTargetMode").addEventListener("click", (event) => {
  const button = event.target.closest("[data-target-mode]");
  if (!button) return;
  $("landing_moving_target_enabled").checked =
    button.dataset.targetMode === "moving";
  $("landing_moving_target_enabled").classList.add("is-edited");
  syncMovingTargetState();
  setConfigDirty(true);
});
$("landingSurfaceMode").addEventListener("click", (event) => {
  const button = event.target.closest("[data-surface-mode]");
  if (!button) return;
  $("landing_surface_mode").value = button.dataset.surfaceMode;
  $("landing_surface_mode").classList.add("is-edited");
  syncLandingSurfaceState();
  setConfigDirty(true);
  updateLanding(
    latestDashboardData?.landing || {},
    latestDashboardData?.motors?.plugin_status || {},
    latestDashboardData?.performance || {},
    latestDashboardData?.rotor_water || {},
  );
});
$("workspaceTabs").addEventListener("click", (event) => {
  const button = event.target.closest("[data-workspace-tab]");
  if (button) setWorkspace(button.dataset.workspaceTab);
});
$("controlSectionTabs").addEventListener("click", (event) => {
  const button = event.target.closest("[data-control-section-tab]");
  if (button) setControlSection(button.dataset.controlSectionTab);
});
$("syncModalConfig").addEventListener("click", syncModalConfigToMain);
$("startTest").addEventListener("click", startTest);
$("test_axis").addEventListener("change", updateTestAxisUnit);
$("horizontal_control_mode").addEventListener("change", () => syncHorizontalModeState(""));
$("pidTabs").addEventListener("click", (event) => {
  const button = event.target.closest("[data-pid-tab]");
  if (button) setPidTab(button.dataset.pidTab);
});
document.querySelectorAll(".tuner, .landing-console").forEach((configSurface) => {
  configSurface.addEventListener("input", (event) => {
    if (event.target.matches("input:not(:disabled), select:not(:disabled)")) {
      event.target.classList.add("is-edited");
      if (event.target.closest(".landing-advanced-settings")) {
        syncLandingStrategyState();
        updateLandingStrategyEditedState();
      } else {
        setConfigDirty(true);
      }
    }
  });
  configSurface.addEventListener("change", (event) => {
    if (event.target.matches("input:not(:disabled), select:not(:disabled)")) {
      event.target.classList.add("is-edited");
      if (event.target.closest(".landing-advanced-settings")) {
        syncLandingStrategyState();
        updateLandingStrategyEditedState();
      } else {
        setConfigDirty(true);
      }
    }
  });
});
$("testModal").addEventListener("input", (event) => {
  if (event.target.matches("input:not(:disabled), select:not(:disabled)")) {
    event.target.classList.add("is-edited");
    updateTestWorkflowView(latestDashboardData?.performance || {});
  }
});
$("testModal").addEventListener("change", (event) => {
  if (event.target.matches("input:not(:disabled), select:not(:disabled)")) {
    event.target.classList.add("is-edited");
    updateTestWorkflowView(latestDashboardData?.performance || {});
  }
});
document.querySelectorAll("[data-chart-series]").forEach(legend => {
  legend.addEventListener("click", () => {
    const series = legend.dataset.chartSeries;
    if (hiddenChartSeries.has(series)) hiddenChartSeries.delete(series);
    else hiddenChartSeries.add(series);
    document.querySelectorAll(`[data-chart-series="${series}"]`).forEach(item => {
      item.classList.toggle("hidden-series", hiddenChartSeries.has(series));
    });
    drawAttitude();
    drawTestResponse(latestDashboardData?.performance || {});
  });
});
function bindChartPointer(canvasId, key, redraw) {
  const canvas = $(canvasId);
  canvas.addEventListener("pointermove", (event) => {
    const rect = canvas.getBoundingClientRect();
    chartPointerX[key] = event.clientX - rect.left;
    redraw();
  });
  canvas.addEventListener("pointerleave", () => {
    chartPointerX[key] = NaN;
    redraw();
  });
}
bindChartPointer("attitudeChart", "attitude", drawAttitude);
bindChartPointer("testResponseChart", "test", () => {
  drawTestResponse(latestDashboardData?.performance || {});
});
if ("ResizeObserver" in window) {
  let resizeFrame = 0;
  const canvasResizeObserver = new ResizeObserver(() => {
    cancelAnimationFrame(resizeFrame);
    resizeFrame = requestAnimationFrame(() => {
      if (activeWorkspace === "flight") drawAttitude();
      if (activeWorkspace === "landing" && latestDashboardData) {
        drawLanding(latestDashboardData.landing || {});
        drawMap(latestDashboardData);
      }
      if (activeWorkspace === "test") {
        drawTestResponse(latestDashboardData?.performance || {});
        drawTestPositionMap(latestDashboardData?.performance || {});
      }
    });
  });
  document.querySelectorAll("canvas").forEach(canvas => canvasResizeObserver.observe(canvas));
}
$("stopTest").addEventListener("click", stopTest);
syncHorizontalModeState("");
syncHorizontalModeState("modal");
updateTestAxisUnit();
setControlSection(activeControlSection);
setWorkspace("flight");
initializeControllerForm();
