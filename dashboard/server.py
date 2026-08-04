#!/usr/bin/env python3
"""Local dashboard server for Gazebo simulation state monitoring."""

from __future__ import annotations

import argparse
import csv
import errno
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque
from copy import deepcopy
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controllers.attitude_pid import PID, Pose, clamp, field, quat_to_euler, read_model_pose, run_cmd, wrap_pi  # noqa: E402

RUNTIME_DIR = PROJECT_ROOT / "data" / "runtime"
PERFORMANCE_DIR = PROJECT_ROOT / "data" / "performance"
TUNING_CONFIG_PATH = RUNTIME_DIR / "tuning_config.json"
TUNING_BACKUP_PATH = RUNTIME_DIR / "tuning_config.before_defaults.json"
TUNING_DEFAULTS_PATH = PROJECT_ROOT / "config" / "tuning_defaults.json"
CONFIG_PUBLISHER_PATH = PROJECT_ROOT / "build" / "tools" / "gz_config_publisher"

LANDING_STRATEGY_PROFILE_DEFAULTS = {
    "smooth": {
        "high_hover_z_m": 1.8,
        "approach_speed_m_s": 0.55,
        "cruise_speed_m_s": 1.5,
        "position_tolerance_m": 0.12,
        "yaw_tolerance_rad": math.radians(4.0),
        "descent_rate_m_s": 0.25,
        "flare_clearance_m": 0.50,
        "flare_rate_m_s": 0.08,
        "touchdown_max_vz_m_s": 0.15,
        "contact_confirm_s": 0.40,
        "spool_down_s": 1.80,
    },
    "standard": {
        "high_hover_z_m": 1.8,
        "approach_speed_m_s": 0.8,
        "cruise_speed_m_s": 2.5,
        "position_tolerance_m": 0.15,
        "yaw_tolerance_rad": math.radians(5.0),
        "descent_rate_m_s": 0.35,
        "flare_clearance_m": 0.40,
        "flare_rate_m_s": 0.12,
        "touchdown_max_vz_m_s": 0.20,
        "contact_confirm_s": 0.30,
        "spool_down_s": 1.50,
    },
    "fast": {
        "high_hover_z_m": 1.6,
        "approach_speed_m_s": 0.95,
        "cruise_speed_m_s": 2.5,
        "position_tolerance_m": 0.20,
        "yaw_tolerance_rad": math.radians(7.0),
        "descent_rate_m_s": 0.45,
        "flare_clearance_m": 0.35,
        "flare_rate_m_s": 0.16,
        "touchdown_max_vz_m_s": 0.25,
        "contact_confirm_s": 0.25,
        "spool_down_s": 1.20,
    },
}

ATTITUDE_INERTIA_KG_M2 = {
    "roll": 0.19588,
    "pitch": 0.35588,
    "yaw": 0.30,
}

DEFAULT_TUNING_CONFIG = {
    "target_z_m": 0.8,
    "target_roll_rad": 0.0,
    "target_pitch_rad": 0.0,
    "target_yaw_rad": 0.0,
    "velocity_control_enabled": False,
    "target_vx_m_s": 0.0,
    "target_vy_m_s": 0.0,
    "velocity_tilt_limit_rad": math.radians(15.0),
    "velocity_accel_limit_m_s2": 2.2,
    "hover_omega_rad_s": 136.362,
    "min_omega_rad_s": 0.0,
    "max_omega_rad_s": 150.0,
    "attitude_setpoint_rate_limit_rad_s": 0.75,
    "height": {"kp": 45.0, "ki": 0.0, "kd": 35.0, "limit": 30.0, "integral_limit": 0.5},
    "roll": {"kp": 193.3, "ki": 0.0, "kd": 8.61, "limit": 2.5, "integral_limit": 0.2},
    "pitch": {"kp": 351.3, "ki": 0.0, "kd": 15.65, "limit": 2.7, "integral_limit": 0.2},
    "yaw": {"kp": 296.1, "ki": 0.0, "kd": 13.19, "limit": 0.7, "integral_limit": 0.2},
    "velocity_x": {"kp": 2.8, "ki": 0.15, "kd": 0.0, "limit": 2.2, "integral_limit": 1.0},
    "velocity_y": {"kp": 2.8, "ki": 0.15, "kd": 0.0, "limit": 2.2, "integral_limit": 1.0},
    "position_control_enabled": False,
    "target_x_m": 0.0,
    "target_y_m": 0.0,
    "position_velocity_limit_m_s": 2.5,
    "position_x": {"kp": 2.5, "ki": 0.0, "kd": 0.95, "limit": 2.5, "integral_limit": 2.0},
    "position_y": {"kp": 2.5, "ki": 0.0, "kd": 0.95, "limit": 2.5, "integral_limit": 2.0},
    "yaw_large_signal_kp": 20.0,
    "yaw_large_signal_kd": 3.0,
    "yaw_schedule_start_rad": 0.02,
    "yaw_schedule_end_rad": 0.08,
    "disturbance": {
        "enabled": False,
        "preset": "off",
        "seed": 20260726,
    },
    "nonidealities": {
        "enabled": False,
        "attitude_noise_std_rad": math.radians(0.02),
        "gyro_noise_std_rad_s": math.radians(0.10),
        "attitude_bias_std_rad": math.radians(0.05),
        "gyro_bias_std_rad_s": math.radians(0.05),
        "position_noise_std_m": 0.003,
        "velocity_noise_std_m_s": 0.01,
        "control_delay_s": 0.015,
        "motor_time_constant_s": 0.08,
        "motor_rate_limit_rad_s2": 500.0,
        "motor_effectiveness": 0.98,
        "seed": 20260726,
    },
    "aerodynamics": {
        "enabled": False,
        "air_density_kg_m3": 1.225,
        "drag_area_x_m2": 0.12,
        "drag_area_y_m2": 0.18,
        "drag_area_z_m2": 0.10,
        "angular_damping_roll_nm_s": 0.12,
        "angular_damping_pitch_nm_s": 0.18,
        "angular_damping_yaw_nm_s": 0.10,
        "wind_x_m_s": 0.0,
        "wind_y_m_s": 0.0,
        "wind_z_m_s": 0.0,
        "gust_rms_m_s": 0.40,
        "gust_correlation_time_s": 0.80,
        "mass_scale": 1.03,
        "inertia_scale_roll": 1.05,
        "inertia_scale_pitch": 0.97,
        "inertia_scale_yaw": 1.04,
        "cg_offset_x_m": 0.010,
        "cg_offset_y_m": -0.008,
        "cg_offset_z_m": 0.005,
        "seed": 20260727,
    },
    "rotor_water": {
        "rotor_interference_enabled": True,
        "coaxial_max_thrust_loss": 0.06,
        "coaxial_inflow_time_constant_s": 0.12,
        "hydrodynamics_enabled": True,
        "water_density_kg_m3": 997.0,
        "water_level_z_m": 0.0,
        "float_virtual_draft_m": 0.055,
        "water_linear_drag_x_n_s_m": 4.0,
        "water_linear_drag_y_n_s_m": 35.0,
        "water_linear_drag_z_n_s_m": 80.0,
        "water_quadratic_drag_x": 0.25,
        "water_quadratic_drag_y": 1.0,
        "water_quadratic_drag_z": 1.1,
        "water_current_x_m_s": 0.0,
        "water_current_y_m_s": 0.0,
        "water_current_z_m_s": 0.0,
        "water_slamming_gain_n_s_m": 35.0,
    },
    "landing": {
        "surface_mode": "water",
        "platform_top_offset_m": 0.20,
        "target_x_m": 0.0,
        "target_y_m": 0.0,
        "target_yaw_rad": 0.0,
        "moving_target_enabled": False,
        "target_vx_m_s": 0.0,
        "target_vy_m_s": 0.0,
        "target_yaw_rate_rad_s": 0.0,
        "target_status_timeout_s": 0.30,
        "target_speed_limit_m_s": 0.80,
        "high_hover_z_m": 1.8,
        "approach_speed_m_s": 0.8,
        "cruise_speed_m_s": 2.5,
        "position_tolerance_m": 0.15,
        "yaw_tolerance_rad": math.radians(5.0),
        "descent_rate_m_s": 0.35,
        "flare_clearance_m": 0.40,
        "flare_rate_m_s": 0.12,
        "touchdown_max_vz_m_s": 0.20,
        "contact_confirm_s": 0.30,
        "spool_down_s": 1.50,
        "departure_horizontal_speed_limit_m_s": 0.30,
        "departure_clearance_margin_m": 0.10,
        "near_horizontal_speed_limit_m_s": 0.30,
        "moving_target_correction_reserve_m_s": 0.30,
        "approach_braking_accel_m_s2": 0.55,
        "abort_position_error_m": 0.40,
        "near_max_descent_speed_m_s": 0.30,
        "go_around_height_m": 1.0,
        "departure_stable_time_s": 0.50,
        "align_stable_time_s": 0.80,
        "hover_stable_time_s": 1.0,
        "approach_relative_speed_tolerance_m_s": 0.15,
        "align_relative_speed_tolerance_m_s": 0.10,
        "hover_relative_speed_tolerance_m_s": 0.12,
        "departure_horizontal_speed_tolerance_m_s": 0.10,
        "height_tolerance_m": 0.16,
        "approach_vertical_speed_tolerance_m_s": 0.10,
        "precision_vertical_speed_tolerance_m_s": 0.08,
        "near_overspeed_grace_s": 1.0,
        "contact_submerged_fraction": 0.02,
        "settling_vertical_speed_limit_m_s": 0.08,
        "settling_time_s": 0.50,
        "contact_loss_grace_s": 0.08,
        "go_around_height_tolerance_m": 0.18,
        "go_around_vertical_speed_tolerance_m_s": 0.12,
        "flare_transition_margin_m": 0.02,
        "departure_tilt_limit_rad": math.radians(5.0),
        "approach_tilt_limit_rad": math.radians(10.0),
        "near_tilt_limit_rad": math.radians(5.0),
        "warning_tilt_rad": math.radians(5.0),
        "abort_tilt_rad": math.radians(8.0),
        "approach_abort_tilt_rad": math.radians(12.0),
        "yaw_rate_tolerance_rad_s": math.radians(5.0),
        "contact_tilt_rate_limit_rad_s": math.radians(10.0),
        "settling_tilt_rate_limit_rad_s": math.radians(5.0),
        "go_around_tilt_tolerance_rad": math.radians(5.0),
    },
    "rate_hz": 100.0,
}


def merge_config(base: dict[str, object], update: dict[str, object]) -> None:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge_config(base[key], value)  # type: ignore[index,arg-type]
        else:
            base[key] = value


def load_versioned_tuning_defaults() -> None:
    try:
        saved = json.loads(TUNING_DEFAULTS_PATH.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            f"invalid versioned tuning defaults: {TUNING_DEFAULTS_PATH}"
        ) from exc
    if not isinstance(saved, dict):
        raise RuntimeError(
            f"versioned tuning defaults must be an object: {TUNING_DEFAULTS_PATH}"
        )
    merge_config(DEFAULT_TUNING_CONFIG, saved)


load_versioned_tuning_defaults()


def sanitize_nonidealities(config: dict[str, object]) -> None:
    data = config.get("nonidealities")
    if not isinstance(data, dict):
        data = deepcopy(DEFAULT_TUNING_CONFIG["nonidealities"])  # type: ignore[arg-type]
        config["nonidealities"] = data
    defaults = DEFAULT_TUNING_CONFIG["nonidealities"]  # type: ignore[assignment]
    data["enabled"] = bool(data.get("enabled", defaults["enabled"]))
    for key, maximum in (
        ("attitude_noise_std_rad", math.radians(10.0)),
        ("gyro_noise_std_rad_s", math.radians(100.0)),
        ("attitude_bias_std_rad", math.radians(10.0)),
        ("gyro_bias_std_rad_s", math.radians(20.0)),
        ("position_noise_std_m", 1.0),
        ("velocity_noise_std_m_s", 5.0),
        ("control_delay_s", 0.5),
        ("motor_time_constant_s", 1.0),
        ("motor_rate_limit_rad_s2", 10000.0),
    ):
        data[key] = clamp(float(data.get(key, defaults[key])), 0.0, maximum)
    data["motor_effectiveness"] = clamp(
        float(data.get("motor_effectiveness", defaults["motor_effectiveness"])),
        0.5,
        1.0,
    )
    data["seed"] = max(0, int(data.get("seed", defaults["seed"])))


def sanitize_aerodynamics(config: dict[str, object]) -> None:
    data = config.get("aerodynamics")
    if not isinstance(data, dict):
        data = deepcopy(DEFAULT_TUNING_CONFIG["aerodynamics"])  # type: ignore[arg-type]
        config["aerodynamics"] = data
    defaults = DEFAULT_TUNING_CONFIG["aerodynamics"]  # type: ignore[assignment]
    data["enabled"] = bool(data.get("enabled", defaults["enabled"]))
    ranges = {
        "air_density_kg_m3": (0.5, 2.0),
        "drag_area_x_m2": (0.0, 2.0),
        "drag_area_y_m2": (0.0, 2.0),
        "drag_area_z_m2": (0.0, 2.0),
        "angular_damping_roll_nm_s": (0.0, 10.0),
        "angular_damping_pitch_nm_s": (0.0, 10.0),
        "angular_damping_yaw_nm_s": (0.0, 10.0),
        "wind_x_m_s": (-30.0, 30.0),
        "wind_y_m_s": (-30.0, 30.0),
        "wind_z_m_s": (-15.0, 15.0),
        "gust_rms_m_s": (0.0, 10.0),
        "gust_correlation_time_s": (0.05, 20.0),
        "mass_scale": (0.7, 1.3),
        "inertia_scale_roll": (0.5, 1.5),
        "inertia_scale_pitch": (0.5, 1.5),
        "inertia_scale_yaw": (0.5, 1.5),
        "cg_offset_x_m": (-0.15, 0.15),
        "cg_offset_y_m": (-0.15, 0.15),
        "cg_offset_z_m": (-0.15, 0.15),
    }
    for key, (minimum, maximum) in ranges.items():
        data[key] = clamp(float(data.get(key, defaults[key])), minimum, maximum)
    data["seed"] = max(0, int(data.get("seed", defaults["seed"])))


def sanitize_rotor_water(config: dict[str, object]) -> None:
    data = config.get("rotor_water")
    if not isinstance(data, dict):
        data = deepcopy(DEFAULT_TUNING_CONFIG["rotor_water"])  # type: ignore[arg-type]
        config["rotor_water"] = data
    defaults = DEFAULT_TUNING_CONFIG["rotor_water"]  # type: ignore[assignment]
    data["rotor_interference_enabled"] = bool(
        data.get("rotor_interference_enabled", defaults["rotor_interference_enabled"])
    )
    data["hydrodynamics_enabled"] = bool(
        data.get("hydrodynamics_enabled", defaults["hydrodynamics_enabled"])
    )
    ranges = {
        "coaxial_max_thrust_loss": (0.0, 0.35),
        "coaxial_inflow_time_constant_s": (0.01, 2.0),
        "water_density_kg_m3": (500.0, 1300.0),
        "water_level_z_m": (-2.0, 2.0),
        "float_virtual_draft_m": (0.0, 0.18),
        "water_linear_drag_x_n_s_m": (0.0, 1000.0),
        "water_linear_drag_y_n_s_m": (0.0, 1000.0),
        "water_linear_drag_z_n_s_m": (0.0, 1000.0),
        "water_quadratic_drag_x": (0.0, 5.0),
        "water_quadratic_drag_y": (0.0, 5.0),
        "water_quadratic_drag_z": (0.0, 5.0),
        "water_current_x_m_s": (-5.0, 5.0),
        "water_current_y_m_s": (-5.0, 5.0),
        "water_current_z_m_s": (-2.0, 2.0),
        "water_slamming_gain_n_s_m": (0.0, 500.0),
    }
    for key, (minimum, maximum) in ranges.items():
        data[key] = clamp(float(data.get(key, defaults[key])), minimum, maximum)


def sanitize_landing(config: dict[str, object]) -> None:
    data = config.get("landing")
    if not isinstance(data, dict):
        data = deepcopy(DEFAULT_TUNING_CONFIG["landing"])  # type: ignore[arg-type]
        config["landing"] = data
    defaults = DEFAULT_TUNING_CONFIG["landing"]  # type: ignore[assignment]
    data["surface_mode"] = (
        "platform" if str(data.get("surface_mode", defaults["surface_mode"]))
        == "platform" else "water"
    )
    supplied_profiles = data.get("strategy_profiles")
    has_saved_profiles = isinstance(supplied_profiles, dict)
    data["moving_target_enabled"] = bool(
        data.get("moving_target_enabled", defaults["moving_target_enabled"])
    )
    ranges = {
        "platform_top_offset_m": (0.05, 2.0),
        "target_x_m": (-100.0, 100.0),
        "target_y_m": (-100.0, 100.0),
        "target_yaw_rad": (-math.pi, math.pi),
        "target_vx_m_s": (-2.0, 2.0),
        "target_vy_m_s": (-2.0, 2.0),
        "target_yaw_rate_rad_s": (-1.0, 1.0),
        "target_status_timeout_s": (0.05, 2.0),
        "target_speed_limit_m_s": (0.05, 2.0),
        "high_hover_z_m": (0.8, 5.0),
        "approach_speed_m_s": (0.1, 2.0),
        "cruise_speed_m_s": (0.5, 3.0),
        "position_tolerance_m": (0.05, 0.5),
        "yaw_tolerance_rad": (math.radians(1.0), math.radians(20.0)),
        "descent_rate_m_s": (0.05, 0.8),
        "flare_clearance_m": (0.15, 1.0),
        "flare_rate_m_s": (0.03, 0.3),
        "touchdown_max_vz_m_s": (0.05, 0.5),
        "contact_confirm_s": (0.1, 2.0),
        "spool_down_s": (0.3, 5.0),
        "departure_horizontal_speed_limit_m_s": (0.05, 1.5),
        "departure_clearance_margin_m": (0.02, 0.50),
        "near_horizontal_speed_limit_m_s": (0.05, 1.5),
        "moving_target_correction_reserve_m_s": (0.05, 1.5),
        "approach_braking_accel_m_s2": (0.1, 3.0),
        "abort_position_error_m": (0.1, 2.0),
        "near_max_descent_speed_m_s": (0.1, 1.0),
        "go_around_height_m": (0.3, 3.0),
        "departure_stable_time_s": (0.1, 3.0),
        "align_stable_time_s": (0.1, 3.0),
        "hover_stable_time_s": (0.1, 5.0),
        "approach_relative_speed_tolerance_m_s": (0.03, 0.8),
        "align_relative_speed_tolerance_m_s": (0.03, 0.5),
        "hover_relative_speed_tolerance_m_s": (0.03, 0.5),
        "departure_horizontal_speed_tolerance_m_s": (0.03, 0.5),
        "height_tolerance_m": (0.05, 0.5),
        "approach_vertical_speed_tolerance_m_s": (0.03, 0.5),
        "precision_vertical_speed_tolerance_m_s": (0.02, 0.3),
        "near_overspeed_grace_s": (0.0, 3.0),
        "contact_submerged_fraction": (0.005, 0.3),
        "settling_vertical_speed_limit_m_s": (0.02, 0.3),
        "settling_time_s": (0.1, 3.0),
        "contact_loss_grace_s": (0.0, 0.5),
        "go_around_height_tolerance_m": (0.05, 0.5),
        "go_around_vertical_speed_tolerance_m_s": (0.03, 0.5),
        "flare_transition_margin_m": (0.0, 0.15),
        "departure_tilt_limit_rad": (math.radians(1.0), math.radians(20.0)),
        "approach_tilt_limit_rad": (math.radians(3.0), math.radians(30.0)),
        "near_tilt_limit_rad": (math.radians(1.0), math.radians(20.0)),
        "warning_tilt_rad": (math.radians(1.0), math.radians(20.0)),
        "abort_tilt_rad": (math.radians(3.0), math.radians(30.0)),
        "approach_abort_tilt_rad": (math.radians(5.0), math.radians(35.0)),
        "yaw_rate_tolerance_rad_s": (math.radians(1.0), math.radians(30.0)),
        "contact_tilt_rate_limit_rad_s": (math.radians(1.0), math.radians(45.0)),
        "settling_tilt_rate_limit_rad_s": (math.radians(1.0), math.radians(30.0)),
        "go_around_tilt_tolerance_rad": (math.radians(1.0), math.radians(20.0)),
    }
    for key, (minimum, maximum) in ranges.items():
        data[key] = clamp(float(data.get(key, defaults[key])), minimum, maximum)
    data["cruise_speed_m_s"] = max(
        float(data["approach_speed_m_s"]),
        float(data["cruise_speed_m_s"]),
    )
    data["abort_tilt_rad"] = max(
        float(data["warning_tilt_rad"]), float(data["abort_tilt_rad"])
    )
    data["approach_abort_tilt_rad"] = max(
        float(data["abort_tilt_rad"]), float(data["approach_abort_tilt_rad"])
    )

    profile_ranges = {
        key: ranges[key] for key in LANDING_STRATEGY_PROFILE_DEFAULTS["standard"]
    }
    profiles: dict[str, dict[str, float]] = {}
    source_profiles = supplied_profiles if isinstance(supplied_profiles, dict) else {}
    for name, profile_defaults in LANDING_STRATEGY_PROFILE_DEFAULTS.items():
        supplied = source_profiles.get(name)
        source = supplied if isinstance(supplied, dict) else {}
        profile: dict[str, float] = {}
        for key, default_value in profile_defaults.items():
            minimum, maximum = profile_ranges[key]
            profile[key] = clamp(float(source.get(key, default_value)), minimum, maximum)
        profile["cruise_speed_m_s"] = max(
            profile["approach_speed_m_s"], profile["cruise_speed_m_s"]
        )
        profiles[name] = profile

    selected = str(data.get("selected_strategy", ""))
    if selected not in profiles:
        profile_keys = tuple(LANDING_STRATEGY_PROFILE_DEFAULTS["standard"])
        selected = min(
            profiles,
            key=lambda name: sum(
                abs(float(data[key]) - profiles[name][key])
                / max(profile_ranges[key][1] - profile_ranges[key][0], 1e-9)
                for key in profile_keys
            ),
        )
    if not has_saved_profiles:
        profiles[selected] = {
            key: float(data[key])
            for key in LANDING_STRATEGY_PROFILE_DEFAULTS["standard"]
        }
    data["selected_strategy"] = selected
    data["strategy_profiles"] = profiles


def active_partition(default: str = "coaxial_uav_static_water") -> str:
    state_file = RUNTIME_DIR / "active_gazebo_partition.env"
    try:
        for line in state_file.read_text(encoding="utf-8").splitlines():
            if line.startswith("GZ_PARTITION="):
                value = line.split("=", 1)[1].strip()
                return value or default
    except OSError:
        pass
    return default


def parse_stats(debug_text: str) -> dict[str, float]:
    sec = float(_field(debug_text, "sec", 0.0))
    nsec_match = re.search(r"sim_time\s*\{.*?nsec\s*:\s*([-+0-9.eE]+)", debug_text, re.S)
    nsec = 0.0 if nsec_match is None else float(nsec_match.group(1))
    return {
        "sim_time_s": sec + nsec * 1e-9,
        "real_time_factor": _field(debug_text, "real_time_factor", 0.0),
        "iterations": _field(debug_text, "iterations", 0.0),
    }


def _field(text: str, name: str, default: float = 0.0) -> float:
    match = re.search(rf"\b{name}\s*:\s*([-+0-9.eE]+)", text)
    return default if match is None else float(match.group(1))


def parse_joint_state(debug_text: str) -> dict[str, float] | None:
    names = re.findall(r'\bname\s*:\s*"([^"]+)"', debug_text)
    velocities = [float(value) for value in re.findall(r'\bvelocity\s*:\s*([-+0-9.eE]+)', debug_text)]
    if not names or not velocities:
        return None

    result: dict[str, float] = {}
    for index, name in enumerate(names):
        if index >= len(velocities):
            break
        if "upper_rotor_joint" in name:
            result["upper_rad_s"] = abs(velocities[index])
        elif "lower_rotor_joint" in name:
            result["lower_rad_s"] = abs(velocities[index])
    return result or None


def parse_string_msg_json(debug_text: str) -> dict[str, object] | None:
    match = re.search(r'\bdata\s*:\s*"((?:\\.|[^"])*)"', debug_text, re.S)
    if match is None:
        return None
    try:
        decoded = json.loads('"' + match.group(1) + '"')
        return json.loads(decoded)
    except json.JSONDecodeError:
        return None


def read_plugin_status(partition: str, model: str) -> dict[str, object] | None:
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    topic = f"/{model}/control/status"
    try:
        output = run_cmd(["gz", "topic", "-e", "-t", topic, "-n", "1"], env, timeout=1.5)
    except Exception:
        return None
    data = parse_string_msg_json(output)
    if not data:
        return None
    data["topic"] = topic
    return data


def read_rotor_water_status(partition: str, model: str) -> dict[str, object] | None:
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    topic = f"/{model}/rotor_water/status"
    try:
        output = run_cmd(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            env,
            timeout=1.5,
        )
    except Exception:
        return None
    data = parse_string_msg_json(output)
    if not data:
        return None
    data["topic"] = topic
    return data


def read_landing_target_status(partition: str) -> dict[str, object] | None:
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    topic = "/coaxial_uav/landing/target/status"
    try:
        output = run_cmd(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            env,
            timeout=1.5,
        )
    except Exception:
        return None
    data = parse_string_msg_json(output)
    if not data:
        return None
    data["topic"] = topic
    return data


def read_commanded_motor_speed() -> dict[str, object] | None:
    state_file = RUNTIME_DIR / "motor_command.json"
    if not state_file.exists():
        return None
    try:
        data = json.loads(state_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return {
        "upper_rad_s": float(data.get("upper_rad_s", 0.0)),
        "lower_rad_s": float(data.get("lower_rad_s", 0.0)),
        "source": "commanded",
        "updated_unix_s": float(data.get("updated_unix_s", 0.0)),
    }


def write_commanded_motor_speed(upper: float, lower: float, source: str = "controller") -> None:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "upper_rad_s": upper,
        "lower_rad_s": lower,
        "source": source,
        "updated_unix_s": time.time(),
    }
    (RUNTIME_DIR / "motor_command.json").write_text(json.dumps(payload), encoding="utf-8")


def publish_motor_speed(topic: str, upper: float, lower: float, env: dict[str, str]) -> None:
    run_cmd(["gz", "topic", "-t", topic, "-m", "gz.msgs.Actuators", "-p", f"velocity:[{upper:.6f}, {lower:.6f}]"], env)
    write_commanded_motor_speed(upper, lower)


def publish_wrench(topic: str, link_name: str, roll_nm: float, pitch_nm: float, yaw_nm: float, env: dict[str, str]) -> None:
    payload = (
        f"entity: {{name: '{link_name}', type: LINK}}, "
        f"wrench: {{torque: {{x: {roll_nm:.6f}, y: {pitch_nm:.6f}, z: {yaw_nm:.6f}}}}}"
    )
    run_cmd(["gz", "topic", "-t", topic, "-m", "gz.msgs.EntityWrench", "-p", payload], env)


def clear_wrench(topic: str, link_name: str, env: dict[str, str]) -> None:
    run_cmd(["gz", "topic", "-t", topic, "-m", "gz.msgs.Entity", "-p", f"name: '{link_name}', type: LINK"], env)


def deep_update(base: dict[str, object], update: dict[str, object]) -> dict[str, object]:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_update(base[key], value)  # type: ignore[index,arg-type]
        else:
            base[key] = value
    return base


def load_tuning_config() -> dict[str, object]:
    config = deepcopy(DEFAULT_TUNING_CONFIG)
    try:
        saved = json.loads(TUNING_CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return config
    if isinstance(saved, dict):
        deep_update(config, saved)
    return config


def save_tuning_config(config: dict[str, object]) -> None:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    tmp_path = TUNING_CONFIG_PATH.with_suffix(".json.tmp")
    tmp_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    tmp_path.replace(TUNING_CONFIG_PATH)


def backup_tuning_config(config: dict[str, object]) -> dict[str, object]:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    created_unix_s = time.time()
    payload = {
        "created_unix_s": created_unix_s,
        "reason": "before_restore_defaults",
        "config": config,
    }
    tmp_path = TUNING_BACKUP_PATH.with_suffix(".json.tmp")
    tmp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp_path.replace(TUNING_BACKUP_PATH)
    return {
        "path": str(TUNING_BACKUP_PATH.relative_to(PROJECT_ROOT)),
        "created_unix_s": created_unix_s,
    }


def pid_from(config: dict[str, object], axis: str) -> PID:
    data = config[axis]  # type: ignore[index]
    return PID(
        kp=float(data["kp"]),  # type: ignore[index]
        ki=float(data["ki"]),  # type: ignore[index]
        kd=float(data["kd"]),  # type: ignore[index]
        limit=float(data["limit"]),  # type: ignore[index]
        integral_limit=float(data["integral_limit"]),  # type: ignore[index]
    )


def axis_state_value(state: dict[str, object], axis: str) -> float:
    if axis == "z":
        return float(state["position"]["z"])  # type: ignore[index]
    return float(state["attitude"][f"{axis}_rad"])  # type: ignore[index]


def axis_config_key(axis: str) -> str:
    if axis in ("x", "y"):
        return f"target_{axis}_m"
    if axis in ("vx", "vy"):
        return f"target_{axis}_m_s"
    return "target_z_m" if axis == "z" else f"target_{axis}_rad"


def test_step_for_axis(test: dict[str, object], axis: str) -> float:
    if axis in ("x", "y"):
        step = clamp(float(test.get("step", 2.0)), -10.0, 10.0)
    elif axis in ("vx", "vy"):
        step = clamp(float(test.get("step", 1.0)), -5.0, 5.0)
    else:
        attitude_limit_deg = 180.0 if axis == "yaw" else 45.0
        step_deg = clamp(
            float(test.get("step", 15.0)),
            -attitude_limit_deg,
            attitude_limit_deg,
        )
        step = math.radians(step_deg)
    if abs(step) < 1e-6:
        raise ValueError("step amplitude must be non-zero")
    return step


def requested_step_unit(axis: str) -> str:
    if axis in ("x", "y"):
        return "m"
    if axis in ("vx", "vy"):
        return "m/s"
    return "deg"


def config_for_test_axis(config: dict[str, object], axis: str) -> dict[str, object]:
    prepared = deepcopy(config)
    is_velocity = axis in ("vx", "vy")
    is_position = axis in ("x", "y")
    prepared["velocity_control_enabled"] = is_velocity
    prepared["position_control_enabled"] = is_position
    if is_velocity or is_position:
        prepared["target_roll_rad"] = 0.0
        prepared["target_pitch_rad"] = 0.0
    return prepared


def axis_output_value(state: dict[str, object], axis: str) -> float | None:
    plugin_status = state.get("motors", {}).get("plugin_status")  # type: ignore[union-attr]
    if not isinstance(plugin_status, dict):
        return None
    if axis == "z":
        return float(plugin_status.get("motor_omega_rad_s", 0.0))
    return float(plugin_status.get(f"{axis}_torque_nm", 0.0))


def axis_output_limit(config: dict[str, object], axis: str) -> float:
    if axis == "z":
        return float(config.get("max_omega_rad_s", 0.0))
    if axis in ("vx", "vy"):
        axis_config = config[f"velocity_{axis[-1]}"]  # type: ignore[index]
        return min(
            float(axis_config["limit"]),  # type: ignore[index]
            float(config.get("velocity_accel_limit_m_s2", 0.0)),
        )
    if axis in ("x", "y"):
        axis_config = config[f"position_{axis}"]  # type: ignore[index]
        return min(
            float(axis_config["limit"]),  # type: ignore[index]
            float(config.get("position_velocity_limit_m_s", 0.0)),
        )
    axis_config = config[axis]  # type: ignore[index]
    return float(axis_config["limit"])  # type: ignore[index]


def response_metrics(
    samples: list[dict[str, object]],
    axis: str,
    initial_target: float,
    final_target: float,
    band_fraction: float,
    output_limit: float,
) -> dict[str, object]:
    if not samples:
        return {"ok": False, "message": "no samples collected"}

    step = final_target - initial_target
    abs_step = abs(step)
    onset_threshold = max(abs_step * 1e-3, 1e-7)
    onset_index = next(
        (
            index
            for index, sample in enumerate(samples)
            if abs(float(sample.get("filtered_target", initial_target)) - initial_target) >= onset_threshold
        ),
        0,
    )
    samples = samples[onset_index:]
    if not samples:
        return {"ok": False, "message": "setpoint transition not observed"}

    direction = 1.0 if step >= 0.0 else -1.0
    values = [float(sample["value"]) for sample in samples]
    onset_time = float(samples[0]["t_s"])
    times = [max(0.0, float(sample["t_s"]) - onset_time) for sample in samples]
    final_value = values[-1]
    final_error = final_target - final_value
    max_abs_error = max(abs(final_target - value) for value in values)
    max_abs_value = max(abs(value) for value in values)
    saturation_count = sum(1 for sample in samples if bool(sample.get("saturated")))

    if abs_step < 1e-9:
        return {
            "ok": True,
            "axis": axis,
            "initial_target": initial_target,
            "final_target": final_target,
            "sample_count": len(samples),
            "final_value": final_value,
            "steady_state_error": final_error,
            "max_abs_error": max_abs_error,
            "max_abs_value": max_abs_value,
            "saturation_ratio": saturation_count / len(samples),
            "message": "zero step; only regulation metrics are meaningful",
        }

    normalized = [(value - initial_target) * direction for value in values]
    target_mag = abs_step
    peak_response = max(normalized)
    overshoot = max(0.0, (peak_response - target_mag) / target_mag)

    rise_start: float | None = None
    rise_end: float | None = None
    for t_s, response in zip(times, normalized):
        if rise_start is None and response >= 0.1 * target_mag:
            rise_start = t_s
        if rise_end is None and response >= 0.9 * target_mag:
            rise_end = t_s
            break
    rise_time = None if rise_start is None or rise_end is None else max(0.0, rise_end - rise_start)

    peak_index = max(range(len(samples)), key=lambda index: normalized[index])
    peak_time = times[peak_index]
    band = max(abs_step * band_fraction, 1e-6 if axis != "z" else 0.002)
    settling_time: float | None = None
    for index, t_s in enumerate(times):
        if all(abs(final_target - value) <= band for value in values[index:]):
            settling_time = t_s
            break

    outputs = [sample.get("output") for sample in samples]
    finite_outputs = [float(value) for value in outputs if value is not None]
    max_abs_output = max((abs(value) for value in finite_outputs), default=0.0)
    sim_times = [float(sample.get("sim_time_s", 0.0)) for sample in samples]
    sample_span = sim_times[-1] - sim_times[0] if len(sim_times) > 1 else 0.0
    measured_sample_rate = (len(sim_times) - 1) / sample_span if sample_span > 0.0 else 0.0
    is_velocity = axis in ("vx", "vy")
    is_position = axis in ("x", "y")
    rise_min = 0.0 if is_position else (0.6 if is_velocity else 0.2)
    rise_max = 1.0 if is_position else (1.2 if is_velocity else 0.4)
    overshoot_max = 15.0 if is_position else (10.0 if is_velocity else 15.0)
    settling_max = 3.0 if (is_velocity or is_position) else 1.5
    steady_error_max = 0.10 if is_position else (0.05 if is_velocity else None)
    rise_pass = rise_time is not None and rise_min <= rise_time <= rise_max
    overshoot_percent = overshoot * 100.0
    overshoot_pass = overshoot_percent <= overshoot_max
    settling_pass = settling_time is not None and settling_time <= settling_max
    steady_error_pass = steady_error_max is None or abs(final_error) <= steady_error_max
    sampling_pass = measured_sample_rate >= 200.0
    return {
        "ok": True,
        "axis": axis,
        "initial_target": initial_target,
        "final_target": final_target,
        "step": step,
        "sample_count": len(samples),
        "final_value": final_value,
        "steady_state_error": final_error,
        "max_abs_error": max_abs_error,
        "max_abs_value": max_abs_value,
        "peak_value": values[peak_index],
        "peak_time_s": peak_time,
        "overshoot_percent": overshoot_percent,
        "rise_time_s": rise_time,
        "settling_time_s": settling_time,
        "settling_band": band,
        "saturation_ratio": saturation_count / len(samples),
        "max_abs_output": max_abs_output,
        "output_limit": output_limit,
        "measured_sample_rate_hz": measured_sample_rate,
        "criteria": {
            "rise_time_s": {"min": rise_min, "max": rise_max, "pass": rise_pass},
            "overshoot_percent": {"max": overshoot_max, "pass": overshoot_pass},
            "settling_time_s": {
                "max": settling_max,
                "band_percent": band_fraction * 100.0,
                "pass": settling_pass,
            },
            "steady_state_error": {"max_abs": steady_error_max, "pass": steady_error_pass},
            "sample_rate_hz": {"min": 200.0, "pass": sampling_pass},
        },
    }


def chart_samples(samples: list[dict[str, object]], max_points: int = 900) -> list[dict[str, object]]:
    if len(samples) <= max_points:
        return samples.copy()

    stride = (len(samples) - 1) / (max_points - 1)
    indices = {round(index * stride) for index in range(max_points)}
    for index in range(1, len(samples)):
        if samples[index].get("phase") != samples[index - 1].get("phase"):
            indices.add(index - 1)
            indices.add(index)
    return [samples[index] for index in sorted(indices)]


def attitude_loop_estimate(config: dict[str, object], axis: str) -> dict[str, float | str]:
    if axis not in ATTITUDE_INERTIA_KG_M2:
        return {}
    axis_config = config[axis]  # type: ignore[index]
    kp = float(axis_config["kp"])  # type: ignore[index]
    ki = float(axis_config["ki"])  # type: ignore[index]
    kd = float(axis_config["kd"])  # type: ignore[index]
    inertia = ATTITUDE_INERTIA_KG_M2[axis]
    if kp <= 0.0 or kd < 0.0 or abs(ki) > 1e-9:
        return {"frequency_estimate_note": "available only for Ki=0, Kp>0 and Kd>=0"}

    natural_frequency = math.sqrt(kp / inertia)
    damping_ratio = kd / (2.0 * math.sqrt(kp * inertia))
    bandwidth_term = (
        1.0 - 2.0 * damping_ratio**2
        + math.sqrt(2.0 - 4.0 * damping_ratio**2 + 4.0 * damping_ratio**4)
    )
    bandwidth_hz = natural_frequency * math.sqrt(max(0.0, bandwidth_term)) / (2.0 * math.pi)

    ratio_squared = (
        4.0 * damping_ratio**2
        + math.sqrt(16.0 * damping_ratio**4 + 4.0)
    ) / 2.0
    crossover = natural_frequency * math.sqrt(ratio_squared)
    phase_margin_deg = math.degrees(math.atan2(kd * crossover, kp))
    return {
        "model_inertia_kg_m2": inertia,
        "natural_frequency_rad_s": natural_frequency,
        "damping_ratio_estimate": damping_ratio,
        "bandwidth_hz_estimate": bandwidth_hz,
        "phase_margin_deg_estimate": phase_margin_deg,
        "frequency_estimate_note": "rigid-body small-signal model, derivative on measured body rate",
    }


def saturation_breakdown(samples: list[dict[str, object]]) -> dict[str, object]:
    count = len(samples)

    def ratio(key: str) -> float:
        if count == 0:
            return 0.0
        return sum(bool(sample.get(key, False)) for sample in samples) / count

    return {
        "planning": {
            "position_velocity_ratio": ratio("position_velocity_saturated"),
            "velocity_acceleration_ratio": ratio("velocity_accel_saturated"),
        },
        "actuator": {
            "attitude_torque_ratio": ratio("attitude_torque_saturated"),
            "motor_speed_ratio": ratio("motor_speed_saturated"),
        },
    }


def repeat_statistics(run_metrics: list[dict[str, object]]) -> dict[str, object]:
    def summarize(values: list[float]) -> dict[str, float] | None:
        finite = [value for value in values if math.isfinite(value)]
        if not finite:
            return None
        mean = sum(finite) / len(finite)
        variance = sum((value - mean) ** 2 for value in finite) / len(finite)
        return {
            "mean": mean,
            "stddev": math.sqrt(variance),
            "min": min(finite),
            "max": max(finite),
        }

    result: dict[str, object] = {}
    for key in (
        "rise_time_s", "overshoot_percent", "settling_time_s",
        "steady_state_error", "measured_sample_rate_hz",
    ):
        summary = summarize([
            float(metrics[key])
            for metrics in run_metrics
            if metrics.get(key) is not None
        ])
        if summary is not None:
            result[key] = summary
    for category, keys in {
        "planning_saturation": (
            ("position_velocity_ratio", "position_velocity_ratio"),
            ("velocity_acceleration_ratio", "velocity_acceleration_ratio"),
        ),
        "actuator_saturation": (
            ("attitude_torque_ratio", "attitude_torque_ratio"),
            ("motor_speed_ratio", "motor_speed_ratio"),
        ),
    }.items():
        source_category = "planning" if category.startswith("planning") else "actuator"
        category_result: dict[str, object] = {}
        for output_key, source_key in keys:
            values = []
            for metrics in run_metrics:
                breakdown = metrics.get("saturation_breakdown")
                if not isinstance(breakdown, dict):
                    continue
                source = breakdown.get(source_category)
                if isinstance(source, dict) and source.get(source_key) is not None:
                    values.append(float(source[source_key]))
            summary = summarize(values)
            if summary is not None:
                category_result[output_key] = summary
        result[category] = category_result
    return result


class StreamingStatusReader:
    def __init__(self, topic: str, env: dict[str, str]) -> None:
        self.topic = topic
        self.env = env
        self.process: subprocess.Popen[str] | None = None
        self.thread: threading.Thread | None = None
        self.condition = threading.Condition()
        self.queue: deque[dict[str, object]] = deque(maxlen=50000)
        self.latest: dict[str, object] | None = None
        self.latest_received_monotonic: float | None = None
        self.error: str | None = None
        self.closed = False

    def start(self) -> None:
        self.process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.topic],
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._read_loop, name="plugin-status-reader", daemon=True)
        self.thread.start()

    def get(self, timeout: float = 3.0) -> dict[str, object]:
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.queue and self.error is None and not self.closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.condition.wait(timeout=remaining)
            if self.queue:
                return dict(self.queue.popleft())
            if self.error is not None:
                raise RuntimeError(self.error)
        raise RuntimeError(f"plugin status not received from topic {self.topic}")

    def latest_snapshot(self) -> dict[str, object] | None:
        with self.condition:
            return None if self.latest is None else dict(self.latest)

    def close(self) -> None:
        self.closed = True
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        with self.condition:
            self.condition.notify_all()

    def _read_loop(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        try:
            for line in self.process.stdout:
                data = parse_string_msg_json(line)
                if not data:
                    continue
                with self.condition:
                    self.latest = data
                    self.latest_received_monotonic = time.monotonic()
                    self.queue.append(data)
                    self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.condition.notify_all()


class PerformanceTestRunner:
    def __init__(self, controller: GazeboPluginController, partition: str, world: str, model: str) -> None:
        self.controller = controller
        self.partition = partition
        self.world = world
        self.model = model
        self.env = os.environ.copy()
        self.env["GZ_PARTITION"] = partition
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.running = False
        self.restore_running_state = True
        self.last: dict[str, object] = {
            "mode": "idle",
            "message": "dynamic performance test idle",
            "samples": [],
            "metrics": None,
        }

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            payload = deepcopy(self.last)
            payload["running"] = self.running
            return payload

    def start(self, payload: dict[str, object]) -> dict[str, object]:
        with self.lock:
            if self.running:
                payload = deepcopy(self.last)
                payload["running"] = True
                return payload
            run_payload = deepcopy(payload)
            run_payload["_controller_snapshot"] = self.controller.snapshot()
            self.running = True
            self.restore_running_state = True
            self.stop_event.clear()
            test = payload.get("test")
            if not isinstance(test, dict):
                test = {}
            axis = str(test.get("axis", "pitch"))
            self.last = {
                "mode": "starting",
                "message": "dynamic performance test starting",
                "axis": axis,
                "requested_step": test.get("step"),
                "requested_step_unit": requested_step_unit(axis),
                "repeat_count": int(clamp(float(test.get("repeat_count", 1)), 1.0, 5.0)),
                "repeat_index": 1,
                "samples": [],
                "metrics": None,
            }
            self.thread = threading.Thread(
                target=self._run,
                args=(run_payload,),
                name="performance-test",
                daemon=True,
            )
            self.thread.start()
            response = deepcopy(self.last)
            response["running"] = True
            return response

    def stop(self, restore_running_state: bool = True) -> dict[str, object]:
        with self.lock:
            self.restore_running_state = restore_running_state
        self.stop_event.set()
        thread = self.thread
        if thread is not None:
            thread.join(timeout=8.0)
        still_running = thread is not None and thread.is_alive()
        if still_running:
            self.controller.stop()
        with self.lock:
            self.running = still_running
            self.last["mode"] = "stopping" if still_running else "stopped"
            self.last["message"] = (
                "dynamic performance test cleanup in progress"
                if still_running
                else "dynamic performance test stopped"
            )
            response = deepcopy(self.last)
            response["running"] = self.running
            return response

    def _restore_controller_snapshot(self, snapshot: dict[str, object]) -> None:
        config = snapshot.get("config")
        if not isinstance(config, dict):
            return
        self.controller.update_config(deepcopy(config), persist=False)
        if bool(snapshot.get("running", False)):
            self.controller.start()
        else:
            self.controller.stop()

    def _set_progress(
        self,
        message: str,
        axis: str,
        repeat_index: int,
        repeat_count: int,
    ) -> None:
        with self.lock:
            self.last = {
                "mode": "running",
                "message": message,
                "axis": axis,
                "repeat_index": repeat_index,
                "repeat_count": repeat_count,
                "samples": [],
                "metrics": None,
                "updated_unix_s": time.time(),
            }

    def _wait_for_stable(
        self,
        status_reader: StreamingStatusReader,
        axis: str,
        target: float,
        step: float,
        repeat_index: int,
        repeat_count: int,
        message: str,
        horizontal_stop: bool = False,
        timeout_s: float = 12.0,
    ) -> None:
        self._set_progress(message, axis, repeat_index, repeat_count)
        phase_start_sim: float | None = None
        stable_start_sim: float | None = None
        wall_deadline = time.monotonic() + max(30.0, timeout_s * 10.0)
        while not self.stop_event.is_set() and time.monotonic() < wall_deadline:
            status = status_reader.get(timeout=3.0)
            sim_time = float(status.get("sim_time_s", 0.0))
            if phase_start_sim is None:
                phase_start_sim = sim_time

            roll = float(status.get("roll_rad", 0.0))
            pitch = float(status.get("pitch_rad", 0.0))
            roll_rate = float(status.get("roll_rate_rad_s", 0.0))
            pitch_rate = float(status.get("pitch_rate_rad_s", 0.0))
            yaw_rate = float(status.get("yaw_rate_rad_s", 0.0))
            horizontal_speed = math.hypot(
                float(status.get("world_vx_m_s", 0.0)),
                float(status.get("world_vy_m_s", 0.0)),
            )

            if horizontal_stop:
                stable = (
                    horizontal_speed <= 0.25
                    and abs(roll) <= math.radians(2.0)
                    and abs(pitch) <= math.radians(2.0)
                    and max(abs(roll_rate), abs(pitch_rate)) <= math.radians(5.0)
                )
            elif axis in ("roll", "pitch", "yaw"):
                value = float(status.get(f"{axis}_rad", 0.0))
                filtered_target = float(status.get(
                    f"filtered_target_{axis}_rad", target
                ))
                tolerance = max(math.radians(0.75), abs(step) * 0.03)
                stable = (
                    abs(wrap_pi(value - target)) <= tolerance
                    and abs(wrap_pi(filtered_target - target)) <= tolerance
                    and max(abs(roll_rate), abs(pitch_rate), abs(yaw_rate))
                    <= math.radians(5.0)
                )
            elif axis in ("vx", "vy"):
                value = float(status.get(f"world_{axis}_m_s", 0.0))
                tolerance = max(0.08, abs(step) * 0.03)
                stable = (
                    abs(value - target) <= tolerance
                    and max(abs(roll_rate), abs(pitch_rate)) <= math.radians(5.0)
                )
            else:
                value = float(status.get(f"world_{axis}_m", 0.0))
                tolerance = max(0.08, abs(step) * 0.03)
                stable = (
                    abs(value - target) <= tolerance
                    and horizontal_speed <= 0.20
                    and max(abs(roll_rate), abs(pitch_rate)) <= math.radians(5.0)
                )

            if stable:
                if stable_start_sim is None:
                    stable_start_sim = sim_time
                elif sim_time - stable_start_sim >= 0.5:
                    return
            else:
                stable_start_sim = None

            if sim_time - phase_start_sim > timeout_s:
                break

        if self.stop_event.is_set():
            raise RuntimeError("dynamic performance test stopped during recovery")
        raise RuntimeError(
            f"repeat {repeat_index}/{repeat_count} did not reach a stable baseline "
            f"within {timeout_s:.1f} simulation seconds"
        )

    def _restore_baseline(
        self,
        base_config: dict[str, object],
        target_key: str,
        initial_target: float,
        axis: str,
        step: float,
        status_reader: StreamingStatusReader,
        repeat_index: int,
        repeat_count: int,
        brake_horizontal: bool,
    ) -> dict[str, object]:
        baseline_config = deepcopy(base_config)
        baseline_config[target_key] = initial_target
        if brake_horizontal:
            braking_config = deepcopy(baseline_config)
            braking_config["velocity_control_enabled"] = True
            braking_config["position_control_enabled"] = False
            braking_config["target_vx_m_s"] = 0.0
            braking_config["target_vy_m_s"] = 0.0
            braking_config["target_roll_rad"] = 0.0
            braking_config["target_pitch_rad"] = 0.0
            self.controller.update_config(braking_config, persist=False)
            self.controller.start()
            self._wait_for_stable(
                status_reader, axis, initial_target, step,
                repeat_index, repeat_count,
                "braking horizontal motion",
                horizontal_stop=True,
            )

        restored = self.controller.update_config(baseline_config, persist=False)
        self.controller.start()
        self._wait_for_stable(
            status_reader, axis, initial_target, step,
            repeat_index, repeat_count,
            "restoring baseline" if brake_horizontal
            else "waiting for stable baseline",
        )
        return restored

    def _run(self, payload: dict[str, object]) -> None:
        status_reader: StreamingStatusReader | None = None
        disturbance_reader: StreamingStatusReader | None = None
        aerodynamics_reader: StreamingStatusReader | None = None
        rotor_water_reader: StreamingStatusReader | None = None
        controller_snapshot = payload.get("_controller_snapshot")
        if not isinstance(controller_snapshot, dict):
            controller_snapshot = self.controller.snapshot()
        try:
            test = payload.get("test")
            if not isinstance(test, dict):
                test = {}
            axis = str(test.get("axis", "pitch"))
            if axis not in ("roll", "pitch", "yaw", "vx", "vy", "x", "y"):
                raise ValueError(f"unsupported test axis: {axis}")
            duration_s = clamp(float(test.get("duration_s", 5.0)), 2.0, 30.0)
            sample_period_s = 0.003
            baseline_s = clamp(float(test.get("baseline_s", 2.0)), 1.0, 10.0)
            repeat_count = int(clamp(float(test.get("repeat_count", 1)), 1.0, 5.0))
            is_velocity = axis in ("vx", "vy")
            is_position = axis in ("x", "y")
            settling_band_percent = 5.0 if (is_velocity or is_position) else 2.0
            step = test_step_for_axis(test, axis)

            request_config = payload.get("config")
            if isinstance(request_config, dict):
                requested_config = request_config
            else:
                requested_config = deepcopy(self.controller.snapshot()["config"])  # type: ignore[index]
            requested_config = config_for_test_axis(requested_config, axis)
            config = self.controller.update_config(requested_config, persist=False)

            target_key = axis_config_key(axis)
            status_reader = StreamingStatusReader(f"/{self.model}/control/status", self.env)
            disturbance_reader = StreamingStatusReader(
                f"/{self.model}/disturbance/status", self.env
            )
            aerodynamics_reader = StreamingStatusReader(
                f"/{self.model}/aerodynamics/status", self.env
            )
            rotor_water_reader = StreamingStatusReader(
                f"/{self.model}/rotor_water/status", self.env
            )
            status_reader.start()
            disturbance_reader.start()
            aerodynamics_reader.start()
            rotor_water_reader.start()
            if is_position:
                current_status = status_reader.get(timeout=3.0)
                config["target_x_m"] = float(current_status["world_x_m"])
                config["target_y_m"] = float(current_status["world_y_m"])
            base_config = deepcopy(config)
            run_results: list[dict[str, object]] = []
            samples: list[dict[str, object]] = []
            metrics: dict[str, object] = {}
            initial_target = float(base_config[target_key])  # type: ignore[index]
            final_target = initial_target + step
            config = self._restore_baseline(
                base_config, target_key, initial_target, axis, step,
                status_reader, 1, repeat_count,
                brake_horizontal=False,
            )
            for repeat_index in range(repeat_count):
                if self.stop_event.is_set():
                    break
                samples = []
                self._sample_for(
                    samples, baseline_s, sample_period_s, initial_target, axis,
                    "baseline", config, status_reader, disturbance_reader,
                    aerodynamics_reader, rotor_water_reader,
                    onset_reference=None,
                    chart_duration_s=baseline_s + duration_s,
                    repeat_index=repeat_index + 1,
                    repeat_count=repeat_count,
                )
                config[target_key] = final_target
                publish_error: list[Exception] = []

                def publish_step() -> None:
                    try:
                        self.controller.update_config(config, persist=False)
                    except Exception as exc:
                        publish_error.append(exc)

                publish_thread = threading.Thread(
                    target=publish_step,
                    name="performance-step-publisher",
                    daemon=True,
                )
                publish_thread.start()
                self._sample_for(
                    samples, duration_s, sample_period_s, final_target, axis,
                    "step", config, status_reader, disturbance_reader,
                    aerodynamics_reader, rotor_water_reader,
                    onset_reference=initial_target,
                    chart_duration_s=baseline_s + duration_s,
                    repeat_index=repeat_index + 1,
                    repeat_count=repeat_count,
                )
                publish_thread.join(timeout=4.0)
                if publish_error:
                    raise publish_error[0]
                step_samples = [
                    sample for sample in samples
                    if sample.get("phase") == "step"
                ]
                metrics = response_metrics(
                    step_samples,
                    axis,
                    initial_target,
                    final_target,
                    settling_band_percent / 100.0,
                    axis_output_limit(config, axis),
                )
                metrics.update(attitude_loop_estimate(config, axis))
                metrics["saturation_breakdown"] = saturation_breakdown(step_samples)
                run_results.append({
                    "repeat_index": repeat_index + 1,
                    "initial_target": initial_target,
                    "final_target": final_target,
                    "metrics": metrics,
                    "samples": chart_samples(samples, max_points=450),
                })
                config = self._restore_baseline(
                    base_config, target_key, initial_target, axis, step,
                    status_reader, repeat_index + 1, repeat_count,
                    brake_horizontal=axis in ("roll", "pitch", "yaw"),
                )
            if not run_results:
                raise RuntimeError("dynamic performance test stopped before any repeat completed")
            result = {
                "mode": "complete",
                "message": "dynamic performance test complete",
                "axis": axis,
                "duration_s": duration_s,
                "sample_period_s": sample_period_s,
                "baseline_s": baseline_s,
                "repeat_count": repeat_count,
                "settling_band_percent": settling_band_percent,
                "requested_step": test.get("step"),
                "requested_step_unit": requested_step_unit(axis),
                "applied_step": step,
                "applied_step_unit": "rad" if axis in ("roll", "pitch", "yaw") else requested_step_unit(axis),
                "config": {**base_config, target_key: final_target},
                "restored_config": config,
                "samples": samples,
                "metrics": metrics,
                "repetitions": run_results,
                "repeat_statistics": repeat_statistics([
                    run["metrics"] for run in run_results  # type: ignore[misc]
                ]),
                "completed_unix_s": time.time(),
            }
            self._write_result(result)
            with self.lock:
                self.last = result
        except Exception as exc:
            with self.lock:
                self.last = {
                    "mode": "error",
                    "message": str(exc),
                    "samples": [],
                    "metrics": None,
                    "updated_unix_s": time.time(),
                }
        finally:
            if status_reader is not None:
                status_reader.close()
            if disturbance_reader is not None:
                disturbance_reader.close()
            if aerodynamics_reader is not None:
                aerodynamics_reader.close()
            if rotor_water_reader is not None:
                rotor_water_reader.close()
            with self.lock:
                restore_running_state = self.restore_running_state
            if self.stop_event.is_set() and not restore_running_state:
                controller_snapshot = deepcopy(controller_snapshot)
                controller_snapshot["running"] = False
            try:
                self._restore_controller_snapshot(controller_snapshot)
                with self.lock:
                    self.last["controller_state_restored"] = True
            except Exception as exc:
                self.controller.stop()
                with self.lock:
                    self.last["controller_state_restored"] = False
                    self.last["restore_error"] = str(exc)
            with self.lock:
                if self.stop_event.is_set():
                    self.last["mode"] = "stopped"
                    self.last["message"] = "dynamic performance test stopped"
                self.running = False

    def _sample_for(
        self,
        samples: list[dict[str, object]],
        duration_s: float,
        sample_period_s: float,
        target: float,
        axis: str,
        phase: str,
        config: dict[str, object],
        status_reader: StreamingStatusReader,
        disturbance_reader: StreamingStatusReader,
        aerodynamics_reader: StreamingStatusReader,
        rotor_water_reader: StreamingStatusReader,
        onset_reference: float | None,
        chart_duration_s: float,
        repeat_index: int = 1,
        repeat_count: int = 1,
    ) -> None:
        output_limit = axis_output_limit(config, axis)
        phase_start_sim: float | None = None
        onset_sim: float | None = None
        last_recorded_sim: float | None = None
        last_snapshot_sim: float | None = None
        wall_deadline = time.monotonic() + max(10.0, duration_s * 10.0)
        while not self.stop_event.is_set() and time.monotonic() < wall_deadline:
            status = status_reader.get(timeout=3.0)
            sim_time = float(status.get("sim_time_s", 0.0))
            if phase_start_sim is None:
                phase_start_sim = sim_time
            elapsed = sim_time - phase_start_sim
            if last_recorded_sim is not None and sim_time - last_recorded_sim < sample_period_s * 0.95:
                continue
            last_recorded_sim = sim_time
            if axis == "z":
                value = float(status["z_m"])
                output = float(status["motor_omega_rad_s"])
            elif axis in ("vx", "vy"):
                value = float(status[f"world_{axis}_m_s"])
                output = float(status[f"velocity_accel_{axis[-1]}_cmd_m_s2"])
            elif axis in ("x", "y"):
                value = float(status[f"world_{axis}_m"])
                output = float(status[f"position_velocity_{axis}_cmd_m_s"])
            else:
                value = float(status[f"{axis}_rad"])
                output = float(status[f"{axis}_torque_nm"])
            filtered_target = target
            if axis in ("roll", "pitch", "yaw"):
                filtered_target = float(status.get(f"filtered_target_{axis}_rad", target))
            elif axis in ("vx", "vy"):
                filtered_target = float(status.get(f"target_{axis}_m_s", target))
            elif axis in ("x", "y"):
                filtered_target = float(status.get(f"target_{axis}_m", target))
            if onset_reference is not None and onset_sim is None:
                onset_threshold = max(abs(target - onset_reference) * 1e-3, 1e-7)
                if abs(filtered_target - onset_reference) >= onset_threshold:
                    onset_sim = sim_time
            measurement_start = phase_start_sim if onset_sim is None else onset_sim
            if onset_reference is None and elapsed > duration_s:
                break
            if onset_reference is not None and onset_sim is not None:
                if sim_time - measurement_start > duration_s:
                    break
            saturated = False
            if output_limit > 0.0:
                saturated = abs(output) >= 0.98 * output_limit
            position_speed = math.hypot(
                float(status.get("position_velocity_x_cmd_m_s", 0.0)),
                float(status.get("position_velocity_y_cmd_m_s", 0.0)),
            )
            position_limit = float(config.get("position_velocity_limit_m_s", 0.0))
            position_velocity_saturated = (
                bool(status.get("position_control_enabled", False))
                and position_limit > 0.0
                and position_speed >= 0.98 * position_limit
            )
            accel_magnitude = math.hypot(
                float(status.get("velocity_accel_x_cmd_m_s2", 0.0)),
                float(status.get("velocity_accel_y_cmd_m_s2", 0.0)),
            )
            accel_limit = min(
                float(config.get("velocity_accel_limit_m_s2", 0.0)),
                9.8 * math.tan(float(config.get("velocity_tilt_limit_rad", 0.0))),
            )
            velocity_accel_saturated = (
                bool(status.get("velocity_control_enabled", False))
                or bool(status.get("position_control_enabled", False))
            ) and accel_limit > 0.0 and accel_magnitude >= 0.98 * accel_limit
            attitude_torque_saturated = any(
                abs(float(status.get(f"{name}_torque_nm", 0.0)))
                >= 0.98 * float(config[name]["limit"])  # type: ignore[index]
                for name in ("roll", "pitch", "yaw")
                if float(config[name]["limit"]) > 0.0  # type: ignore[index]
            )
            requested_motor_omega = float(status.get(
                "requested_motor_omega_rad_s",
                status.get("motor_omega_rad_s", 0.0),
            ))
            max_motor_omega = float(config.get("max_omega_rad_s", 0.0))
            motor_speed_saturated = (
                max_motor_omega > 0.0
                and requested_motor_omega >= 0.98 * max_motor_omega
            )
            disturbance = disturbance_reader.latest_snapshot() or {}
            aerodynamics = aerodynamics_reader.latest_snapshot() or {}
            rotor_water = rotor_water_reader.latest_snapshot() or {}
            wind = aerodynamics.get("wind_world_m_s", [0.0, 0.0, 0.0])
            aero_force = aerodynamics.get("force_world_n", [0.0, 0.0, 0.0])
            aero_torque = aerodynamics.get("torque_body_nm", [0.0, 0.0, 0.0])
            if not isinstance(wind, list) or len(wind) < 3:
                wind = [0.0, 0.0, 0.0]
            if not isinstance(aero_force, list) or len(aero_force) < 3:
                aero_force = [0.0, 0.0, 0.0]
            if not isinstance(aero_torque, list) or len(aero_torque) < 3:
                aero_torque = [0.0, 0.0, 0.0]
            test_start_sim = float(samples[0]["sim_time_s"]) if samples else sim_time
            sample = {
                "t_s": max(0.0, sim_time - test_start_sim),
                "phase_t_s": elapsed,
                "phase": phase,
                "axis": axis,
                "target": target,
                "filtered_target": filtered_target,
                "value": value,
                "error": target - value,
                "output": output,
                "saturated": saturated,
                "position_velocity_saturated": position_velocity_saturated,
                "velocity_accel_saturated": velocity_accel_saturated,
                "attitude_torque_saturated": attitude_torque_saturated,
                "motor_speed_saturated": motor_speed_saturated,
                "z_m": float(status.get("z_m", 0.0)),
                "roll_rad": float(status.get("roll_rad", 0.0)),
                "pitch_rad": float(status.get("pitch_rad", 0.0)),
                "yaw_rad": float(status.get("yaw_rad", 0.0)),
                "roll_rate_rad_s": float(status.get("roll_rate_rad_s", 0.0)),
                "pitch_rate_rad_s": float(status.get("pitch_rate_rad_s", 0.0)),
                "yaw_rate_rad_s": float(status.get("yaw_rate_rad_s", 0.0)),
                "measured_x_m": float(status.get("measured_x_m", status.get("world_x_m", 0.0))),
                "measured_y_m": float(status.get("measured_y_m", status.get("world_y_m", 0.0))),
                "measured_z_m": float(status.get("measured_z_m", status.get("z_m", 0.0))),
                "measured_vx_m_s": float(status.get("measured_vx_m_s", status.get("world_vx_m_s", 0.0))),
                "measured_vy_m_s": float(status.get("measured_vy_m_s", status.get("world_vy_m_s", 0.0))),
                "measured_vz_m_s": float(status.get("measured_vz_m_s", status.get("z_rate_m_s", 0.0))),
                "measured_roll_rad": float(status.get("measured_roll_rad", status.get("roll_rad", 0.0))),
                "measured_pitch_rad": float(status.get("measured_pitch_rad", status.get("pitch_rad", 0.0))),
                "measured_yaw_rad": float(status.get("measured_yaw_rad", status.get("yaw_rad", 0.0))),
                "measured_roll_rate_rad_s": float(status.get("measured_roll_rate_rad_s", status.get("roll_rate_rad_s", 0.0))),
                "measured_pitch_rate_rad_s": float(status.get("measured_pitch_rate_rad_s", status.get("pitch_rate_rad_s", 0.0))),
                "measured_yaw_rate_rad_s": float(status.get("measured_yaw_rate_rad_s", status.get("yaw_rate_rad_s", 0.0))),
                "world_vx_m_s": float(status.get("world_vx_m_s", 0.0)),
                "world_vy_m_s": float(status.get("world_vy_m_s", 0.0)),
                "world_x_m": float(status.get("world_x_m", 0.0)),
                "world_y_m": float(status.get("world_y_m", 0.0)),
                "target_vx_m_s": float(status.get("target_vx_m_s", 0.0)),
                "target_vy_m_s": float(status.get("target_vy_m_s", 0.0)),
                "velocity_accel_x_cmd_m_s2": float(status.get("velocity_accel_x_cmd_m_s2", 0.0)),
                "velocity_accel_y_cmd_m_s2": float(status.get("velocity_accel_y_cmd_m_s2", 0.0)),
                "position_velocity_x_cmd_m_s": float(status.get("position_velocity_x_cmd_m_s", 0.0)),
                "position_velocity_y_cmd_m_s": float(status.get("position_velocity_y_cmd_m_s", 0.0)),
                "motor_omega_rad_s": float(status.get("motor_omega_rad_s", 0.0)),
                "requested_motor_omega_rad_s": requested_motor_omega,
                "upper_motor_rad_s": float(status.get("upper_motor_rad_s", 0.0)),
                "lower_motor_rad_s": float(status.get("lower_motor_rad_s", 0.0)),
                "upper_motor_rpm": abs(float(status.get("upper_motor_rad_s", 0.0))) * 60.0 / (2.0 * math.pi),
                "lower_motor_rpm": abs(float(status.get("lower_motor_rad_s", 0.0))) * 60.0 / (2.0 * math.pi),
                "roll_torque_nm": float(status.get("roll_torque_nm", 0.0)),
                "pitch_torque_nm": float(status.get("pitch_torque_nm", 0.0)),
                "yaw_torque_nm": float(status.get("yaw_torque_nm", 0.0)),
                "requested_roll_torque_nm": float(status.get("requested_roll_torque_nm", 0.0)),
                "requested_pitch_torque_nm": float(status.get("requested_pitch_torque_nm", 0.0)),
                "requested_yaw_torque_nm": float(status.get("requested_yaw_torque_nm", 0.0)),
                "nonidealities_enabled": bool(status.get("nonidealities_enabled", False)),
                "yaw_large_signal_blend": float(status.get("yaw_large_signal_blend", 0.0)),
                "effective_yaw_kp": float(status.get("effective_yaw_kp", 0.0)),
                "effective_yaw_kd": float(status.get("effective_yaw_kd", 0.0)),
                "disturbance_enabled": bool(disturbance.get("enabled", False)),
                "disturbance_active": bool(disturbance.get("active", False)),
                "disturbance_preset": str(disturbance.get("preset", "off")),
                "disturbance_disk_height_m": float(disturbance.get("disk_height_m", 0.0)),
                "disturbance_height_ratio": float(disturbance.get("height_ratio", 0.0)),
                "disturbance_envelope": float(disturbance.get("envelope", 0.0)),
                "disturbance_force_z_n": float(disturbance.get("force_z_n", 0.0)),
                "disturbance_roll_torque_nm": float(disturbance.get("roll_torque_nm", 0.0)),
                "disturbance_pitch_torque_nm": float(disturbance.get("pitch_torque_nm", 0.0)),
                "disturbance_yaw_torque_nm": float(disturbance.get("yaw_torque_nm", 0.0)),
                "disturbance_sim_time_s": float(disturbance.get("sim_time_s", 0.0)),
                "aerodynamics_enabled": bool(aerodynamics.get("enabled", False)),
                "wind_x_m_s": float(wind[0]),
                "wind_y_m_s": float(wind[1]),
                "wind_z_m_s": float(wind[2]),
                "aerodynamics_force_x_n": float(aero_force[0]),
                "aerodynamics_force_y_n": float(aero_force[1]),
                "aerodynamics_force_z_n": float(aero_force[2]),
                "aerodynamics_torque_roll_nm": float(aero_torque[0]),
                "aerodynamics_torque_pitch_nm": float(aero_torque[1]),
                "aerodynamics_torque_yaw_nm": float(aero_torque[2]),
                "aerodynamics_sim_time_s": float(aerodynamics.get("sim_time_s", 0.0)),
                "rotor_interference_enabled": bool(
                    rotor_water.get("rotor_interference_enabled", False)
                ),
                "coaxial_loss_fraction": float(
                    rotor_water.get("coaxial_loss_fraction", 0.0)
                ),
                "rotor_thrust_correction_n": float(
                    rotor_water.get("rotor_thrust_correction_n", 0.0)
                ),
                "hydrodynamics_enabled": bool(
                    rotor_water.get("hydrodynamics_enabled", False)
                ),
                "water_contact": bool(rotor_water.get("water_contact", False)),
                "left_float_submerged_fraction": float(
                    rotor_water.get("left_float_submerged_fraction", 0.0)
                ),
                "right_float_submerged_fraction": float(
                    rotor_water.get("right_float_submerged_fraction", 0.0)
                ),
                "buoyancy_n": float(rotor_water.get("buoyancy_n", 0.0)),
                "slamming_force_n": float(
                    rotor_water.get("slamming_force_n", 0.0)
                ),
                "rotor_water_sim_time_s": float(
                    rotor_water.get("sim_time_s", 0.0)
                ),
                "sim_time_s": sim_time,
            }
            samples.append(sample)
            if last_snapshot_sim is None or sim_time - last_snapshot_sim >= 0.05:
                last_snapshot_sim = sim_time
                with self.lock:
                    self.last = {
                        "mode": "running",
                        "message": f"{phase} sampling",
                        "axis": axis,
                        "baseline_s": chart_duration_s - duration_s if phase == "step" else duration_s,
                        "duration_s": duration_s if phase == "step" else chart_duration_s - duration_s,
                        "chart_duration_s": chart_duration_s,
                        "repeat_index": repeat_index,
                        "repeat_count": repeat_count,
                        "samples": chart_samples(samples),
                        "metrics": None,
                        "updated_unix_s": time.time(),
                    }
        else:
            if not self.stop_event.is_set():
                raise RuntimeError(f"{phase} sampling did not advance {duration_s:.3f} simulation seconds")

    def _write_result(self, result: dict[str, object]) -> Path:
        PERFORMANCE_DIR.mkdir(parents=True, exist_ok=True)
        RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = PERFORMANCE_DIR / f"dynamic_{result.get('axis', 'axis')}_{stamp}.json"
        csv_path = path.with_suffix(".csv")
        result["saved_path"] = str(path.relative_to(PROJECT_ROOT))
        result["data_csv_path"] = str(csv_path.relative_to(PROJECT_ROOT))
        fields = [
            "phase", "t_s", "phase_t_s", "sim_time_s", "target", "filtered_target",
            "z_m",
            "roll_rad", "pitch_rad", "yaw_rad",
            "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s",
            "measured_x_m", "measured_y_m", "measured_z_m",
            "measured_vx_m_s", "measured_vy_m_s", "measured_vz_m_s",
            "measured_roll_rad", "measured_pitch_rad", "measured_yaw_rad",
            "measured_roll_rate_rad_s", "measured_pitch_rate_rad_s",
            "measured_yaw_rate_rad_s",
            "world_vx_m_s", "world_vy_m_s",
            "world_x_m", "world_y_m",
            "target_vx_m_s", "target_vy_m_s",
            "velocity_accel_x_cmd_m_s2", "velocity_accel_y_cmd_m_s2",
            "position_velocity_x_cmd_m_s", "position_velocity_y_cmd_m_s",
            "upper_motor_rad_s", "lower_motor_rad_s",
            "upper_motor_rpm", "lower_motor_rpm", "motor_omega_rad_s",
            "requested_motor_omega_rad_s",
            "roll_torque_nm", "pitch_torque_nm", "yaw_torque_nm",
            "requested_roll_torque_nm", "requested_pitch_torque_nm",
            "requested_yaw_torque_nm",
            "position_velocity_saturated", "velocity_accel_saturated",
            "attitude_torque_saturated", "motor_speed_saturated",
            "nonidealities_enabled",
            "yaw_large_signal_blend", "effective_yaw_kp", "effective_yaw_kd",
            "disturbance_enabled", "disturbance_active", "disturbance_preset",
            "disturbance_disk_height_m", "disturbance_height_ratio",
            "disturbance_envelope", "disturbance_force_z_n",
            "disturbance_roll_torque_nm", "disturbance_pitch_torque_nm",
            "disturbance_yaw_torque_nm", "disturbance_sim_time_s",
            "aerodynamics_enabled",
            "wind_x_m_s", "wind_y_m_s", "wind_z_m_s",
            "aerodynamics_force_x_n", "aerodynamics_force_y_n",
            "aerodynamics_force_z_n",
            "aerodynamics_torque_roll_nm", "aerodynamics_torque_pitch_nm",
            "aerodynamics_torque_yaw_nm", "aerodynamics_sim_time_s",
            "rotor_interference_enabled", "coaxial_loss_fraction",
            "rotor_thrust_correction_n",
            "hydrodynamics_enabled", "water_contact",
            "left_float_submerged_fraction", "right_float_submerged_fraction",
            "buoyancy_n", "slamming_force_n", "rotor_water_sim_time_s",
            "axis", "value", "error", "output", "saturated",
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(result.get("samples", []))  # type: ignore[arg-type]
        path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        latest = RUNTIME_DIR / "latest_performance_test.json"
        latest.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return path


class StreamingPoseReader:
    def __init__(self, topic: str, model_name: str, env: dict[str, str]) -> None:
        self.topic = topic
        self.model_name = model_name
        self.env = env
        self.process: subprocess.Popen[str] | None = None
        self.thread: threading.Thread | None = None
        self.condition = threading.Condition()
        self.latest: Pose | None = None
        self.error: str | None = None
        self.closed = False

    def start(self) -> None:
        self.process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.topic],
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._read_loop, name="pose-stream-reader", daemon=True)
        self.thread.start()

    def get(self, timeout: float = 3.0) -> Pose:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.latest is None and self.error is None and not self.closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.condition.wait(timeout=remaining)
            if self.latest is not None:
                return self.latest
            if self.error is not None:
                raise RuntimeError(self.error)
        raise RuntimeError(f"model pose not received from topic {self.topic}: {self.model_name}")

    def close(self) -> None:
        self.closed = True
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        with self.condition:
            self.condition.notify_all()

    def _read_loop(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        in_pose = False
        depth = 0
        current: list[str] = []
        try:
            for line in self.process.stdout:
                stripped = line.strip()
                if not in_pose and stripped == "pose {":
                    in_pose = True
                    depth = 1
                    current = [line]
                    continue
                if not in_pose:
                    continue
                current.append(line)
                depth += stripped.count("{")
                depth -= stripped.count("}")
                if depth == 0:
                    self._update_from_pose_block("\n".join(current))
                    in_pose = False
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.condition.notify_all()

    def _update_from_pose_block(self, block: str) -> None:
        if f'name: "{self.model_name}"' not in block:
            return
        position = re.search(r"position\s*\{(?P<body>.*?)\n\s*\}", block, re.S)
        orientation = re.search(r"orientation\s*\{(?P<body>.*?)\n\s*\}", block, re.S)
        pos_body = "" if position is None else position.group("body")
        ori_body = "" if orientation is None else orientation.group("body")
        pose = Pose(
            x=field(pos_body, "x"),
            y=field(pos_body, "y"),
            z=field(pos_body, "z"),
            qx=field(ori_body, "x"),
            qy=field(ori_body, "y"),
            qz=field(ori_body, "z"),
            qw=field(ori_body, "w", 1.0),
        )
        with self.condition:
            self.latest = pose
            self.condition.notify_all()


class TuningController:
    def __init__(self, partition: str, world: str, model: str) -> None:
        self.partition = partition
        self.world = world
        self.model = model
        self.link = f"{model}::base_link"
        self.env = os.environ.copy()
        self.env["GZ_PARTITION"] = partition
        self.pose_topic = f"/world/{world}/pose/info"
        self.motor_topic = f"/{model}/gazebo/command/motor_speed"
        self.wrench_topic = f"/world/{world}/wrench/persistent"
        self.clear_topic = f"/world/{world}/wrench/clear"
        self.lock = threading.Lock()
        self.config = load_tuning_config()
        self._sanitize_locked()
        self.running = False
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.last: dict[str, object] = {"mode": "idle", "message": "controller stopped"}

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "running": self.running,
                "config": json.loads(json.dumps(self.config)),
                "last": dict(self.last),
            }

    def update_config(self, update: dict[str, object], persist: bool = True) -> dict[str, object]:
        with self.lock:
            deep_update(self.config, update)
            self._sanitize_locked()
            config = deepcopy(self.config)
        return config

    def start(self) -> dict[str, object]:
        with self.lock:
            if self.running:
                return {
                    "running": self.running,
                    "config": json.loads(json.dumps(self.config)),
                    "last": dict(self.last),
                }
            self.running = True
            self.stop_event.clear()
            self.last = {"mode": "starting", "message": "controller starting"}
            self.thread = threading.Thread(target=self._loop, name="tuning-controller", daemon=True)
            self.thread.start()
            return {
                "running": self.running,
                "config": json.loads(json.dumps(self.config)),
                "last": dict(self.last),
            }

    def stop(self) -> dict[str, object]:
        self.stop_event.set()
        thread = self.thread
        if thread is not None:
            thread.join(timeout=3.0)
        self._safe_zero_outputs()
        with self.lock:
            self.running = False
            self.last = {"mode": "idle", "message": "controller stopped and outputs cleared"}
            return {
                "running": self.running,
                "config": json.loads(json.dumps(self.config)),
                "last": dict(self.last),
            }

    def _sanitize_locked(self) -> None:
        self.config["rate_hz"] = clamp(float(self.config.get("rate_hz", 4.0)), 0.5, 10.0)
        self.config["target_z_m"] = clamp(float(self.config.get("target_z_m", 0.8)), 0.0, 5.0)
        self.config["min_omega_rad_s"] = clamp(float(self.config.get("min_omega_rad_s", 0.0)), 0.0, 300.0)
        self.config["max_omega_rad_s"] = clamp(float(self.config.get("max_omega_rad_s", 167.0)), 0.0, 300.0)
        self.config["hover_omega_rad_s"] = clamp(float(self.config.get("hover_omega_rad_s", 136.362)), 0.0, 300.0)
        self.config["attitude_setpoint_rate_limit_rad_s"] = clamp(
            float(self.config.get("attitude_setpoint_rate_limit_rad_s", 0.75)), 0.0, 20.0
        )
        self.config["velocity_control_enabled"] = bool(
            self.config.get("velocity_control_enabled", False)
        )
        self.config["target_vx_m_s"] = clamp(
            float(self.config.get("target_vx_m_s", 0.0)), -10.0, 10.0
        )
        self.config["target_vy_m_s"] = clamp(
            float(self.config.get("target_vy_m_s", 0.0)), -10.0, 10.0
        )
        self.config["velocity_tilt_limit_rad"] = clamp(
            float(self.config.get("velocity_tilt_limit_rad", math.radians(15.0))),
            0.0,
            math.radians(45.0),
        )
        self.config["velocity_accel_limit_m_s2"] = clamp(
            float(self.config.get("velocity_accel_limit_m_s2", 2.0)), 0.0, 10.0
        )
        self.config["position_control_enabled"] = bool(
            self.config.get("position_control_enabled", False)
        )
        if self.config["position_control_enabled"]:
            self.config["velocity_control_enabled"] = False
        self.config["target_x_m"] = clamp(
            float(self.config.get("target_x_m", 0.0)), -100.0, 100.0
        )
        self.config["target_y_m"] = clamp(
            float(self.config.get("target_y_m", 0.0)), -100.0, 100.0
        )
        self.config["position_velocity_limit_m_s"] = clamp(
            float(self.config.get("position_velocity_limit_m_s", 2.5)), 0.0, 10.0
        )
        self.config["yaw_large_signal_kp"] = max(
            0.0, float(self.config.get("yaw_large_signal_kp", 20.0))
        )
        self.config["yaw_large_signal_kd"] = max(
            0.0, float(self.config.get("yaw_large_signal_kd", 3.0))
        )
        self.config["yaw_schedule_start_rad"] = clamp(
            float(self.config.get("yaw_schedule_start_rad", 0.02)), 0.0, math.pi
        )
        self.config["yaw_schedule_end_rad"] = clamp(
            float(self.config.get("yaw_schedule_end_rad", 0.08)),
            float(self.config["yaw_schedule_start_rad"]) + 1e-4,
            math.pi,
        )
        for axis in (
            "height", "roll", "pitch", "yaw",
            "velocity_x", "velocity_y", "position_x", "position_y",
        ):
            data = self.config[axis]  # type: ignore[index]
            data["limit"] = abs(float(data.get("limit", 0.0)))  # type: ignore[union-attr,index]
            data["integral_limit"] = abs(float(data.get("integral_limit", 0.0)))  # type: ignore[union-attr,index]
        sanitize_nonidealities(self.config)
        sanitize_aerodynamics(self.config)
        sanitize_rotor_water(self.config)
        sanitize_landing(self.config)

    def _safe_zero_outputs(self) -> None:
        try:
            publish_motor_speed(self.motor_topic, 0.0, 0.0, self.env)
        except Exception:
            pass
        try:
            clear_wrench(self.clear_topic, self.link, self.env)
        except Exception:
            pass

    def _loop(self) -> None:
        height_pid = pid_from(self.config, "height")
        height_pid.kd = 0.0
        roll_pid = pid_from(self.config, "roll")
        pitch_pid = pid_from(self.config, "pitch")
        yaw_pid = pid_from(self.config, "yaw")
        pose_reader = StreamingPoseReader(self.pose_topic, self.model, self.env)
        previous_t = time.monotonic()
        previous_z: float | None = None
        try:
            pose_reader.start()
            while not self.stop_event.is_set():
                with self.lock:
                    config = json.loads(json.dumps(self.config))
                for axis, pid in (("height", height_pid), ("roll", roll_pid), ("pitch", pitch_pid), ("yaw", yaw_pid)):
                    data = config[axis]
                    pid.kp = float(data["kp"])
                    pid.ki = float(data["ki"])
                    pid.kd = 0.0 if axis == "height" else float(data["kd"])
                    pid.limit = float(data["limit"])
                    pid.integral_limit = float(data["integral_limit"])
                now = time.monotonic()
                dt = max(now - previous_t, 1e-3)
                previous_t = now

                pose = pose_reader.get(timeout=3.0)
                roll, pitch, yaw = quat_to_euler(pose.qx, pose.qy, pose.qz, pose.qw)
                z_rate = 0.0 if previous_z is None else (pose.z - previous_z) / dt
                previous_z = pose.z

                z_error = float(config["target_z_m"]) - pose.z
                omega_delta = height_pid.update(z_error, dt) - float(config["height"]["kd"]) * z_rate
                omega = clamp(
                    float(config["hover_omega_rad_s"]) + omega_delta,
                    float(config["min_omega_rad_s"]),
                    float(config["max_omega_rad_s"]),
                )
                roll_cmd = roll_pid.update(wrap_pi(float(config["target_roll_rad"]) - roll), dt)
                pitch_cmd = pitch_pid.update(wrap_pi(float(config["target_pitch_rad"]) - pitch), dt)
                yaw_cmd = yaw_pid.update(wrap_pi(float(config["target_yaw_rad"]) - yaw), dt)

                publish_motor_speed(self.motor_topic, omega, omega, self.env)
                publish_wrench(self.wrench_topic, self.link, roll_cmd, pitch_cmd, yaw_cmd, self.env)

                with self.lock:
                    self.last = {
                        "mode": "running",
                        "z_m": pose.z,
                        "z_error_m": z_error,
                        "z_rate_m_s": z_rate,
                        "roll_rad": roll,
                        "pitch_rad": pitch,
                        "yaw_rad": yaw,
                        "motor_omega_rad_s": omega,
                        "roll_torque_nm": roll_cmd,
                        "pitch_torque_nm": pitch_cmd,
                        "yaw_torque_nm": yaw_cmd,
                        "updated_unix_s": time.time(),
                    }

                period = 1.0 / max(float(config["rate_hz"]), 0.5)
                time.sleep(max(0.0, period - (time.monotonic() - now)))
        except Exception as exc:
            with self.lock:
                self.last = {"mode": "error", "message": str(exc)}
        finally:
            pose_reader.close()
            self._safe_zero_outputs()
            with self.lock:
                self.running = False


class GazeboPluginController:
    def __init__(self, partition: str, world: str, model: str) -> None:
        self.partition = partition
        self.world = world
        self.model = model
        self.env = os.environ.copy()
        self.env["GZ_PARTITION"] = partition
        self.topic = f"/{model}/control/config"
        self.lock = threading.Lock()
        self.publisher_lock = threading.Lock()
        self.publisher_process: subprocess.Popen[str] | None = None
        self.config = load_tuning_config()
        self._sanitize_locked()
        self.running = False
        self.last: dict[str, object] = {"mode": "idle", "message": "Gazebo PID plugin stopped"}
        self._start_config_publisher()

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "running": self.running,
                "config": json.loads(json.dumps(self.config)),
                "last": dict(self.last),
            }

    def update_config(
        self,
        update: dict[str, object],
        persist: bool = True,
        publish_enabled: bool | None = None,
        publish_if_running: bool = True,
    ) -> dict[str, object]:
        with self.lock:
            deep_update(self.config, update)
            self._sanitize_locked()
            config = json.loads(json.dumps(self.config))
            running = self.running
            if publish_enabled is not None:
                self.running = publish_enabled
        if persist:
            save_tuning_config(config)
        if publish_enabled is not None:
            self._publish_config(config, publish_enabled)
        elif publish_if_running and running:
            self._publish_config(config, True)
        return config

    def restore_defaults(self) -> dict[str, object]:
        with self.lock:
            previous_config = json.loads(json.dumps(self.config))
            backup = backup_tuning_config(previous_config)
            self.config = deepcopy(DEFAULT_TUNING_CONFIG)
            self._sanitize_locked()
            self.running = False
            self.last = {
                "mode": "idle",
                "message": "default parameters restored; controller stopped",
                "updated_unix_s": time.time(),
            }
            config = json.loads(json.dumps(self.config))
            last = dict(self.last)
        save_tuning_config(config)
        self._publish_config(config, False, reset_integrators=True)
        write_commanded_motor_speed(0.0, 0.0, source="plugin_stop")
        return {
            "running": False,
            "config": config,
            "last": last,
            "backup": backup,
        }

    def close(self) -> None:
        with self.publisher_lock:
            process = self.publisher_process
            self.publisher_process = None
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()

    def start(self) -> dict[str, object]:
        with self.lock:
            self.running = True
            config = json.loads(json.dumps(self.config))
        self._publish_config(config, True, reset_integrators=True)
        with self.lock:
            self.last = {"mode": "running", "message": "Gazebo PID plugin enabled", "updated_unix_s": time.time()}
            return {
                "running": self.running,
                "config": json.loads(json.dumps(self.config)),
                "last": dict(self.last),
            }

    def stop(self) -> dict[str, object]:
        with self.lock:
            self.running = False
            config = json.loads(json.dumps(self.config))
        self._publish_config(config, False, reset_integrators=True)
        write_commanded_motor_speed(0.0, 0.0, source="plugin_stop")
        with self.lock:
            self.last = {"mode": "idle", "message": "Gazebo PID plugin disabled", "updated_unix_s": time.time()}
            return {
                "running": self.running,
                "config": json.loads(json.dumps(self.config)),
                "last": dict(self.last),
            }

    def start_landing(self) -> dict[str, object]:
        with self.lock:
            self.running = True
            config = json.loads(json.dumps(self.config))
        self._publish_config(
            config, True, reset_integrators=True, landing_start=True
        )
        with self.lock:
            self.last = {
                "mode": "landing",
                "message": "segmented landing started",
                "updated_unix_s": time.time(),
            }
        return self.snapshot()

    def local_land(self) -> dict[str, object]:
        with self.lock:
            config = json.loads(json.dumps(self.config))
        self._publish_config(config, True, landing_local_land=True)
        with self.lock:
            self.last = {
                "mode": "landing",
                "message": "local landing requested",
                "updated_unix_s": time.time(),
            }
        return self.snapshot()

    def _sanitize_locked(self) -> None:
        self.config["rate_hz"] = clamp(float(self.config.get("rate_hz", 100.0)), 1.0, 1000.0)
        self.config["target_z_m"] = clamp(float(self.config.get("target_z_m", 0.8)), 0.0, 5.0)
        self.config["min_omega_rad_s"] = clamp(float(self.config.get("min_omega_rad_s", 0.0)), 0.0, 300.0)
        self.config["max_omega_rad_s"] = clamp(float(self.config.get("max_omega_rad_s", 150.0)), 0.0, 300.0)
        self.config["hover_omega_rad_s"] = clamp(float(self.config.get("hover_omega_rad_s", 136.362)), 0.0, 300.0)
        self.config["attitude_setpoint_rate_limit_rad_s"] = clamp(
            float(self.config.get("attitude_setpoint_rate_limit_rad_s", 0.75)), 0.0, 20.0
        )
        self.config["velocity_control_enabled"] = bool(
            self.config.get("velocity_control_enabled", False)
        )
        self.config["target_vx_m_s"] = clamp(
            float(self.config.get("target_vx_m_s", 0.0)), -10.0, 10.0
        )
        self.config["target_vy_m_s"] = clamp(
            float(self.config.get("target_vy_m_s", 0.0)), -10.0, 10.0
        )
        self.config["velocity_tilt_limit_rad"] = clamp(
            float(self.config.get("velocity_tilt_limit_rad", math.radians(15.0))),
            0.0,
            math.radians(45.0),
        )
        self.config["velocity_accel_limit_m_s2"] = clamp(
            float(self.config.get("velocity_accel_limit_m_s2", 2.0)), 0.0, 10.0
        )
        self.config["position_control_enabled"] = bool(
            self.config.get("position_control_enabled", False)
        )
        if self.config["position_control_enabled"]:
            self.config["velocity_control_enabled"] = False
        self.config["target_x_m"] = clamp(
            float(self.config.get("target_x_m", 0.0)), -100.0, 100.0
        )
        self.config["target_y_m"] = clamp(
            float(self.config.get("target_y_m", 0.0)), -100.0, 100.0
        )
        self.config["position_velocity_limit_m_s"] = clamp(
            float(self.config.get("position_velocity_limit_m_s", 2.5)), 0.0, 10.0
        )
        self.config["yaw_large_signal_kp"] = max(
            0.0, float(self.config.get("yaw_large_signal_kp", 20.0))
        )
        self.config["yaw_large_signal_kd"] = max(
            0.0, float(self.config.get("yaw_large_signal_kd", 3.0))
        )
        self.config["yaw_schedule_start_rad"] = clamp(
            float(self.config.get("yaw_schedule_start_rad", 0.02)), 0.0, math.pi
        )
        self.config["yaw_schedule_end_rad"] = clamp(
            float(self.config.get("yaw_schedule_end_rad", 0.08)),
            float(self.config["yaw_schedule_start_rad"]) + 1e-4,
            math.pi,
        )
        for axis in (
            "height", "roll", "pitch", "yaw",
            "velocity_x", "velocity_y", "position_x", "position_y",
        ):
            data = self.config[axis]  # type: ignore[index]
            data["limit"] = abs(float(data.get("limit", 0.0)))  # type: ignore[union-attr,index]
            data["integral_limit"] = abs(float(data.get("integral_limit", 0.0)))  # type: ignore[union-attr,index]
        disturbance = self.config.get("disturbance")
        if not isinstance(disturbance, dict):
            disturbance = {}
            self.config["disturbance"] = disturbance
        disturbance["enabled"] = bool(disturbance.get("enabled", False))
        preset = str(disturbance.get("preset", "off"))
        disturbance["preset"] = preset if preset in {
            "off", "calm", "mild", "strong", "asymmetric"
        } else "off"
        disturbance["seed"] = int(clamp(
            float(disturbance.get("seed", 20260726)), 0.0, 4294967295.0
        ))
        sanitize_nonidealities(self.config)
        sanitize_aerodynamics(self.config)
        sanitize_rotor_water(self.config)
        sanitize_landing(self.config)

    def _start_config_publisher(self) -> subprocess.Popen[str]:
        if not CONFIG_PUBLISHER_PATH.exists():
            raise RuntimeError(
                f"Gazebo config publisher not built: {CONFIG_PUBLISHER_PATH}"
            )
        process = subprocess.Popen(
            [str(CONFIG_PUBLISHER_PATH), self.topic],
            env=self.env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self.publisher_process = process
        return process

    def _publish_payload(self, payload: str) -> None:
        with self.publisher_lock:
            process = self.publisher_process
            if process is None or process.poll() is not None:
                process = self._start_config_publisher()
            assert process.stdin is not None and process.stdout is not None
            try:
                process.stdin.write(payload + "\n")
                process.stdin.flush()
                if process.stdout.readline().strip() != "ok":
                    raise RuntimeError("Gazebo config publisher did not acknowledge payload")
            except (BrokenPipeError, OSError):
                process = self._start_config_publisher()
                assert process.stdin is not None and process.stdout is not None
                process.stdin.write(payload + "\n")
                process.stdin.flush()
                if process.stdout.readline().strip() != "ok":
                    raise RuntimeError("Gazebo config publisher restart failed")

    def _publish_config(
        self,
        config: dict[str, object],
        enabled: bool,
        reset_integrators: bool = False,
        landing_start: bool = False,
        landing_local_land: bool = False,
    ) -> None:
        flat: dict[str, object] = {
            "enabled": enabled,
            "reset_integrators": reset_integrators,
            "landing_start": landing_start,
            "landing_local_land": landing_local_land,
            "landing_mission_active": landing_start or landing_local_land,
            "target_z_m": config["target_z_m"],
            "target_roll_rad": config["target_roll_rad"],
            "target_pitch_rad": config["target_pitch_rad"],
            "target_yaw_rad": config["target_yaw_rad"],
            "velocity_control_enabled": config["velocity_control_enabled"],
            "target_vx_m_s": config["target_vx_m_s"],
            "target_vy_m_s": config["target_vy_m_s"],
            "velocity_tilt_limit_rad": config["velocity_tilt_limit_rad"],
            "velocity_accel_limit_m_s2": config["velocity_accel_limit_m_s2"],
            "position_control_enabled": config["position_control_enabled"],
            "target_x_m": config["target_x_m"],
            "target_y_m": config["target_y_m"],
            "position_velocity_limit_m_s": config["position_velocity_limit_m_s"],
            "max_omega_rad_s": config["max_omega_rad_s"],
            "attitude_setpoint_rate_limit_rad_s": config["attitude_setpoint_rate_limit_rad_s"],
            "yaw_large_signal_kp": config["yaw_large_signal_kp"],
            "yaw_large_signal_kd": config["yaw_large_signal_kd"],
            "yaw_schedule_start_rad": config["yaw_schedule_start_rad"],
            "yaw_schedule_end_rad": config["yaw_schedule_end_rad"],
        }
        disturbance = config["disturbance"]  # type: ignore[index]
        nonidealities = config["nonidealities"]  # type: ignore[index]
        aerodynamics = config["aerodynamics"]  # type: ignore[index]
        rotor_water = config["rotor_water"]  # type: ignore[index]
        landing = config["landing"]  # type: ignore[index]
        flat.update(
            {
                "disturbance_enabled": disturbance["enabled"],  # type: ignore[index]
                "disturbance_preset": disturbance["preset"],  # type: ignore[index]
                "disturbance_seed": disturbance["seed"],  # type: ignore[index]
                "nonidealities_enabled": nonidealities["enabled"],  # type: ignore[index]
                "attitude_noise_std_rad": nonidealities["attitude_noise_std_rad"],  # type: ignore[index]
                "gyro_noise_std_rad_s": nonidealities["gyro_noise_std_rad_s"],  # type: ignore[index]
                "attitude_bias_std_rad": nonidealities["attitude_bias_std_rad"],  # type: ignore[index]
                "gyro_bias_std_rad_s": nonidealities["gyro_bias_std_rad_s"],  # type: ignore[index]
                "position_noise_std_m": nonidealities["position_noise_std_m"],  # type: ignore[index]
                "velocity_noise_std_m_s": nonidealities["velocity_noise_std_m_s"],  # type: ignore[index]
                "control_delay_s": nonidealities["control_delay_s"],  # type: ignore[index]
                "motor_time_constant_s": nonidealities["motor_time_constant_s"],  # type: ignore[index]
                "motor_rate_limit_rad_s2": nonidealities["motor_rate_limit_rad_s2"],  # type: ignore[index]
                "motor_effectiveness": nonidealities["motor_effectiveness"],  # type: ignore[index]
                "nonidealities_seed": nonidealities["seed"],  # type: ignore[index]
            }
        )
        for key in (
            "enabled",
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
        ):
            published_key = (
                "aerodynamics_enabled" if key == "enabled"
                else ("aerodynamics_seed" if key == "seed" else key)
            )
            flat[published_key] = aerodynamics[key]  # type: ignore[index]
        for key in (
            "rotor_interference_enabled",
            "coaxial_max_thrust_loss",
            "coaxial_inflow_time_constant_s",
            "hydrodynamics_enabled",
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
        ):
            flat[key] = rotor_water[key]  # type: ignore[index]
        for key in (
            "surface_mode",
            "platform_top_offset_m",
            "target_x_m",
            "target_y_m",
            "target_yaw_rad",
            "moving_target_enabled",
            "target_vx_m_s",
            "target_vy_m_s",
            "target_yaw_rate_rad_s",
            "target_status_timeout_s",
            "target_speed_limit_m_s",
            "high_hover_z_m",
            "approach_speed_m_s",
            "cruise_speed_m_s",
            "position_tolerance_m",
            "yaw_tolerance_rad",
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
            "departure_tilt_limit_rad",
            "approach_tilt_limit_rad",
            "near_tilt_limit_rad",
            "warning_tilt_rad",
            "abort_tilt_rad",
            "approach_abort_tilt_rad",
            "yaw_rate_tolerance_rad_s",
            "contact_tilt_rate_limit_rad_s",
            "settling_tilt_rate_limit_rad_s",
            "go_around_tilt_tolerance_rad",
        ):
            flat[f"landing_{key}"] = landing[key]  # type: ignore[index]
        for axis in ("height", "roll", "pitch", "yaw"):
            axis_config = config[axis]  # type: ignore[index]
            for key in ("kp", "ki", "kd", "limit"):
                flat[f"{axis}_{key}"] = axis_config[key]  # type: ignore[index]
        for axis in ("velocity_x", "velocity_y"):
            axis_config = config[axis]  # type: ignore[index]
            for key in ("kp", "ki", "kd", "limit", "integral_limit"):
                flat[f"{axis}_{key}"] = axis_config[key]  # type: ignore[index]
        for axis in ("position_x", "position_y"):
            axis_config = config[axis]  # type: ignore[index]
            for key in ("kp", "ki", "kd", "limit", "integral_limit"):
                flat[f"{axis}_{key}"] = axis_config[key]  # type: ignore[index]

        payload = json.dumps(flat, separators=(",", ":"))
        self._publish_payload(payload)


def sample_motor_state(
    partition: str,
    world: str,
    model: str,
    plugin_status: dict[str, object] | None = None,
) -> dict[str, object]:
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    joint_topic = f"/world/{world}/model/{model}/joint_state"

    status = plugin_status or read_plugin_status(partition, model)
    if status is not None:
        omega = abs(float(status.get("motor_omega_rad_s", 0.0)))
        upper = abs(float(status.get("upper_motor_rad_s", omega)))
        lower = abs(float(status.get("lower_motor_rad_s", omega)))
        return {
            "upper_rad_s": upper,
            "lower_rad_s": lower,
            "upper_rpm": upper * 60.0 / (2.0 * math.pi),
            "lower_rpm": lower * 60.0 / (2.0 * math.pi),
            "source": "pid_plugin",
            "topic": status.get("topic", f"/{model}/control/status"),
            "plugin_status": status,
        }

    try:
        output = run_cmd(["gz", "topic", "-e", "-t", joint_topic, "-n", "1"], env, timeout=1.5)
        joint_speeds = parse_joint_state(output)
        if joint_speeds is not None and "upper_rad_s" in joint_speeds and "lower_rad_s" in joint_speeds:
            return {
                **joint_speeds,
                "upper_rpm": joint_speeds.get("upper_rad_s", 0.0) * 60.0 / (2.0 * math.pi),
                "lower_rpm": joint_speeds.get("lower_rad_s", 0.0) * 60.0 / (2.0 * math.pi),
                "source": "joint_state",
                "topic": joint_topic,
            }
    except Exception:
        pass

    commanded = read_commanded_motor_speed()
    if commanded is not None:
        upper = float(commanded["upper_rad_s"])
        lower = float(commanded["lower_rad_s"])
        return {
            **commanded,
            "upper_rpm": upper * 60.0 / (2.0 * math.pi),
            "lower_rpm": lower * 60.0 / (2.0 * math.pi),
            "topic": "data/runtime/motor_command.json",
        }

    return {
        "upper_rad_s": 0.0,
        "lower_rad_s": 0.0,
        "upper_rpm": 0.0,
        "lower_rpm": 0.0,
        "source": "unavailable",
        "topic": joint_topic,
    }


def sample_state(
    partition: str,
    world: str,
    model: str,
    plugin_status: dict[str, object] | None = None,
) -> dict[str, object]:
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    pose_topic = f"/world/{world}/pose/info"
    stats_topic = f"/world/{world}/stats"
    pose = read_model_pose(pose_topic, model, env)
    roll, pitch, yaw = quat_to_euler(pose.qx, pose.qy, pose.qz, pose.qw)
    stats = parse_stats(run_cmd(["gz", "topic", "-e", "-t", stats_topic, "-n", "1"], env, timeout=3.0))
    motors = sample_motor_state(partition, world, model, plugin_status)
    rotor_water = read_rotor_water_status(partition, model)
    return {
        "ok": True,
        "timestamp": time.time(),
        "position": {"x": pose.x, "y": pose.y, "z": pose.z},
        "attitude": {
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
        },
        "stats": stats,
        "motors": motors,
        "rotor_water": rotor_water or {},
    }


class LandingRecorder:
    def __init__(
        self,
        controller: GazeboPluginController,
        partition: str,
        model: str,
    ) -> None:
        self.controller = controller
        self.env = os.environ.copy()
        self.env["GZ_PARTITION"] = partition
        self.model = model
        self.lock = threading.Lock()
        self.running = False
        self.thread: threading.Thread | None = None
        self.last: dict[str, object] = {
            "running": False,
            "state": "IDLE",
            "samples": [],
        }

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return json.loads(json.dumps(self.last))

    def start(self) -> dict[str, object]:
        with self.lock:
            if self.running:
                raise RuntimeError("segmented landing is already running")
            self.running = True
            self.last = {"running": True, "state": "STARTING", "samples": []}
            self.thread = threading.Thread(
                target=self._run, name="landing-recorder", daemon=True
            )
            self.thread.start()
            return dict(self.last)

    def local_land(self) -> dict[str, object]:
        self.controller.local_land()
        with self.lock:
            self.last["message"] = "local landing requested"
            return {
                "running": self.running,
                "state": self.last.get("state", "ALIGN"),
                "message": self.last["message"],
            }

    def _run(self) -> None:
        reader = StreamingStatusReader(
            f"/{self.model}/landing/status", self.env
        )
        samples: list[dict[str, object]] = []
        try:
            reader.start()
            self.controller.start_landing()
            start_sim: float | None = None
            previous_sim: float | None = None
            landing_seen_active = False
            deadline = time.monotonic() + 180.0
            while time.monotonic() < deadline:
                status = reader.get(timeout=3.0)
                sim_time = float(status.get("sim_time_s", 0.0))
                if start_sim is None:
                    start_sim = sim_time
                if previous_sim is not None and sim_time - previous_sim < 0.0028:
                    continue
                previous_sim = sim_time
                sample = {
                    "t_s": sim_time - start_sim,
                    "sim_time_s": sim_time,
                    "state": str(status.get("landing_state", "IDLE")),
                    "state_time_s": float(status.get("landing_state_time_s", 0.0)),
                    "x_ref_m": float(status.get("landing_target_x_m", 0.0)),
                    "y_ref_m": float(status.get("landing_target_y_m", 0.0)),
                    "yaw_ref_deg": math.degrees(float(
                        status.get("landing_target_yaw_rad", 0.0)
                    )),
                    "approach_z_m": float(
                        status.get("landing_approach_z_m", 0.0)
                    ),
                    "started_on_water": bool(
                        status.get("landing_started_on_water", False)
                    ),
                    "surface_mode": str(
                        status.get("landing_surface_mode", "water")
                    ),
                    "surface_z_m": float(
                        status.get("landing_surface_z_m", 0.0)
                    ),
                    "geometry_description_version": str(status.get(
                        "landing_geometry_description_version", ""
                    )),
                    "float_bottom_offset_m": float(status.get(
                        "landing_float_bottom_offset_m", 0.0
                    )),
                    "water_equilibrium_body_offset_m": float(status.get(
                        "landing_water_equilibrium_body_offset_m", 0.0
                    )),
                    "platform_safe_half_length_m": float(status.get(
                        "landing_platform_safe_half_length_m", 0.0
                    )),
                    "platform_safe_half_width_m": float(status.get(
                        "landing_platform_safe_half_width_m", 0.0
                    )),
                    "contact_min_clearance_m": float(status.get(
                        "landing_contact_min_clearance_m", 0.0
                    )),
                    "contact_max_clearance_m": float(status.get(
                        "landing_contact_max_clearance_m", 0.0
                    )),
                    "platform_contact": bool(
                        status.get("landing_platform_contact", False)
                    ),
                    "z_ref_m": float(status.get("landing_target_z_m", 0.0)),
                    "z_m": float(status.get("z_m", 0.0)),
                    "vz_ref_m_s": float(status.get("landing_target_vz_m_s", 0.0)),
                    "vz_m_s": float(status.get("z_rate_m_s", 0.0)),
                    "x_m": float(status.get("world_x_m", 0.0)),
                    "y_m": float(status.get("world_y_m", 0.0)),
                    "vx_m_s": float(status.get("world_vx_m_s", 0.0)),
                    "vy_m_s": float(status.get("world_vy_m_s", 0.0)),
                    "horizontal_speed_m_s": math.hypot(
                        float(status.get("world_vx_m_s", 0.0)),
                        float(status.get("world_vy_m_s", 0.0)),
                    ),
                    "position_velocity_limit_m_s": float(
                        status.get("position_velocity_limit_m_s", 0.0)
                    ),
                    "landing_speed_profile": str(
                        status.get("landing_speed_profile", "legacy")
                    ),
                    "roll_deg": math.degrees(float(status.get("roll_rad", 0.0))),
                    "pitch_deg": math.degrees(float(status.get("pitch_rad", 0.0))),
                    "yaw_deg": math.degrees(float(status.get("yaw_rad", 0.0))),
                    "roll_rate_deg_s": math.degrees(float(status.get("roll_rate_rad_s", 0.0))),
                    "pitch_rate_deg_s": math.degrees(float(status.get("pitch_rate_rad_s", 0.0))),
                    "float_clearance_m": float(status.get("float_clearance_m", 0.0)),
                    "float_signed_clearance_m": float(status.get(
                        "float_signed_clearance_m",
                        status.get("float_clearance_m", 0.0),
                    )),
                    "left_submerged": float(status.get("left_float_submerged_fraction", 0.0)),
                    "right_submerged": float(status.get("right_float_submerged_fraction", 0.0)),
                    "buoyancy_n": float(status.get("buoyancy_compensation_n", 0.0)),
                    "slamming_force_n": float(status.get("slamming_force_n", 0.0)),
                    "impact_impulse_n_s": float(status.get("landing_impact_impulse_n_s", 0.0)),
                    "touchdown_vz_m_s": float(status.get("landing_touchdown_vz_m_s", 0.0)),
                    "abort_reason": str(status.get("landing_abort_reason", "")),
                    "abort_trigger_state": str(
                        status.get("landing_abort_trigger_state", "")
                    ),
                    "abort_measured_value": float(
                        status.get("landing_abort_measured_value", 0.0)
                    ),
                    "abort_limit_value": float(
                        status.get("landing_abort_limit_value", 0.0)
                    ),
                    "motor_omega_rad_s": float(status.get("motor_omega_rad_s", 0.0)),
                    "horizontal_error_m": float(
                        status.get("landing_horizontal_error_m", 0.0)
                    ),
                    "yaw_error_deg": math.degrees(float(
                        status.get("landing_yaw_error_rad", 0.0)
                    )),
                    "target_vx_m_s": float(
                        status.get("landing_target_vx_m_s", 0.0)
                    ),
                    "target_vy_m_s": float(
                        status.get("landing_target_vy_m_s", 0.0)
                    ),
                    "target_healthy": bool(
                        status.get("landing_target_healthy", True)
                    ),
                    "target_status_age_s": float(
                        status.get("landing_target_status_age_s", 0.0)
                    ),
                    "touchdown_horizontal_error_m": float(
                        status.get(
                            "landing_touchdown_horizontal_error_m", 0.0
                        )
                    ),
                    "touchdown_relative_speed_m_s": float(
                        status.get(
                            "landing_touchdown_relative_speed_m_s", 0.0
                        )
                    ),
                    "touchdown_yaw_error_deg": math.degrees(float(
                        status.get(
                            "landing_touchdown_yaw_error_rad", 0.0
                        )
                    )),
                    "dual_contact_delay_s": float(
                        status.get("landing_dual_contact_delay_s", 0.0)
                    ),
                }
                samples.append(sample)
                with self.lock:
                    self.last = {
                        "running": True,
                        "state": sample["state"],
                        "sample_count": len(samples),
                        "samples": chart_samples(samples, max_points=500),
                        "latest": sample,
                    }
                state = sample["state"]
                landing_active = bool(status.get("landing_active", False))
                landing_seen_active = landing_seen_active or landing_active
                if state in ("LANDED", "ABORTED") and not bool(
                    landing_active
                ):
                    break
                if landing_seen_active and state == "IDLE" and not landing_active:
                    break
            else:
                raise RuntimeError("segmented landing timed out")
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            PERFORMANCE_DIR.mkdir(parents=True, exist_ok=True)
            csv_path = PERFORMANCE_DIR / f"landing_{stamp}.csv"
            json_path = PERFORMANCE_DIR / f"landing_{stamp}.json"
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
                writer.writeheader()
                writer.writerows(samples)
            summary = {
                "state": samples[-1]["state"],
                "sample_count": len(samples),
                "touchdown_vz_m_s": float(samples[-1]["touchdown_vz_m_s"]),
                "peak_impact_n": max(float(s["slamming_force_n"]) for s in samples),
                "max_abs_roll_deg": max(abs(float(s["roll_deg"])) for s in samples),
                "max_abs_pitch_deg": max(abs(float(s["pitch_deg"])) for s in samples),
                "max_horizontal_speed_m_s": max(
                    float(s["horizontal_speed_m_s"]) for s in samples
                ),
                "max_position_velocity_limit_m_s": max(
                    float(s["position_velocity_limit_m_s"]) for s in samples
                ),
                "landing_speed_profile": str(samples[-1]["landing_speed_profile"]),
                "geometry_description_version": str(
                    samples[-1]["geometry_description_version"]
                ),
                "float_bottom_offset_m": float(
                    samples[-1]["float_bottom_offset_m"]
                ),
                "water_equilibrium_body_offset_m": float(
                    samples[-1]["water_equilibrium_body_offset_m"]
                ),
                "platform_safe_half_length_m": float(
                    samples[-1]["platform_safe_half_length_m"]
                ),
                "platform_safe_half_width_m": float(
                    samples[-1]["platform_safe_half_width_m"]
                ),
                "contact_min_clearance_m": float(
                    samples[-1]["contact_min_clearance_m"]
                ),
                "contact_max_clearance_m": float(
                    samples[-1]["contact_max_clearance_m"]
                ),
                "abort_reason": str(samples[-1]["abort_reason"]),
                "abort_trigger_state": str(samples[-1]["abort_trigger_state"]),
                "abort_measured_value": float(
                    samples[-1]["abort_measured_value"]
                ),
                "abort_limit_value": float(samples[-1]["abort_limit_value"]),
                "final_horizontal_error_m": float(samples[-1]["horizontal_error_m"]),
                "final_yaw_error_deg": float(samples[-1]["yaw_error_deg"]),
                "touchdown_horizontal_error_m": float(
                    samples[-1]["touchdown_horizontal_error_m"]
                ),
                "touchdown_relative_speed_m_s": float(
                    samples[-1]["touchdown_relative_speed_m_s"]
                ),
                "touchdown_yaw_error_deg": float(
                    samples[-1]["touchdown_yaw_error_deg"]
                ),
                "dual_contact_delay_s": float(
                    samples[-1]["dual_contact_delay_s"]
                ),
                "saved_csv": str(csv_path.relative_to(PROJECT_ROOT)),
            }
            json_path.write_text(json.dumps({
                **summary, "samples": samples
            }, indent=2), encoding="utf-8")
            with self.lock:
                self.last = {
                    "running": False,
                    **summary,
                    "saved_json": str(json_path.relative_to(PROJECT_ROOT)),
                    "samples": chart_samples(samples, max_points=500),
                }
        except Exception as exc:
            with self.lock:
                self.last = {
                    "running": False,
                    "state": "ERROR",
                    "error": str(exc),
                    "samples": chart_samples(samples, max_points=500),
                }
        finally:
            reader.close()
            with self.lock:
                self.running = False


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "CoaxialUavDashboard/0.1"

    def do_GET(self) -> None:
        route = urlparse(self.path).path
        if route in ("/", "/index.html"):
            self._send_file("index.html", "text/html; charset=utf-8")
        elif route == "/styles.css":
            self._send_file("styles.css", "text/css; charset=utf-8")
        elif route == "/app.js":
            self._send_file("app.js", "application/javascript; charset=utf-8")
        elif route == "/events":
            self._events()
        elif route == "/snapshot":
            self._snapshot()
        elif route == "/control":
            self._json_response(self.server.controller.snapshot())  # type: ignore[attr-defined]
        elif route == "/test":
            self._json_response(self.server.performance.snapshot())  # type: ignore[attr-defined]
        elif route == "/landing":
            self._json_response(self.server.landing.snapshot())  # type: ignore[attr-defined]
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        route = urlparse(self.path).path
        if route == "/control/config":
            self._control_config()
        elif route == "/control/defaults":
            self._control_defaults()
        elif route == "/control/start":
            self._control_start()
        elif route == "/control/stop":
            self._control_stop()
        elif route == "/test/start":
            self._test_start()
        elif route == "/test/stop":
            self._json_response(self.server.performance.stop())  # type: ignore[attr-defined]
        elif route == "/landing/start":
            self._landing_start()
        elif route in {"/landing/local", "/landing/abort"}:
            self._json_response(self.server.landing.local_land())  # type: ignore[attr-defined]
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, fmt: str, *args: object) -> None:
        sys.stderr.write("%s - %s\n" % (self.log_date_time_string(), fmt % args))

    def _send_file(self, name: str, content_type: str) -> None:
        content = (PROJECT_ROOT / "dashboard" / name).read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _snapshot(self) -> None:
        payload = self.server.sample_json()  # type: ignore[attr-defined]
        self._json_response(payload)

    def _json_response(self, payload: dict[str, object]) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _read_json_body(self) -> dict[str, object]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        body = self.rfile.read(length).decode("utf-8")
        return json.loads(body)

    def _control_config(self) -> None:
        try:
            performance = self.server.performance.snapshot()  # type: ignore[attr-defined]
            if bool(performance.get("running", False)):
                raise RuntimeError(
                    "cannot save controller config while a dynamic performance test is running"
                )
            if bool(self.server.landing.snapshot().get("running", False)):  # type: ignore[attr-defined]
                raise RuntimeError("cannot save config while landing is running")
            payload = self._read_json_body()
            plugin_status = self.server.control_status_reader.latest_snapshot()  # type: ignore[attr-defined]
            plugin_enabled = plugin_status.get("enabled")
            updated = self.server.controller.update_config(  # type: ignore[attr-defined]
                payload,
                publish_enabled=(
                    plugin_enabled if isinstance(plugin_enabled, bool) else None
                ),
                publish_if_running=False,
            )
            self._json_response({"ok": True, "config": updated})
        except Exception as exc:
            self.send_response(HTTPStatus.CONFLICT)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": str(exc)}).encode("utf-8"))

    def _control_start(self) -> None:
        try:
            performance = self.server.performance.snapshot()  # type: ignore[attr-defined]
            if bool(performance.get("running", False)):
                raise RuntimeError(
                    "cannot start the main controller while a dynamic performance test is running"
                )
            if bool(self.server.landing.snapshot().get("running", False)):  # type: ignore[attr-defined]
                raise RuntimeError("cannot start manual control while landing is running")
            self._json_response(self.server.controller.start())  # type: ignore[attr-defined]
        except Exception as exc:
            self.send_response(HTTPStatus.CONFLICT)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": str(exc)}).encode("utf-8"))

    def _control_stop(self) -> None:
        performance = self.server.performance.snapshot()  # type: ignore[attr-defined]
        if bool(performance.get("running", False)):
            self.server.performance.stop(restore_running_state=False)  # type: ignore[attr-defined]
        if bool(self.server.landing.snapshot().get("running", False)):  # type: ignore[attr-defined]
            self.server.landing.abort()  # type: ignore[attr-defined]
        self._json_response(self.server.controller.stop())  # type: ignore[attr-defined]

    def _control_defaults(self) -> None:
        try:
            performance = self.server.performance.snapshot()  # type: ignore[attr-defined]
            if bool(performance.get("running", False)):
                raise RuntimeError(
                    "cannot restore defaults while a dynamic performance test is running"
                )
            restored = self.server.controller.restore_defaults()  # type: ignore[attr-defined]
            self._json_response({"ok": True, **restored})
        except Exception as exc:
            self.send_response(HTTPStatus.CONFLICT)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({
                "ok": False,
                "error": str(exc),
            }).encode("utf-8"))

    def _test_start(self) -> None:
        try:
            if bool(self.server.landing.snapshot().get("running", False)):  # type: ignore[attr-defined]
                raise RuntimeError("cannot start a dynamic test while landing is running")
            payload = self._read_json_body()
            result = self.server.performance.start(payload)  # type: ignore[attr-defined]
            self._json_response({"ok": True, "test": result})
        except Exception as exc:
            self.send_response(HTTPStatus.BAD_REQUEST)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": str(exc)}).encode("utf-8"))

    def _landing_start(self) -> None:
        try:
            if bool(self.server.performance.snapshot().get("running", False)):  # type: ignore[attr-defined]
                raise RuntimeError("cannot start landing during a dynamic test")
            payload = self._read_json_body()
            landing_override = payload.get("landing")
            if landing_override is not None:
                if not isinstance(landing_override, dict):
                    raise ValueError("landing override must be an object")
                self.server.controller.update_config(  # type: ignore[attr-defined]
                    {"landing": landing_override}, persist=False
                )
            controller_config = self.server.controller.snapshot().get("config", {})  # type: ignore[attr-defined]
            landing_config = (
                controller_config.get("landing", {})
                if isinstance(controller_config, dict) else {}
            )
            surface_mode = (
                str(landing_config.get("surface_mode", "water"))
                if isinstance(landing_config, dict) else "water"
            )
            moving_target = isinstance(landing_config, dict) and bool(
                landing_config.get("moving_target_enabled", False)
            )
            rotor_water = read_rotor_water_status(
                self.server.partition, self.server.model  # type: ignore[attr-defined]
            ) or {}
            if rotor_water.get("vehicle_geometry_version") != "float_geometry_v1":
                raise RuntimeError(
                    "unified vehicle geometry is not loaded; restart Gazebo"
                )
            if surface_mode == "platform":
                target_status = read_landing_target_status(
                    self.server.partition  # type: ignore[attr-defined]
                ) or {}
                platform_ready = (
                    target_status.get("platform_available") is True
                    and target_status.get("platform_mode_version")
                    == "solid_deck_v1"
                    and target_status.get("platform_height_config_version")
                    == "configurable_v1"
                    and target_status.get("surface_geometry_version")
                    == "solid_deck_geometry_v1"
                )
                if not platform_ready:
                    raise RuntimeError(
                        "configurable-height landing platform plugin is not loaded; restart Gazebo"
                    )
                controller_status = (
                    self.server.control_status_reader.latest_snapshot()  # type: ignore[attr-defined]
                )
                aircraft_x = float(controller_status.get("world_x_m", 0.0))
                aircraft_y = float(controller_status.get("world_y_m", 0.0))
                aircraft_z = float(controller_status.get("z_m", 0.0))
                target_x = float(landing_config.get("target_x_m", 0.0))
                target_y = float(landing_config.get("target_y_m", 0.0))
                target_yaw = float(landing_config.get("target_yaw_rad", 0.0))
                dx = aircraft_x - target_x
                dy = aircraft_y - target_y
                cos_yaw = math.cos(target_yaw)
                sin_yaw = math.sin(target_yaw)
                local_x = cos_yaw * dx + sin_yaw * dy
                local_y = -sin_yaw * dx + cos_yaw * dy
                half_length = float(
                    target_status.get("platform_half_length_m", 0.0)
                )
                half_width = float(
                    target_status.get("platform_half_width_m", 0.0)
                )
                float_half_length = float(
                    rotor_water.get("float_footprint_half_length_m", 0.0)
                )
                float_half_width = float(
                    rotor_water.get("float_footprint_half_width_m", 0.0)
                )
                water_level_z = float(
                    controller_config.get("rotor_water", {}).get(
                        "water_level_z_m", 0.0
                    )
                ) if isinstance(controller_config.get("rotor_water"), dict) else 0.0
                platform_top_z = water_level_z + float(
                    landing_config.get("platform_top_offset_m", 0.20)
                )
                signed_clearance = aircraft_z - float(
                    rotor_water.get("float_bottom_offset_m", 0.0)
                ) - platform_top_z
                initial_overlap = (
                    abs(local_x) <= half_length + float_half_length
                    and abs(local_y) <= half_width + float_half_width
                    and signed_clearance < float(target_status.get(
                        "initial_overlap_min_clearance_m", 0.0
                    ))
                )
                if initial_overlap:
                    raise RuntimeError(
                        "landing platform would intersect the aircraft at mission start; "
                        "move the platform landing point or take off first"
                    )
            elif moving_target:
                coupling_available = (
                    rotor_water.get(
                        "moving_target_current_coupling_available"
                    ) is True
                    or rotor_water.get("moving_target_current_coupled") is True
                    or isinstance(rotor_water.get("landing_surface_mode"), str)
                )
                if not coupling_available:
                    raise RuntimeError(
                        "moving-target water coupling plugin is not loaded; restart Gazebo"
                    )
            self._json_response(self.server.landing.start())  # type: ignore[attr-defined]
        except Exception as exc:
            self.send_response(HTTPStatus.CONFLICT)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({
                "running": False, "error": str(exc)
            }).encode("utf-8"))

    def _events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        while True:
            payload = self.server.sample_json()  # type: ignore[attr-defined]
            message = "data: " + json.dumps(payload, separators=(",", ":")) + "\n\n"
            try:
                self.wfile.write(message.encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
            time.sleep(self.server.sample_period_s)  # type: ignore[attr-defined]


class DashboardServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], args: argparse.Namespace):
        self.control_status_reader: Optional[StreamingStatusReader] = None
        self.landing_target_status_reader: Optional[StreamingStatusReader] = None
        super().__init__(address, DashboardHandler)
        self.partition = args.partition
        self.world = args.world
        self.model = args.model
        self.sample_period_s = 1.0 / max(args.rate, 0.2)
        status_env = os.environ.copy()
        status_env["GZ_PARTITION"] = args.partition
        self.control_status_reader = StreamingStatusReader(
            f"/{args.model}/control/status", status_env
        )
        self.control_status_reader.start()
        self.landing_target_status_reader = StreamingStatusReader(
            "/coaxial_uav/landing/target/status", status_env
        )
        self.landing_target_status_reader.start()
        self.controller = GazeboPluginController(args.partition, args.world, args.model)
        self.performance = PerformanceTestRunner(self.controller, args.partition, args.world, args.model)
        self.landing = LandingRecorder(
            self.controller, args.partition, args.model
        )

    def sample_json(self) -> dict[str, object]:
        try:
            payload = sample_state(
                self.partition,
                self.world,
                self.model,
                self.control_status_reader.latest_snapshot(),
            )
            payload["controller"] = self.controller.snapshot()
            payload["performance"] = self.performance.snapshot()
            payload["landing"] = self.landing.snapshot()
            payload["landing_target"] = (
                self.landing_target_status_reader.latest_snapshot()
            )
            return payload
        except Exception as exc:
            return {
                "ok": False,
                "timestamp": time.time(),
                "error": str(exc),
                "controller": self.controller.snapshot(),
                "performance": self.performance.snapshot(),
                "landing": self.landing.snapshot(),
            }

    def server_close(self) -> None:
        control_reader = getattr(self, "control_status_reader", None)
        if control_reader is not None:
            control_reader.close()
        landing_target_reader = getattr(
            self, "landing_target_status_reader", None
        )
        if landing_target_reader is not None:
            landing_target_reader.close()
        super().server_close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve a browser dashboard for Gazebo UAV state.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5223)
    parser.add_argument("--no-auto-port", action="store_true", help="fail instead of trying the next port when busy")
    parser.add_argument("--partition", default=active_partition())
    parser.add_argument("--world", default="static_water_takeoff")
    parser.add_argument("--model", default="coaxial_uav")
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--once", action="store_true", help="print one JSON sample and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.once:
        print(json.dumps(sample_state(args.partition, args.world, args.model), indent=2))
        return 0
    server = None
    selected_port = args.port
    port_candidates = [args.port] if args.no_auto_port else range(args.port, args.port + 20)
    for port in port_candidates:
        selected_port = port
        try:
            server = DashboardServer((args.host, port), args)
            break
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
            if args.no_auto_port:
                print(
                    f"Port {port} is already in use. Open the existing dashboard, "
                    f"stop that process, or start another one with --port {port + 1}.",
                    file=sys.stderr,
                    flush=True,
                )
                return 98
            continue
    if server is None:
        print(f"No free dashboard port found in {args.port}-{args.port + 19}.", file=sys.stderr, flush=True)
        return 98
    print(f"Dashboard: http://{args.host}:{selected_port}", flush=True)
    print(f"Gazebo partition={args.partition} world={args.world} model={args.model}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDashboard stopped.", flush=True)
    finally:
        server.controller.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
