#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


import os
import copy
import json
import time
import socket
import threading
import collections
import math
import numpy as np
# import cv2
# import pandas as pd
# from scipy.stats import linregress
# from scipy.signal import medfilt
# from scipy.interpolate import UnivariateSpline
# from scipy.interpolate import CubicSpline

supports_day_night = False
try:
    from datetime import datetime, timezone
    import pytz
    from datetime import datetime, timezone, timedelta
    from astral import Observer
    from astral.sun import sun
    supports_day_night = True
except:
    pass

from nepi_interfaces.msg import NavPose, NavPoseOrientation

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_track



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_auto"
logger = Logger(log_name = log_name)

# Rate-limit interval (seconds) for pt_auto_2 INPUT/OUTPUT debug logs.
# Set to <0 to disable logs, 0 to print every cycle, or >0 to throttle.
AUTO2_LOG_THROTTLE_SEC = -1.0
AUTO2_RESET_TIMEOUT_SEC = 3.0




#########################
# Time State Buffer
#########################
# Thread-safe ring buffer of timestamped samples + linear interpolation, used to
# reconstruct platform state (IMU roll/pitch, or PT pan/tilt) at a past image
# timestamp (t_img) for latency-correct target-vector estimation.
#
# OWNERSHIP: the buffer LOGIC lives here (hot-reloadable with the controller and
# unit-testable), but the app node OWNS the instances and feeds them from its
# async sensor callbacks at native rate, then resolves state at t_img and stamps
# the result onto the target dict. pt_auto_2 consumes the resolved state only.
#
# CLOCK: every timestamp (samples and query) must share ONE clock -- epoch
# seconds from nepi_sdk.get_time() / nepi_sdk.sec_from_msg_stamp(header.stamp).
# Do NOT feed float32 epoch time fields (e.g. NavPosePanTilt.timestamp,
# NavPose.time_orientation): at ~1.75e9 a float32 has ~128 s resolution, which
# destroys sub-second alignment. Use full-precision ROS header stamps, or stamp
# on arrival with nepi_sdk.get_time().
#
# INTERP: linear between bracketing samples; outside the buffered span it clamps
# to the nearest end and flags _extrapolated. No angle-wrap handling -- valid for
# this platform's bounded, fast-sampled angles (roll/pitch, pan/tilt within
# mechanical limits), NOT for wrapping yaw.

class TimeStateBuffer:

    def __init__(self, fields, maxlen = 512, max_age_sec = 5.0):
        self.fields = list(fields)
        self.maxlen = int(maxlen)
        self.max_age_sec = float(max_age_sec)
        self._t = collections.deque(maxlen = self.maxlen)
        self._v = collections.deque(maxlen = self.maxlen)
        self._lock = threading.Lock()

    def add(self, t, values):
        # t: epoch seconds (float). values: dict keyed by fields, or sequence
        # aligned to fields. Non-monotonic / non-positive stamps are dropped so
        # the buffer stays sorted for the bisection search in resolve().
        try:
            t = float(t)
        except (TypeError, ValueError):
            return False
        if t <= 0.0:
            return False
        if isinstance(values, dict):
            row = [float(values.get(f, 0.0)) for f in self.fields]
        else:
            try:
                row = [float(v) for v in values]
            except (TypeError, ValueError):
                return False
            if len(row) != len(self.fields):
                return False
        with self._lock:
            if len(self._t) > 0 and t <= self._t[-1]:
                return False
            self._t.append(t)
            self._v.append(row)
        return True

    def resolve(self, t_query):
        # Returns {field: value, ...} plus meta keys:
        #   '_valid'        True unless the buffer is empty / t_query invalid
        #   '_age_sec'      newest_sample_time - t_query (data latency at query)
        #   '_extrapolated' True if t_query fell outside the buffered span (clamped)
        out = {f: 0.0 for f in self.fields}
        out['_valid'] = False
        out['_age_sec'] = -1.0
        out['_extrapolated'] = False
        try:
            tq = float(t_query)
        except (TypeError, ValueError):
            return out
        if tq <= 0.0:
            return out
        with self._lock:
            n = len(self._t)
            if n == 0:
                return out
            ts = list(self._t)
            vs = list(self._v)
        newest_t = ts[-1]
        out['_age_sec'] = newest_t - tq
        # Clamp outside the buffered span to the nearest end (no extrapolation).
        if tq <= ts[0]:
            row = vs[0]
            for i, f in enumerate(self.fields):
                out[f] = row[i]
            out['_valid'] = True
            out['_extrapolated'] = (tq < ts[0])
            return out
        if tq >= newest_t:
            row = vs[-1]
            for i, f in enumerate(self.fields):
                out[f] = row[i]
            out['_valid'] = True
            out['_extrapolated'] = (tq > newest_t)
            return out
        # Bisection for the bracketing interval ts[lo] <= tq < ts[hi].
        lo = 0
        hi = n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= tq:
                lo = mid
            else:
                hi = mid
        t0 = ts[lo]
        t1 = ts[hi]
        v0 = vs[lo]
        v1 = vs[hi]
        span = t1 - t0
        frac = 0.0 if span <= 0.0 else (tq - t0) / span
        for i, f in enumerate(self.fields):
            out[f] = v0[i] + frac * (v1[i] - v0[i])
        out['_valid'] = True
        return out

    def newest_time(self):
        with self._lock:
            if len(self._t) == 0:
                return 0.0
            return self._t[-1]

    def clear(self):
        with self._lock:
            self._t.clear()
            self._v.clear()

ZERO_TRANSFORM_DICT = copy.deepcopy(nepi_nav.BLANK_TRANSFORM_DICT)

BLANK_TRANSFORMS_DICT = {
    'location': copy.deepcopy(ZERO_TRANSFORM_DICT),
    'heading': copy.deepcopy(ZERO_TRANSFORM_DICT),
    'orientation': copy.deepcopy(ZERO_TRANSFORM_DICT),
    'position': copy.deepcopy(ZERO_TRANSFORM_DICT),
    'altitude': copy.deepcopy(ZERO_TRANSFORM_DICT),
    'depth': copy.deepcopy(ZERO_TRANSFORM_DICT),
    'pan_tilt': copy.deepcopy(ZERO_TRANSFORM_DICT)
}


NAVPOSE_SOURCE_MESSAGE_DICT = {'NavPose' : NavPose, 'NavPoseOrientation': NavPoseOrientation}
NAVPOSE_COMPONENTS = ['location','heading','orientation','position','altitude','depth']
NAVPOSE_COMPONENT_KEYS = ['init','update','offset','reset']

DATA_DICT = {
    # Required Fields
    'data_time': 0.0,
    'process_time': 0.0,

    ##################
    # Pan Tilt Data
    'pan_tilt_max_speed_dps': 10,


    'pan_speed_start_ratio': 1.0,
    'pan_timestamp': 0,
    'pan_deg': 0.0,
    'pan_dps': 0.0,


    'tilt_speed_start_ratio': 1.0,
    'tilt_timestamp': 0,
    'tilt_deg': 0.0,
    'tilt_dps': 0.0,

    #####################
    # Pan Tilt Auto Settings

    'auto_is_night': False,
    'auto_was_night': False,
    'auto_lat': 0,
    'auto_long': 0,
    'auto_night_updated': False,
  
    'auto_pt_stop': False,
    'auto_pan_home': False,
    'auto_tilt_home': False,

    'auto_pan_click_pos_update': None,
    'auto_tilt_click_pos_update': None,
    'auto_pan_click_pos_disabled': None,
    'auto_tilt_click_pos_disabled': None,

    'auto_pan_pos_update': None,
    'auto_tilt_pos_update': None,
    'auto_pan_pos': 0,
    'auto_tilt_pos': 0,
    'auto_pos_display_title': 'Position Goal',
    'auto_pan_pos_display': 0,
    'auto_tilt_pos_display': 0,
    'auto_pan_pos_disabled': False,
    'auto_tilt_pos_disabled': False,

    'auto_pan_ratio_update': None,
    'auto_tilt_ratio_update': None,
    'auto_pan_ratio_set': 0.5,
    'auto_tilt_ratio_set': 0.5,
    'auto_pan_ratio_display': 0.5,
    'auto_tilt_ratio_display': 0.5,
    'auto_pan_ratio_disabled': False,
    'auto_tilt_ratio_disabled': False,

    'auto_pan_speed_ratio_update': None,
    'auto_tilt_speed_ratio_update': None,
    'auto_pan_speed_ratio_set': 0.5,
    'auto_tilt_speed_ratio_set': 0.5,
    'auto_pan_speed_ratio_disabled': False,
    'auto_tilt_speed_ratio_disabled': False,

    'scan_enabled': False,
    'pan_scan_enabled': False,
    'tilt_scan_enabled': False,

    'tracking_best_filter_options': copy.deepcopy(nepi_track.BEST_FILTER_OPTIONS),
    'tracking_dict': copy.deepcopy(nepi_track.BLANK_SETTINGS_DICT),
    'track_enabled': False,
    'pan_track_enabled': False,
    'tilt_track_enabled': False,

    'stab_enabled': False,
    'pan_stab_enabled': False,
    'tilt_stab_enabled': False,

    #####################
    # Auto Input Data

    'navpose_topic': '',
    'navpose_data': copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT),
    'navpose_config': None,


    'pan_min_deg': -170,
    'pan_max_deg': 170,

    'tilt_min_deg': -50,
    'tilt_max_deg': 50,

    'roll_timestamp': 0,
    'roll_deg': -999,
    'roll_dps': 0.0,

    'pitch_timestamp': 0,
    'pitch_deg': -999,
    'pitch_dps': 0.0,

    'yaw_timestamp': 0,
    'yaw_deg': -999,
    'yaw_dps': 0.0,

    'heading_timestamp': 0,
    'heading_deg': -999,
    'heading_dps': -999,

    'targets_timestamp': 0,
    'targets_list': None,

    'target_timestamp': 0,
    'target_dict': None,
    'last_target_dict': None,




    #####################
    # Auto Output Data
    'auto_pan_deg': 0.0,
    'auto_pan_adj': 0.0,
    'auto_pan_goal': 0.0,
    'auto_pan_dir': 0,
    'auto_pan_dps': 0.0,
    'auto_pan_vel_rate': 0.0,
    'auto_pan_pos_rate': 0.0,  

    'auto_tilt_deg': 0.0,
    'auto_tilt_adj': 0.0,
    'auto_tilt_goal': 0.0,
    'auto_tilt_dir': 0,
    'auto_tilt_dps': 0.0,
    'auto_tilt_vel_rate': 0.0,
    'auto_tilt_pos_rate': 0.0,    

    'last_auto_time': None,
    'last_pan_vel_time': 0,
    'last_pan_pos_time': 0,
    'last_tilt_vel_time': 0,
    'last_tilt_pos_time': 0,
     # Add Custom Fields Here
     # --- pt_auto_2 vector-controller state (additive; unused by pt_auto_1) ---
     # Per-axis servo state (integral, FF/SOGI filter memory, last cmd). Self-
     # initializes via .get(); see _blank_axis_state(). Nested so deepcopy carries it.
    'auto2_pan_state': {},
    'auto2_tilt_state': {},
    # SCAN sweep integrator state {az, dir, active}; self-inits via _blank_scan_state().
    'auto2_scan_state': {},
    # TRACK lock state {los, last_time, valid}; self-inits via _blank_track_state().
    'auto2_track_state': {},
    # STAB point-and-hold state {los, seeded}; self-inits via _blank_stab_state().
    'auto2_stab_state': {},
    # Cached mount rotations (numpy 3x3) extracted from the RUI NavPose config;
    # None -> identity until the first config arrives. auto2_twist_prev = previous
    # cycle's boat heading (ENU yaw, deg) for the yaw-stabilize de-rotation.
    'auto2_M_imu': None,
    'auto2_M_pt': None,
    'auto2_twist_prev': None,
    # Wrapped-yaw LPF state used ONLY by pt_auto_2 yaw stabilization.
    'auto2_yaw_lpf_state': {},
    # One-shot click->target-vector update {az_off, el_off} (None = none pending);
    # set by the app node, consumed by pt_auto_2 to move the STAB held vector.
    'auto2_click_update': None,
    'auto2_last_update_time': None,  # wall time of previous pt_auto_2 cycle (s)
    'auto2_last_update_mono': None,  # MONOTONIC time of previous cycle (s); dt/loop_hz source (NTP-immune)
    'auto2_cycle_ms': 0.0,           # app-node-measured duration of the PREVIOUS control step (ms)
    'auto2_overrun': 0,              # 1 if the previous step ran longer than its period
    'auto2_last_data_time': None,    # last seen data_time, for staleness watchdog
    'auto2_data_change_time': None,  # wall time when data_time last changed (s)

}


PROCESSES_DICT = dict()

DEFAULT_PROCESS = 'pt_auto_2'

#########################
# Auto Process Functions
#########################

############################

pt_auto_2_settings = {
    # Required Fields
    'auto_update_rate': 15,

    # Custom Fields. Automatically Populated in RUI
    'auto_controls_dict': {
        # --- Operating-mode vector source. The PAN enables in auto_data_dict
        #     (pan_stab/scan/track_enabled, set by the app node) pick the mode that
        #     drives the desired boat_level look vector; TILT always rides that
        #     vector's elevation (the TILT enables do nothing). Priority per cycle:
        #     STAB > LOCK (track w/ fresh target) > SCAN > HOLD. STAB forces SCAN and
        #     TRACK off (written back to auto_data_dict so the app node absorbs it). ---
        # STAB (pan_stab_enabled): point-and-hold. The held boat_level vector is
        #     seeded from where the camera points at the enable rising edge and moved
        #     by a user click (see auto2_stab_state / auto2_click_update); it has no
        #     tunable settings here.
        # SCAN (pan_scan_enabled): constant-speed pan bounce between the soft limits.
        'scan_speed_dps': 15.0,     # sweep speed (deg/s)
        'scan_el_deg': 0.0,         # elevation while sweeping (deg); 0 = horizon
        # Yaw stabilization (global): 0 = held/locked bearing is boat-relative
        # (today's proven behavior); 1 = hold the bearing in the world frame using
        # IMU yaw (heading may slowly drift). Leveling is unaffected either way.
        'yaw_stab_enable': 1,
        'yaw_lpf_tau_sec': 0.5,      # LPF tau for yaw-stab path only (s); 0 disables smoothing
        'yaw_rate_deadband_dps': 0.5, # gyro yaw rate deadband (deg/s); below this the yaw FF is zero

        # TRACK (pan_track_enabled): v2 lock onto the detected target LOS.
        'track_lost_sec': 2.0,      # drop the lock if no valid detection within this (s)
        # LOS jitter filter (single EMA on the locked target vector + jump gate).
        # Kills bbox jitter when the target is still; a large angular step (or a
        # brand-new acquisition) bypasses the smoother and snaps through.
        'track_jitter_filter_enabled': 1,  # 1 = smooth the locked LOS; 0 = raw detections
        'track_smoothing_sec': 0.3,        # EMA time constant (s); higher = smoother/more lag, 0 = gate only
        'track_jump_deg': 10.0,            # LOS step that counts as a real move -> bypass + reseed (deg)

        # NOTE: roll/pitch leveling is always applied when the IMU is valid; the
        # IMU and PT mounting rotations (M_imu / M_pt) come from the RUI NavPose
        # config, and the camera-offset signs are LOCKED in code (_AUTO2_*
        # constants) -- validated on the rig and no longer tunable here.

        # --- Per-axis servo gains (PAN_* / TILT_*) ---
        # NOTE: ff_deadband_dps / ff_cutoff_hz / ff_lead_max_deg /
        # cmd_change_dps / integral_max_dps / watchdog_sec / softstop_margin_deg are
        # LOCKED in code (_AUTO2_* constants) -- no longer tunable here.
        # NOTE: pan_cmd_sign / tilt_cmd_sign / tilt_axis_sign are LOCKED in code
        # (_AUTO2_PAN_CMD_SIGN / _AUTO2_TILT_CMD_SIGN / _AUTO2_TILT_AXIS_SIGN) --
        # validated on the rig and no longer tunable here.

        # PAN axis PID + FF (no swell prediction — ocean swell negligible on pan)
        'PAN_kp': 1.5,                  # P gain: deg of axis error -> dps
        'PAN_ki': 0.0,                  # I gain: removes steady drift/bias (deg*s -> dps)
        'PAN_reference_ff_gain': 0.7,   # commanded-rate FF gain (scan sweep; analytic, no predictor)
        'PAN_disturbance_ff_gain': 1.0, # gyro-projected boat-motion FF gain (0 = off until bench-tuned)
        'PAN_pos_deadband_deg': 0.5,    # position-error deadband; suppresses pan setpoint jitter (deg)
        'PAN_max_vel_dps': 40.0,        # commanded velocity saturation (dps, <= hardware max)
        'PAN_vel_min_dps': 0.5,         # breakaway floor: min P+I speed to beat stiction (dps)

        # TILT axis PID + FF
        'TILT_kp': 0.5,
        'TILT_ki': 0.0,
        'TILT_reference_ff_gain': 0.8,   # commanded-rate FF gain (no tilt sweep today, so usually idle)
        'TILT_disturbance_ff_gain': 1.0, # gyro-projected boat-motion FF gain (0 = off until bench-tuned)
        'TILT_pos_deadband_deg': 0.5,    # position-error deadband; suppresses tilt setpoint jitter (deg)
        'TILT_ff_lead_sec': 0.2,         # swell predictor lead horizon (s); 0 = denoise only
        'TILT_max_vel_dps': 40.0,
        'TILT_vel_min_dps': 0.5,
        # TILT swell predictor (SOGI-FLL) — one enable, remaining internals
        # hard-coded as _AUTO2_SWELL_* module constants.
        'TILT_swell_predict_enabled': 1, # master on/off for swell prediction on tilt
        'TILT_swell_sogi_k': 1.0,       # SOGI bandwidth (higher = wider passband, more noise)
        'TILT_swell_amp_min_dps': 1.0,   # min swell amplitude to engage (dps)
        'TILT_swell_conf_min': 0.2,      # confidence threshold (0-1; lower = engages more readily)

        # --- Telemetry ---
        'telem_enabled': 0,         # 1 = stream UDP/JSON telemetry for PlotJuggler
        # NOTE: the telemetry target IP is intentionally NOT a control. Control
        # values are a float32[] in the status msg, so a string IP would break
        # status serialization. Set the IP in code via _auto2_TELEM_IP below.
    }
}


# -----------------------------------------------------------------------------
# pt_auto_2 live telemetry (UDP + JSON) for real-time tuning in PlotJuggler.
# Used ONLY by pt_auto_2; pt_auto_1 and all other functions are unaffected.
# In PlotJuggler: Streaming -> UDP Server -> set port below, message protocol JSON.
# Aim telemetry at the PC running PlotJuggler by editing _auto2_TELEM_IP below.
# Failures are swallowed so telemetry can never disturb the control loop.
# -----------------------------------------------------------------------------
_auto2_TELEM_IP = "192.168.179.137"     # <-- EDIT: IP of the PC running PlotJuggler
_auto2_TELEM_PORT = 9870              # <-- match the PlotJuggler UDP Server port
_auto2_telem_sock = None

# Sidus jog direction constants. Match SingleAxisTimedSpeedMove.DIRECTION_POSITIVE
# (=1) / DIRECTION_NEGATIVE (=-1). Per driver convention positive = pan left /
# tilt down (ENU frame). Plain literals to avoid a module-level msg dependency.
_auto2_DIR_POS = 1
_auto2_DIR_NEG = -1

# -----------------------------------------------------------------------------
# LOCKED hardware sign conventions for pt_auto_2. Validated on the rig
# (2026-06-24) and frozen here: a saved auto_processes_dict can no longer
# override them, and they no longer appear as editable RUI controls (which was
# the source of the earlier "sign won't change" confusion). To change one, edit
# it here and redeploy.
# LOCKED hardware sign conventions for pt_auto_2. Validated on the rig
# (2026-06-24) and frozen here: a saved auto_processes_dict can no longer
# override them, and they no longer appear as editable RUI controls (which was
# the source of the earlier "sign won't change" confusion). To change one, edit
# it here and redeploy. (IMU roll/pitch mounting is now handled by the M_imu
# transform from the RUI NavPose config; the old _AUTO2_ROLL_SIGN/_PITCH_SIGN
# leveling signs were removed when the explicit mount chain landed.)
#   _AUTO2_TILT_AXIS_SIGN  geometry(+up tilt) -> hardware tilt-feedback convention
#   _AUTO2_PAN_CMD_SIGN    pan  axis-rate -> motor direction (pan now moving, correct)
#   _AUTO2_TILT_CMD_SIGN   tilt axis-rate -> motor direction (loop settles = correct)
# -----------------------------------------------------------------------------
_AUTO2_TILT_AXIS_SIGN = -1.0
_AUTO2_PAN_CMD_SIGN = 1.0
_AUTO2_TILT_CMD_SIGN = 1.0

# Camera-offset mapping signs for TRACK mode, validated on the rig and frozen
# here (formerly tunable track_az_sign / track_el_sign).
#   _AUTO2_TRACK_AZ_SIGN   camera azimuth offset   -> pt_base pan
#   _AUTO2_TRACK_EL_SIGN   camera elevation offset -> pt_base tilt
_AUTO2_TRACK_AZ_SIGN = 1.0
_AUTO2_TRACK_EL_SIGN = -1.0

# Sign of the reported image-roll (deg) for digital roll stabilization. The raw
# geometric value is +CCW of the camera frame about its boresight relative to the
# gravity horizon; flip this if the downstream image rotation ends up backwards
# on the bench (kept as an explicit knob because roll sign has bitten us before).
_AUTO2_IMAGE_ROLL_SIGN = 1.0

# Stab/Sweep target-vector slider ruler (boat_level az/el). Azimuth uses the live
# pan soft-limit span (gathered as pan_min_deg/pan_max_deg); elevation uses this
# fixed +/- span so the tilt slider scale is independent of the (possibly
# asymmetric) tilt soft limits. The ruler is a display/command SCALE only -- the
# held target vector is never clamped to it, so an extreme roll simply rails the
# slider thumb without disturbing the vector.
_AUTO2_SLIDER_EL_MIN = -40.0
_AUTO2_SLIDER_EL_MAX = 40.0

# LOCKED servo tuning constants for pt_auto_2 (formerly editable settings).
# Frozen here so they no longer clutter the RUI or get overridden by a saved
# auto_processes_dict. To change one, edit it here and redeploy.
_AUTO2_FF_DEADBAND_DPS = 0.5      # soft deadband on the disturbance-rate FF (dps)
_AUTO2_POS_DEADBAND_DEG = 0.5     # soft deadband on P term (deg)
_AUTO2_FF_CUTOFF_HZ = 3.0         # low-pass cutoff for the disturbance rate (Hz)
_AUTO2_FF_LEAD_MAX_DEG = 5.0      # (reserved) legacy position-lead clamp (deg)
_AUTO2_TAN_E_CLAMP = 1.5          # clamp on tan(elevation) in the pan disturbance projection
_AUTO2_CMD_CHANGE_DPS = 0.2       # min change before re-sending a jog (dps)
_AUTO2_INTEGRAL_MAX_DPS = 5.0     # anti-windup clamp on the I contribution (dps)
_AUTO2_WATCHDOG_SEC = 0.5         # stale-data timeout -> zero velocity (s)
_AUTO2_SOFTSTOP_MARGIN_DEG = 2.0  # stop outward velocity this far from limits (deg)

# Swell predictor (SOGI-FLL) locked internals — ocean-swell physics constants.
# These are algorithm internals with no operator-level intuition; to change one,
# edit here and redeploy. Swell prediction is TILT-only (PAN axis unaffected by
# ocean swell in practice).
_AUTO2_SWELL_FREQ_MIN_HZ = 0.1    # FLL min freq guard (Hz) — swell never below ~10s period
_AUTO2_SWELL_FREQ_MAX_HZ = 0.6    # FLL max freq guard (Hz) — swell never above ~1.7s period
_AUTO2_SWELL_FREQ_SEED_HZ = 0.4   # FLL initial frequency guess (Hz) — mid-band start
_AUTO2_SWELL_FLL_GAMMA = 1.0      # FLL adaptation gain (amplitude-normalized)
_AUTO2_SWELL_BIAS_HZ = 0.02       # DC-bias removal highpass cutoff (Hz); tau ~8s
_AUTO2_SWELL_CONF_HZ = 0.1        # confidence estimator LPF cutoff (Hz); tau ~1.6s

# pt_auto_2 operating modes (PAN-axis driven; see pt_auto_2 docstring).
#   HOLD  no mode active   -> both axes stop
#   STAB  hold a heading    (forces SCAN+TRACK off)
#   SCAN  sweep the horizon
#   LOCK  track a locked target
#   WATCHDOG telemetry-only code emitted when the app-node liveness check trips
_auto2_MODE_HOLD = 0
_auto2_MODE_STAB = 1
_auto2_MODE_SCAN = 2
_auto2_MODE_LOCK = 3
_auto2_MODE_WATCHDOG = 9

def _auto2_sanitize_telemetry(payload):
    # Coerce non-finite floats (NaN/Inf) to None so json.dumps emits valid JSON.
    # json.dumps does NOT raise on NaN/Inf -- it writes the bare tokens NaN /
    # Infinity, which are invalid JSON and cause strict receivers (e.g.
    # PlotJuggler) to silently drop every packet, killing the whole stream.
    out = {}
    for k, v in payload.items():
        if isinstance(v, float) and not math.isfinite(v):
            out[k] = None
        else:
            out[k] = v
    return out

def _auto2_send_telemetry(payload, ip=None):
    global _auto2_telem_sock
    try:
        if _auto2_telem_sock is None:
            _auto2_telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _auto2_telem_sock.setblocking(False)
        dest_ip = (ip or _auto2_TELEM_IP)
        _auto2_telem_sock.sendto(
            json.dumps(_auto2_sanitize_telemetry(payload), allow_nan=False).encode('utf-8'),
            (dest_ip, _auto2_TELEM_PORT))
    except Exception:
        pass  # never let telemetry break stabilization

#########################
# Vector geometry helpers (operate in the PT-base frame defined by M_pt)
#########################
# These describe pan/tilt about the PT base, independent of how the base is
# mounted (the mount is M_pt, applied separately). PT convention: +tilt looks up,
# +pan is CW-from-top (about -z); at pan=0/tilt=0 the boresight is +x, level.
# boresight d = (cos t cos p, -cos t sin p, sin t), so:
#   pan = atan2(-y, x);  tilt = atan2(z, hypot(x, y))
# Pure-Python per-vector ops (the numpy toolkit below builds the frame matrices).

def _rot_x(v, a):
    c = math.cos(a); s = math.sin(a)
    x, y, z = v
    return [x, c * y - s * z, s * y + c * z]

def _rot_y(v, a):
    c = math.cos(a); s = math.sin(a)
    x, y, z = v
    return [c * x + s * z, y, -s * x + c * z]

def _rot_z(v, a):
    c = math.cos(a); s = math.sin(a)
    x, y, z = v
    return [c * x - s * y, s * x + c * y, z]

def _wrap_to_180(a):
    # Wrap an angle (deg) to (-180, 180].
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a

def vector_from_az_el(az_deg, el_deg):
    # boat_level look vector from azimuth/elevation. az/el ARE the desired
    # pan/tilt when roll = pitch = 0 (level platform).
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    ce = math.cos(el)
    return [ce * math.cos(az), -ce * math.sin(az), math.sin(el)]

def solve_pan_tilt_from_vector(v):
    x, y, z = v
    pan = math.degrees(math.atan2(-y, x))
    tilt = math.degrees(math.atan2(z, math.sqrt(x * x + y * y)))
    return pan, tilt

#########################
# Rotation toolkit (numpy) -- explicit mount-chain frame transforms
#########################
# Conventions (pinned; see repo memory):
#   * R_to_from: v_to = R @ v_from  (subscripts read right->left, from->to).
#   * World = ENU (x=East, y=North, z=Up; gravity along -z).
#   * Body  = x-fwd, y-port, z-up (REP-103).
#   * Euler order = ZYX intrinsic (yaw about up, then pitch, then roll):
#       R_world_imu = Rz(yaw) @ Ry(pitch) @ Rx(roll).
#   * M_imu = R_body_imu (IMU-sensor coords -> body); M_pt = R_body_ptbase.
# Built fresh each cycle from the RUI mount transforms (cheap 3x3 work). The
# pure-python _rot_x/_rot_y/_rot_z above stay for the per-vector hot path.

_I3 = np.eye(3)

def _Rx(a):
    c = math.cos(a); s = math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])

def _Ry(a):
    c = math.cos(a); s = math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

def _Rz(a):
    c = math.cos(a); s = math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

def R_from_zyx(roll_deg, pitch_deg, yaw_deg):
    # Intrinsic ZYX (yaw->pitch->roll) -> rotation matrix (child-coords -> world).
    return (_Rz(math.radians(yaw_deg))
            @ _Ry(math.radians(pitch_deg))
            @ _Rx(math.radians(roll_deg)))

def apply_R(R, v):
    # Apply a 3x3 to a 3-list; return a 3-list of python floats.
    out = np.asarray(R) @ np.array([v[0], v[1], v[2]], dtype=float)
    return [float(out[0]), float(out[1]), float(out[2])]

def validate_rotation(R, tol=1e-4):
    # True if R is orthonormal with det ~ +1 (a proper rotation).
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3):
        return False
    if not np.allclose(R.T @ R, _I3, atol=tol):
        return False
    return abs(float(np.linalg.det(R)) - 1.0) <= tol

def transform_dict_to_R(tf):
    # Build a rotation from a NEPI Transform dict (BLANK_TRANSFORM_DICT shape:
    # roll_deg/pitch_deg/yaw_deg + per-axis *_invert). The invert flag negates that
    # axis' angle. Translation/heading fields are ignored (only rotation enters
    # far-field pointing). Returns a proper rotation built in ZYX order.
    if not isinstance(tf, dict):
        return _I3.copy()
    def _ang(key):
        a = float(tf.get(key + '_deg', 0.0))
        return -a if bool(tf.get(key + '_invert', False)) else a
    return R_from_zyx(_ang('roll'), _ang('pitch'), _ang('yaw'))

def swing_twist_up(R_world_body):
    # Swing-twist decomposition of a body->world rotation about WORLD up (z):
    # R_world_body = R_twist @ R_swing, where R_twist = Rz(twist) is a pure rotation
    # about world up (the boat heading / ENU yaw) and R_swing carries only the
    # roll/pitch leveling (zero azimuth). Returns (R_twist, R_swing, twist_deg).
    # The twist is the azimuth of the body x-axis (column 0) in the world EN-plane;
    # removing it leaves a swing whose body-x lies in the world x-z plane. Leveling
    # depends only on R_swing and is therefore yaw-independent (proven on the rig).
    R = np.asarray(R_world_body, dtype=float)
    bx = R[:, 0]
    twist = math.atan2(bx[1], bx[0])
    R_twist = _Rz(twist)
    R_swing = R_twist.T @ R
    return R_twist, R_swing, math.degrees(twist)


# Why the -roll/-yaw here: this path decomposes attitude in the BODY frame
# (x=fwd, y=port, z=up) after stripping the IMU mount:
#   R_world_body = R_world_imu_from_ahrs(r,p,y) @ M_imu.T
# and later maps level -> PT base with M_pt.T @ R_swing.T.
#
# On this rig both mounts are a 180deg yaw: M_imu ~= M_pt ~= Rz(pi), so the chain
# introduces a conjugation by D = diag(-1,-1,1) around the swing decomposition. That
# conjugation flips roll and yaw signs while leaving pitch unchanged. Applying
# R_from_zyx(-roll, pitch, -yaw) here pre-compensates that effect and preserves the
# proven bench behavior of the full mount chain.
#
# This is not an AHRS chirality/NED correction; it is a consequence of the chosen
# body-frame mount pipeline with these mount transforms.
def R_world_imu_from_ahrs(roll_deg, pitch_deg, yaw_deg):
    # Raw AHRS roll/pitch/yaw (deg) -> R_world_imu in right-handed ENU. Centralized
    # so every consumer (live swing, twist, and the image-time swing_at) shares it.
    return R_from_zyx(-roll_deg, pitch_deg, -yaw_deg)


#########################
# Image-roll (digital roll stabilization)
#########################
# The PT unit has pan + tilt but NO roll axis, so when the platform rolls the
# camera image rotates about its boresight. These helpers compute that image roll
# (deg) so a downstream consumer can de-rotate the frame. The angle is the roll of
# the CAMERA frame about its own boresight relative to the gravity horizon.
#
# Camera/PT-base frame convention (see solve_pan_tilt_from_vector / vector_from_az_el):
#   X = boresight (into the scene), Y = port/left, Z = up  (all at pan=0, tilt=0).
# The pan/tilt kinematics rotate that frame: pan about base Z, then tilt about the
# panned lateral axis:
#   R_ptbase_cam = Rz(-pan_geom) @ Ry(-tilt_geom)
# where pan_geom/tilt_geom are GEOMETRIC angles (+pan = solver pan, +tilt = look up).
#
# The camera axes in the gravity-leveled world are the columns of
#   R_level_cam = R_swing @ M_pt @ R_ptbase_cam
# and the image roll is the angle, about the boresight (camera X), between the
# camera Y axis and the horizon. Because only the world-up (Z) row matters, this is
#   image_roll = atan2( camY . up , camZ . up ) = atan2( R_level_cam[2,1], R_level_cam[2,2] )
# It is YAW-INDEPENDENT by construction (a twist about world-up leaves the Z row of
# R_level_cam unchanged), so the roll/pitch swing alone is sufficient -- no heading
# needed, matching the leveling path.

def R_ptbase_cam(pan_geom_deg, tilt_geom_deg):
    # Camera orientation in the PT base frame for a given geometric pan/tilt.
    return _Rz(-math.radians(pan_geom_deg)) @ _Ry(-math.radians(tilt_geom_deg))

def image_roll_deg_from_swing(R_swing, M_pt, pan_geom_deg, tilt_geom_deg):
    # Image roll (deg) about the boresight from the roll/pitch swing, the PT mount,
    # and the GEOMETRIC pan/tilt. R_swing = R_body<-level leveling (yaw stripped);
    # M_pt = R_body<-ptbase mount. Only the world-up row is used, so this is
    # heading-independent. Sign knob applied last.
    R_level_cam = np.asarray(R_swing, dtype=float) @ np.asarray(M_pt, dtype=float) \
        @ R_ptbase_cam(pan_geom_deg, tilt_geom_deg)
    roll = math.degrees(math.atan2(float(R_level_cam[2, 1]), float(R_level_cam[2, 2])))
    return _AUTO2_IMAGE_ROLL_SIGN * roll

def image_roll_deg_at(roll_deg, pitch_deg, pan_deg, tilt_deg, M_imu, M_pt):
    # Image-time entry point: given the platform state (raw AHRS roll/pitch +
    # raw PT feedback pan/tilt) reconstructed at an image timestamp, return the
    # image roll (deg). Builds the yaw-independent swing internally (heading = 0 is
    # exact). pan_deg is geometric pan; tilt_deg is the hardware tilt feedback, so
    # it is mapped to geometry (+up) with _AUTO2_TILT_AXIS_SIGN, exactly as the
    # TRACK/STAB anchoring does. Returns 0.0 if roll/pitch are invalid (-999).
    if roll_deg == -999 or pitch_deg == -999:
        return 0.0
    _t, R_swing, _d = swing_twist_up(
        R_world_imu_from_ahrs(roll_deg, pitch_deg, 0.0) @ np.asarray(M_imu, dtype=float).T)
    tilt_geom_deg = _AUTO2_TILT_AXIS_SIGN * tilt_deg
    return image_roll_deg_from_swing(R_swing, M_pt, pan_deg, tilt_geom_deg)


#########################
# Generic axis velocity servo (AxisVelocityController)
#########################
# Reusable per-axis (pan/tilt) velocity servo extracted from the proven tilt
# stabilizer: position P+I with soft deadband, desired-rate feedforward
# (d/dt desired angle, low-pass), optional SOGI-FLL swell predictor (position
# lead + predictive FF), stiction breakaway floor, anti-windup, and soft-stop.
# State lives in a per-axis dict (survives importlib.reload via DATA_DICT).

def _blank_axis_state():
    return {
        'integral': 0.0,           # accumulated position error for I term (deg*s)
        'dist_rate_filt': 0.0,     # low-pass state of the gyro disturbance-rate FF (dps)
        'last_cmd_vel_dps': 0.0,   # last velocity actually sent (dps)
        'sw_omega': None,          # SOGI-FLL angular frequency (rad/s)
        'sw_valpha': 0.0,
        'sw_vbeta': 0.0,
        'sw_bias': None,           # DC/bias EMA of the disturbance-rate signal (dps)
        'sw_e2': 0.0,
        'sw_u2': 0.0,
    }


def _blank_scan_state():
    # SCAN sweep integrator. 'active' = SCAN owned the axis last cycle, so re-entry
    # (e.g. after a lock drops) re-seeds 'az' to the live pan angle for a bump-free
    # resume. 'dir' = +/-1 current sweep direction. 'el' = boat_level sweep elevation
    # (deg); None until seeded on sweep entry, then live-adjustable via the tilt slider.
    return {'az': 0.0, 'dir': 1.0, 'active': False, 'el': None}


def _blank_track_state():
    # TRACK lock. 'los' = stored boat_level target unit vector (None = no lock);
    # 'last_time' = wall time (s) of the last ingested valid detection;
    # 'los_filter' = EMA jitter-filter memory (see _blank_los_filter_state).
    return {'los': None, 'last_time': 0.0, 'valid': False, 'los_filter': {}}


def _blank_stab_state():
    # STAB point-and-hold. 'los' = held boat_level target unit vector (None = not
    # yet seeded). 'seeded' = True once the vector has been captured on the STAB
    # enable rising edge (or moved by a user click); cleared when STAB is disabled
    # so the next enable re-seeds to wherever the camera is then pointed. The held
    # vector never times out -- only a TRACK detection lock does.
    return {'los': None, 'seeded': False}


def _blank_los_filter_state():
    # EMA filter memory for the tracked LOS unit vector.
    return {'los_hat': None,    # filtered unit vector (None = not yet seeded)
            'last_time': None}  # wall time of the last filter update (s)


def _blank_yaw_lpf_state():
    # Wrapped-angle EMA memory for yaw stabilization only.
    return {'yaw_hat_deg': None, 'last_time': None}


def _normalize3(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 1e-9:
        return [v[0], v[1], v[2]]
    return [v[0] / n, v[1] / n, v[2] / n]


def _angle_between_unit_deg(a, b):
    # Angle (deg) between two ~unit vectors; clamps the dot for the acos domain.
    d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    if d > 1.0:
        d = 1.0
    elif d < -1.0:
        d = -1.0
    return math.degrees(math.acos(d))


def _filter_los(state, los_meas, t, smoothing_sec, jump_deg):
    # Single exponential moving average on a LOS unit vector + jump gate. Smooths
    # sub-threshold bbox jitter; a large step (new/re-acquired target) bypasses the
    # smoother and snaps through. smoothing_sec is the EMA time constant (s): the
    # alpha = dt/(tau+dt) form keeps the smoothing rate-correct as dt varies.
    # Mutates `state` in place. Returns (los_unit, ang_step_deg, reseeded_bool).
    los_meas = _normalize3(los_meas)
    los_prev = state.get('los_hat', None)
    t_prev = state.get('last_time', None)
    if los_prev is None or t_prev is None:
        state['los_hat'] = los_meas
        state['last_time'] = t
        return los_meas, 0.0, True
    dt = t - t_prev
    if dt <= 0.0:
        return los_prev, 0.0, False
    ang_step = _angle_between_unit_deg(los_meas, los_prev)
    if jump_deg > 0.0 and ang_step >= jump_deg:
        # Real target move -> bypass the smoother and snap to the measurement.
        state['los_hat'] = los_meas
        state['last_time'] = t
        return los_meas, ang_step, True
    # EMA blend. tau<=0 -> no smoothing (alpha=1, gate only).
    if smoothing_sec <= 0.0:
        alpha = 1.0
    else:
        alpha = dt / (smoothing_sec + dt)
    los_hat = [alpha * los_meas[i] + (1.0 - alpha) * los_prev[i] for i in range(3)]
    los_hat = _normalize3(los_hat)
    state['los_hat'] = los_hat
    state['last_time'] = t
    return los_hat, ang_step, False


def _filter_wrapped_yaw_deg(state, yaw_meas_deg, t, tau_sec, reset=False):
    # 1st-order LPF on wrapped yaw angle. Filter update is on wrapped error so
    # crossing +/-180 deg is continuous. Mutates `state` in place.
    y_meas = _wrap_to_180(float(yaw_meas_deg))
    if reset:
        state['yaw_hat_deg'] = y_meas
        state['last_time'] = t
        return y_meas
    y_prev = state.get('yaw_hat_deg', None)
    t_prev = state.get('last_time', None)
    if y_prev is None or t_prev is None:
        state['yaw_hat_deg'] = y_meas
        state['last_time'] = t
        return y_meas
    dt = t - t_prev
    if dt <= 0.0:
        return _wrap_to_180(y_prev)
    if tau_sec <= 0.0:
        alpha = 1.0
    else:
        alpha = dt / (tau_sec + dt)
    y_hat = _wrap_to_180(y_prev + alpha * _wrap_to_180(y_meas - y_prev))
    state['yaw_hat_deg'] = y_hat
    state['last_time'] = t
    return y_hat


def _axis_velocity_update(state, desired_pos_deg, actual_pos_deg, min_deg, max_deg,
                          dt, first_cycle, wrap_pan, enabled, cmd_sign, eff_max_dps, p,
                          ref_rate_dps=0.0, dist_rate_dps=0.0):
    # Returns (cmd_dps, dbg); cmd_dps is signed in MOTOR direction (cmd_sign
    # applied). Mutates `state` in place.
    kp                = float(p.get('kp', 0.8))
    ki                = float(p.get('ki', 0.2))
    reference_ff_gain   = float(p.get('reference_ff_gain', 0.7))
    disturbance_ff_gain = float(p.get('disturbance_ff_gain', 0.0))
    ff_deadband_dps   = _AUTO2_FF_DEADBAND_DPS
    pos_deadband_deg  = float(p.get('pos_deadband_deg', _AUTO2_POS_DEADBAND_DEG))
    ff_cutoff_hz      = _AUTO2_FF_CUTOFF_HZ
    ff_lead_sec       = float(p.get('ff_lead_sec', 0.0))
    integral_max_dps  = _AUTO2_INTEGRAL_MAX_DPS
    softstop_margin   = _AUTO2_SOFTSTOP_MARGIN_DEG
    vel_min_dps       = float(p.get('vel_min_dps', 0.5))
    swell_ff_enabled  = bool(p.get('swell_predict_enabled', 0))
    swell_freq_min_hz = _AUTO2_SWELL_FREQ_MIN_HZ
    swell_freq_max_hz = _AUTO2_SWELL_FREQ_MAX_HZ
    swell_freq_seed_hz = _AUTO2_SWELL_FREQ_SEED_HZ
    swell_sogi_k      = float(p.get('swell_sogi_k', 1.0))
    swell_fll_gamma   = _AUTO2_SWELL_FLL_GAMMA
    swell_bias_hz     = _AUTO2_SWELL_BIAS_HZ
    swell_conf_hz     = _AUTO2_SWELL_CONF_HZ
    swell_amp_min_dps = float(p.get('swell_amp_min_dps', 1.0))
    swell_conf_min    = float(p.get('swell_conf_min', 0.5))
    cs = 1.0 if cmd_sign >= 0 else -1.0

    # Bounded-axis handling. Both pan and tilt are mechanically limited (a
    # reachable arc that spans < 360 deg with an unreachable gap behind it), so
    # the in-range path to the setpoint is the DIRECT difference, not the
    # shortest circular path. Clamp the setpoint into the reachable arc and only
    # wrap the error when the axis is genuinely continuous (>= 360 deg span) --
    # otherwise wrapping could route the servo through the unreachable back gap
    # (it would drive into a soft limit and stall instead of going the long way).
    if desired_pos_deg > max_deg:
        desired_pos_deg = max_deg
    elif desired_pos_deg < min_deg:
        desired_pos_deg = min_deg
    wrap_continuous = wrap_pan and ((max_deg - min_deg) >= 359.0)

    # Gated off: hold, clear servo memory (preserve last cmd so the caller can
    # force a single zero send). No windup carries across an enable toggle.
    if not enabled:
        last_cmd = state.get('last_cmd_vel_dps', 0.0)
        bs = _blank_axis_state()
        bs['last_cmd_vel_dps'] = last_cmd
        state.clear()
        state.update(bs)
        dbg = {'desired': desired_pos_deg, 'actual': actual_pos_deg, 'err': 0.0,
               'cmd': 0.0, 'p_term': 0.0, 'i_term': 0.0, 'ff': 0.0, 'integral': 0.0,
               'rate_filt': 0.0, 'sw_freq_hz': 0.0, 'sw_conf': 0.0,
               'ref_ff': 0.0, 'dist_ff': 0.0,
               'softstop': 0, 'vel_min': 0}
        return 0.0, dbg

    integral     = state.get('integral', 0.0)
    rate_filt    = state.get('dist_rate_filt', 0.0)
    sw_omega     = state.get('sw_omega', None)
    sw_valpha    = state.get('sw_valpha', 0.0)
    sw_vbeta     = state.get('sw_vbeta', 0.0)
    sw_bias      = state.get('sw_bias', None)
    sw_e2        = state.get('sw_e2', 0.0)
    sw_u2        = state.get('sw_u2', 0.0)
    if first_cycle:
        integral = 0.0
        rate_filt = 0.0
        sw_omega = None
        sw_valpha = 0.0
        sw_vbeta = 0.0
        sw_bias = None
        sw_e2 = 0.0
        sw_u2 = 0.0

    two_pi = 6.283185307179586

    # Disturbance-rate feedforward source: the gyro-projected body-rate for THIS
    # axis (passed in as dist_rate_dps), low-passed. Replaces the old numerical
    # d/dt(desired_pos) -- the boat-motion rate is now measured, not differentiated,
    # so it carries no setpoint-step noise and reacts with zero lag.
    dist_rate_in = dist_rate_dps
    if dt > 0.0 and ff_cutoff_hz > 0.0:
        tau = 1.0 / (two_pi * ff_cutoff_hz)
        alpha = dt / (dt + tau)
    else:
        alpha = 1.0
    rate_filt = rate_filt + alpha * (dist_rate_in - rate_filt)

    # Swell predictor (SOGI-FLL) on the GYRO disturbance-rate signal: DC-bias
    # removal isolates the swell tone, the FLL self-tracks its frequency, and the
    # quadrature pair reconstructs the rate phase-advanced by ff_lead_sec to offset
    # actuator lag. Confidence-gated so lost lock degrades to the raw filtered rate.
    # Now fed by the gyro (not the differentiated setpoint), so it only ever sees
    # real boat motion. Disabled (swell_predict_enabled absent/0) -> skipped; the
    # raw filtered rate is used. Amplitude and reset thresholds are in rate units (dps).
    sw_freq_hz = 0.0
    sw_amp_dps = 0.0
    sw_conf = 0.0
    rate_pred = rate_filt
    if swell_ff_enabled and (not first_cycle) and dt > 0.0:
        w_min = two_pi * swell_freq_min_hz
        w_max = two_pi * swell_freq_max_hz
        if (sw_omega is None) or (sw_omega <= 0.0):
            sw_omega = two_pi * swell_freq_seed_hz
        if sw_bias is None:
            sw_bias = dist_rate_in

        tau_bias = 1.0 / (two_pi * swell_bias_hz) if swell_bias_hz > 0.0 else 0.0
        a_bias = dt / (dt + tau_bias) if tau_bias > 0.0 else 1.0
        sw_bias = sw_bias + a_bias * (dist_rate_in - sw_bias)
        u = dist_rate_in - sw_bias
        e = u - sw_valpha
        dva = sw_omega * (swell_sogi_k * e - sw_vbeta)
        dvb = sw_omega * sw_valpha
        sw_valpha = sw_valpha + dt * dva
        sw_vbeta = sw_vbeta + dt * dvb
        denom = sw_valpha * sw_valpha + sw_vbeta * sw_vbeta + 1e-6
        sw_omega = sw_omega - dt * swell_fll_gamma * sw_omega * (e * sw_vbeta) / denom
        if sw_omega < w_min:
            sw_omega = w_min
        elif sw_omega > w_max:
            sw_omega = w_max
        tau_c = 1.0 / (two_pi * swell_conf_hz) if swell_conf_hz > 0.0 else 0.0
        a_c = dt / (dt + tau_c) if tau_c > 0.0 else 1.0
        sw_e2 = sw_e2 + a_c * (e * e - sw_e2)
        sw_u2 = sw_u2 + a_c * (u * u - sw_u2)
        sw_amp_dps = denom ** 0.5
        sw_freq_hz = sw_omega / two_pi
        conf = 1.0 - (sw_e2 / (sw_u2 + 1e-6))
        if conf < 0.0:
            conf = 0.0
        elif conf > 1.0:
            conf = 1.0
        if sw_amp_dps < swell_amp_min_dps:
            conf = 0.0
        sw_conf = conf
        # Phase-advance the reconstructed RATE by ff_lead_sec (actuator-lag
        # compensation). ff_lead_sec=0 gives denoise-only (no prediction).
        wt = sw_omega * ff_lead_sec
        rate_pred = sw_valpha * math.cos(wt) - sw_vbeta * math.sin(wt)

    # Disturbance FF: predicted (confidence-blended) gyro rate when the predictor
    # is enabled+locked, else the raw filtered gyro rate. Soft deadband kills
    # sensor noise, then linear (no breakaway).
    dist_rate_ff = rate_filt
    if swell_ff_enabled:
        ff_blend = sw_conf if sw_conf >= swell_conf_min else 0.0
        dist_rate_ff = ff_blend * rate_pred + (1.0 - ff_blend) * rate_filt
    if dist_rate_ff > ff_deadband_dps:
        dist_db = dist_rate_ff - ff_deadband_dps
    elif dist_rate_ff < -ff_deadband_dps:
        dist_db = dist_rate_ff + ff_deadband_dps
    else:
        dist_db = 0.0
    dist_ff = disturbance_ff_gain * dist_db

    # Reference FF: the intended (commanded) axis rate for the active mode -- the
    # scan sweep rate on pan, zero for stab/lock/hold. Analytic, so it carries no
    # differentiation noise and naturally ignores detection "snaps" (those are
    # setpoint steps the P term owns, not reference velocity). No predictor.
    ref_ff = reference_ff_gain * ref_rate_dps

    ff_term = ref_ff + dist_ff

    pos_err = desired_pos_deg - actual_pos_deg
    if wrap_continuous:
        pos_err = _wrap_to_180(pos_err)

    # P with soft deadband.
    if pos_err > pos_deadband_deg:
        err_db = pos_err - pos_deadband_deg
    elif pos_err < -pos_deadband_deg:
        err_db = pos_err + pos_deadband_deg
    else:
        err_db = 0.0
    p_term = kp * err_db

    # I: accumulate only outside the deadband; clamp (anti-windup).
    integral_prev = integral
    if first_cycle or err_db == 0.0:
        integral_cand = 0.0
    else:
        integral_cand = integral_prev + (pos_err * dt)
        if ki > 0.0:
            i_limit = integral_max_dps / ki
            if integral_cand > i_limit:
                integral_cand = i_limit
            elif integral_cand < -i_limit:
                integral_cand = -i_limit

    # P+I with stiction breakaway floor; the deadband owns the stop.
    vel_min_active = 0
    u_pos = p_term + ki * integral_cand
    if err_db == 0.0:
        u_pos = 0.0
    elif abs(u_pos) < vel_min_dps:
        u_pos = vel_min_dps if u_pos > 0.0 else -vel_min_dps
        vel_min_active = 1
        integral_cand = integral_prev

    # FF passes linearly (own deadband); sum + saturate.
    u_unsat = ff_term + u_pos
    if u_unsat > eff_max_dps:
        u_sat = eff_max_dps
    elif u_unsat < -eff_max_dps:
        u_sat = -eff_max_dps
    else:
        u_sat = u_unsat
    if (u_sat != u_unsat) and ((u_unsat > 0.0) == (pos_err > 0.0)) and not first_cycle and err_db != 0.0:
        integral = integral_prev
    else:
        integral = integral_cand
    i_term = ki * integral

    # Soft-stop guard in AXIS space (before motor-sign mapping).
    softstop_active = 0
    if (actual_pos_deg >= (max_deg - softstop_margin)) and (u_sat > 0.0):
        u_sat = 0.0
        softstop_active = 1
    elif (actual_pos_deg <= (min_deg + softstop_margin)) and (u_sat < 0.0):
        u_sat = 0.0
        softstop_active = 1

    cmd_dps = cs * u_sat
    if cmd_dps > eff_max_dps:
        cmd_dps = eff_max_dps
    elif cmd_dps < -eff_max_dps:
        cmd_dps = -eff_max_dps
    cmd_dps = round(cmd_dps, 2)

    state['integral'] = integral
    state['dist_rate_filt'] = rate_filt
    state['sw_omega'] = sw_omega
    state['sw_valpha'] = sw_valpha
    state['sw_vbeta'] = sw_vbeta
    state['sw_bias'] = sw_bias
    state['sw_e2'] = sw_e2
    state['sw_u2'] = sw_u2

    dbg = {'desired': desired_pos_deg, 'actual': actual_pos_deg, 'err': pos_err,
           'cmd': cmd_dps, 'p_term': p_term, 'i_term': i_term, 'ff': ff_term,
           'integral': integral, 'rate_filt': rate_filt, 'sw_freq_hz': sw_freq_hz,
           'sw_conf': sw_conf, 'ref_ff': ref_ff, 'dist_ff': dist_ff,
           'softstop': softstop_active, 'vel_min': vel_min_active}
    return cmd_dps, dbg



def is_night(lat, lon, utc_datetime):
    # Ensure UTC datetime is timezone aware
    if utc_datetime.tzinfo is None:
        utc_datetime = utc_datetime.replace(tzinfo=timezone.utc)
        
    observer = Observer(latitude=lat, longitude=lon)
    target_date = utc_datetime.date()
    
    try:
        # 1. Gather sunrise/sunset pairs across 3 days to catch all boundary crossings
        days = [target_date - timedelta(days=1), target_date, target_date + timedelta(days=1)]
        sun_events = []
        
        for d in days:
            s = sun(observer, date=d)
            sun_events.append(s)
            
        # 2. Extract and strictly sort all sunrises and sunsets chronologically
        sunrises = sorted([s['sunrise'] for s in sun_events])
        sunsets = sorted([s['sunset'] for s in sun_events])
        
        # 3. Find the closest sunrise that happened BEFORE or EXACTLY at our target time
        past_sunrises = [r for r in sunrises if r <= utc_datetime]
        if not past_sunrises:
            return True  # If no sunrise happened yet in this 3-day block, it's night
            
        latest_sunrise = past_sunrises[-1]
        
        # 4. Find the absolute next sunset that follows that specific sunrise
        future_sunsets = [s for s in sunsets if s > latest_sunrise]
        if not future_sunsets:
            return True
            
        corresponding_sunset = future_sunsets[0]
        
        # 5. It is DAY only if our target time sits between that sunrise and its matching sunset
        if latest_sunrise <= utc_datetime <= corresponding_sunset:
            return False  # It is Day
        else:
            return True   # It is Night
            
    except ValueError:
        # Handles edge cases in polar regions where the sun never sets/rises
        return False



def pt_auto_2(pt_connect_if, 
                        auto_data_dict, 
                        auto_data_dict_last,
                        auto_settings_dict, 
                        ):
    # Unified vector-lock controller (ArgusVectorController). Builds a desired
    # boat_level look vector, transforms it into pt_base via live roll/pitch,
    # solves desired pan/tilt, and drives BOTH axes with the generic velocity
    # servo (_axis_velocity_update) in MMV velocity mode.
    #
    # MODE MODEL: the PAN enables in auto_data_dict (set by the app node) select
    # how the desired boat_level vector is built; TILT always rides that vector's
    # elevation (the TILT enables do nothing). Priority each cycle:
    #   STAB (pan_stab_enabled): point-and-hold the seeded/clicked target vector.
    #        Forces SCAN and TRACK off (written back so the app node absorbs it).
    #   LOCK (pan_track_enabled AND a fresh valid detection): v2 target LOS.
    #   SCAN (pan_scan_enabled): constant-speed pan bounce between the soft limits.
    #   HOLD (none active): zero velocity, both axes stop.
    # Both axes command together: pan_on = tilt_on = (mode != HOLD) AND not watchdog.
    #
    # DRIVER CONTRACT:
    #   pt_connect_if.jog_timed_speed_dps_pan/tilt(direction, speed_dps, duration_s=-1)
    #     direction +1/-1; speed_dps UNSIGNED (driver clamps to speed_max_dps);
    #     duration -1 = run until next command; speed 0 = stop.
    start_time = nepi_utils.get_time()
    # Monotonic clock for the control-loop period ONLY (loop_hz/dt). Immune to NTP
    # steps that corrupt wall-clock deltas. Everything else here stays wall-clock.
    mono_now = time.monotonic()

    last_time = copy.deepcopy(auto_data_dict['last_auto_time'])
    delta_time = nepi_utils.get_time() - last_time

    #############################
    # Update PT Data
    #############################
    pan_deg = auto_data_dict['pan_deg']
    tilt_deg = auto_data_dict['tilt_deg'] 

    #############################
    # Update Nav Data
    #############################
    navpose_topic = auto_data_dict['navpose_topic']
    navpose_source_name = os.path.basename(navpose_topic)
    navpose_dict = auto_data_dict['navpose_data']
    #logger.log_warn("pt_auto_2 navpose_dict: " + str(navpose_dict))

    navpose_config_dict = auto_data_dict['navpose_config']
    #logger.log_warn("pt_auto_2 navpose_config_dict: " + str(navpose_config_dict))

    #logger.log_warn("pt_auto_2 got navpose_config: " + str(navpose_config_dict))
    # --- Mount transforms (M_imu, M_pt) from the RUI NavPose config -------------
    # Index by COMPONENT name and take the transform from the first slot that
    # carries a real source topic (priority update > init > offset > reset), so the
    # live "both on Init" config and the example "both on Update" config both
    # resolve. 'orientation' -> M_imu (IMU sensor -> body); 'pan_tilt' -> M_pt
    # (PT base -> body). The operator types the mount Euler into the RUI NavPose
    # Manager with Apply Transforms OFF; we read the numbers here and apply the
    # rotation ourselves (correct ZYX composition, not the manager's planar add).
    navpose_info_dict = None
    if navpose_config_dict is not None:
        navpose_info_dict = dict()
        comps = navpose_config_dict.get('components_list', []) or []
        infos = navpose_config_dict.get('components_info', []) or []
        for i, comp in enumerate(comps):
            if comp in ('None', '', None) or i >= len(infos):
                continue
            info = infos[i]
            for slot in ('update', 'init', 'offset', 'reset'):
                topic = info.get(slot + '_topic', '')
                if topic not in ('', 'None', 'Fixed', None):
                    navpose_info_dict[comp] = {
                        'type': slot,
                        'topic': topic,
                        'transform': info.get(slot + '_topic_transform', None),
                    }
                    break
    #logger.log_warn("pt_auto_2 navpose_info_dict: " + str(navpose_info_dict), throttle_s = 10)


    #################
    # Update Day Night
    #################
    time_sec_system = nepi_utils.get_time()
    datatime_utc = datetime.now(pytz.utc)
    # local_timezone = 'CST6CDT'
    # datatime_local = nepi_utils.convert_time_to_datetime(time_sec_system, timezone = local_timezone)
    auto_is_night = auto_data_dict['auto_is_night']
    auto_was_night = copy.deepcopy(auto_is_night)
    auto_data_dict['auto_was_night'] = auto_was_night
    if navpose_dict is not None:
        if supports_day_night == True and navpose_dict['has_location'] == True:
            lat = navpose_dict['latitude']
            long = navpose_dict['longitude']
            #logger.log_warn("Will Try Is Night Calc with: " + str([datatime_utc, lat, long]))
            try:
                if lat != -999 and long != -999:
                    auto_data_dict['auto_lat'] = lat
                    auto_data_dict['auto_long'] = long
                    auto_is_night = is_night(lat, long, datatime_utc)
                    #logger.log_warn("Is Night: " + str(auto_is_night), throttle_s = 10)
            except Exception as e:
                logger.log_warn("Is Night calc failed with: " + str(e), throttle_s = 10)
            
    auto_data_dict['auto_is_night'] = auto_is_night
    if auto_is_night != auto_was_night:
        auto_data_dict['auto_night_updated'] = True
        logger.log_warn("Is Night Updated with: " + str(auto_is_night))
    else:
        auto_data_dict['auto_night_updated'] = False


    #################################################################
    def _resolve_mount(comp_name, cache_key):
        # Build a validated rotation from the component's mount transform; fall back
        # to the last good value, then identity, so a momentary missing/bad config
        # never zeroes the leveling chain.
        R = None
        if navpose_info_dict is not None and comp_name in navpose_info_dict:
            R = transform_dict_to_R(navpose_info_dict[comp_name].get('transform'))
            if not validate_rotation(R):
                logger.log_warn("pt_auto_2 invalid mount for " + comp_name
                                + "; holding last good", throttle_s = 10)
                R = None
        if R is None:
            R = auto_data_dict.get(cache_key, None)
        if R is None:
            R = _I3.copy()
        auto_data_dict[cache_key] = R
        return np.asarray(R, dtype=float)

    M_imu = _resolve_mount('orientation', 'auto2_M_imu')
    M_pt  = _resolve_mount('pan_tilt',    'auto2_M_pt')
    # Heading component of the PT mount rotation.  Offsets the slider ruler so
    # that slider-center = pan 0 (PT home) regardless of how the PT is mounted.
    M_pt_yaw_deg = math.degrees(math.atan2(float(M_pt[1, 0]), float(M_pt[0, 0])))


 

    roll_deg = -999
    pitch_deg = -999
    yaw_deg = -999
    heading_deg = -999



    if navpose_dict is not None:
        timestamp = nepi_utils.get_time()

        auto_data_dict['pitch_deg'] = pitch_deg


        if 'time_orientation' in navpose_dict.keys():
            orient_timestamp = navpose_dict['time_orientation']
        elif 'timestamp' in navpose_dict.keys():
            orient_timestamp = navpose_dict['timestamp']
        else:
            orient_timestamp = timestamp
        auto_data_dict['roll_timestamp'] = orient_timestamp
        auto_data_dict['pitch_timestamp'] = orient_timestamp
        auto_data_dict['yaw_timestamp'] = orient_timestamp


        [roll_deg,pitch_deg,yaw_deg] = [navpose_dict['roll_deg'],navpose_dict['pitch_deg'],navpose_dict['yaw_deg']]
        auto_data_dict['roll_deg'] = roll_deg
        auto_data_dict['pitch_deg'] = pitch_deg
        auto_data_dict['yaw_deg'] = yaw_deg


        # Angular rates: direct MicroStrain gyro via NavPose *_deg_per_sec.
        # Sign: +roll, +pitch, -yaw (empirically verified on rig 2026-07-22).
        auto_data_dict['roll_dps']  = navpose_dict.get('roll_deg_per_sec', 0.0)
        auto_data_dict['pitch_dps'] = navpose_dict.get('pitch_deg_per_sec', 0.0)
        auto_data_dict['yaw_dps']   = -navpose_dict.get('yaw_deg_per_sec', 0.0)

        heading_timestamp = 0
        heading_deg = 0.0
        heading_dps = 0.0
        if 'heading_deg' in navpose_dict.keys():

            if navpose_dict['heading_deg'] != -999:
                stab_pan_ready = True
                if 'time_heading' in navpose_dict.keys():
                    heading_timestamp = navpose_dict['time_heading']
                elif 'timestamp' in navpose_dict.keys():
                    heading_timestamp = navpose_dict['timestamp']
                else:
                        heading_timestamp = timestamp
                heading_deg = navpose_dict['heading_deg']
                heading_dps = (auto_data_dict['heading_deg'] - auto_data_dict_last['heading_deg']) / delta_time
        
        auto_data_dict['heading_timestamp'] = heading_timestamp
        auto_data_dict['heading_deg'] = heading_deg
        auto_data_dict['heading_dps'] = heading_dps

    ##########################
    # Calculate pan tilt Stab adjustments
    ##########################
    ## Transpose Source Frame Nav to Pan Tilt Frame
    rpy_vector = [roll_deg, -1 * pitch_deg, heading_deg ]
    if -999 not in rpy_vector:
        [ar,ap,ay] = rpy_vector

        [art,apt,ayt]  = nepi_nav.rotate_enu_angles([ar,ap,ay],tilt_deg,'y')
        [ar,ap,ay] = [art,apt,ayt]

        [arp,app,ayp]  = nepi_nav.rotate_enu_angles([ar,ap,ay],pan_deg,'z')
        [ar,ap,ay] = [arp,app,ayp]


        
        ## Calculate Pan Adjustment
        p_adj = 0 #####
        auto_data_dict['auto_pan_adj'] = p_adj



        ## Calculate Tilt Adjustment
        t_adj = ap
        auto_data_dict['auto_tilt_adj'] = t_adj






    #################
    # Find Best Target
    #################
    tracking_dict = auto_data_dict['tracking_dict']
    auto_data_dict['last_target_dict'] = copy.deepcopy(auto_data_dict['target_dict'])
    tracking_dict = auto_data_dict.get('tracking_dict', copy.deepcopy(nepi_track.BLANK_SETTINGS_DICT))
    targets_timestamp = auto_data_dict['target_timestamp']
    targets_list = auto_data_dict['targets_list']
    best_target_dict = None
    if targets_list is not None:
        if len(targets_list) > 0:
            [best_target_dict,tracking_dict] = nepi_track.get_best_from_targets(targets_list,tracking_dict)
    auto_data_dict['target_dict'] = best_target_dict
    if best_target_dict is not None:
        auto_data_dict['target_timestamp'] = best_target_dict['timestamp']
    else:
        auto_data_dict['target_timestamp'] = 0



    ##########################
    # Gather settings
    ##########################
    controls_dict = auto_settings_dict['auto_controls_dict']
    scan_speed_dps    = float(controls_dict.get('scan_speed_dps', 5.0))
    scan_el_deg       = float(controls_dict.get('scan_el_deg', 0.0))
    yaw_stab_enable   = bool(controls_dict.get('yaw_stab_enable', 0))
    yaw_lpf_tau_sec   = max(0.0, float(controls_dict.get('yaw_lpf_tau_sec', 0.25)))
    yaw_rate_deadband_dps = max(0.0, float(controls_dict.get('yaw_rate_deadband_dps', 0.5)))
    track_lost_sec    = float(controls_dict.get('track_lost_sec', 1.5))
    track_jitter_enabled        = bool(controls_dict.get('track_jitter_filter_enabled', 1))
    track_smoothing_sec         = float(controls_dict.get('track_smoothing_sec', 0.3))
    track_jump_deg              = float(controls_dict.get('track_jump_deg', 5.0))
    # Hardware sign conventions + camera-offset signs are LOCKED in code (see
    # _AUTO2_* constants); validated on the rig and not overridable via settings.
    pan_cmd_sign      = _AUTO2_PAN_CMD_SIGN
    tilt_cmd_sign     = _AUTO2_TILT_CMD_SIGN
    tilt_axis_sign    = _AUTO2_TILT_AXIS_SIGN
    track_az_sign     = _AUTO2_TRACK_AZ_SIGN
    track_el_sign     = _AUTO2_TRACK_EL_SIGN
    cmd_change_dps    = _AUTO2_CMD_CHANGE_DPS
    watchdog_sec      = _AUTO2_WATCHDOG_SEC
    telem_enabled     = bool(controls_dict.get('telem_enabled', 1))
    telem_ip          = _auto2_TELEM_IP   # code-set target IP, not a control

    # Per-axis PID+FF gains, fully separated (PAN_* and TILT_* only -- no shared
    # fallback, so the two axes can never alias each other's values). FF is split
    # into reference_ff_gain (analytic commanded-rate, e.g. the scan sweep) and
    # disturbance_ff_gain (gyro-projected boat-motion rate, default 0.0 until
    # bench-tuned). Old saved configs without PAN_/TILT_ keys take these defaults.
    pan_gains = {
        'kp':           float(controls_dict.get('PAN_kp', 1.0)),
        'ki':           float(controls_dict.get('PAN_ki', 0.0)),
        'reference_ff_gain':   float(controls_dict.get('PAN_reference_ff_gain', 0.7)),
        'disturbance_ff_gain': float(controls_dict.get('PAN_disturbance_ff_gain', 0.0)),
        'pos_deadband_deg':    float(controls_dict.get('PAN_pos_deadband_deg', 0.5)),
        'max_vel_dps':  float(controls_dict.get('PAN_max_vel_dps', 40.0)),
        'vel_min_dps':  float(controls_dict.get('PAN_vel_min_dps', 0.5)),
    }
    tilt_gains = {
        'kp':           float(controls_dict.get('TILT_kp', 1.0)),
        'ki':           float(controls_dict.get('TILT_ki', 0.0)),
        'reference_ff_gain':   float(controls_dict.get('TILT_reference_ff_gain', 0.7)),
        'disturbance_ff_gain': float(controls_dict.get('TILT_disturbance_ff_gain', 0.0)),
        'pos_deadband_deg':    float(controls_dict.get('TILT_pos_deadband_deg', 0.5)),
        'ff_lead_sec':  float(controls_dict.get('TILT_ff_lead_sec', 0.2)),
        'max_vel_dps':  float(controls_dict.get('TILT_max_vel_dps', 40.0)),
        'vel_min_dps':  float(controls_dict.get('TILT_vel_min_dps', 0.5)),
        'swell_predict_enabled': int(controls_dict.get('TILT_swell_predict_enabled', 0)),
        'swell_sogi_k':          float(controls_dict.get('TILT_swell_sogi_k', 1.0)),
        'swell_amp_min_dps':     float(controls_dict.get('TILT_swell_amp_min_dps', 1.0)),
        'swell_conf_min':        float(controls_dict.get('TILT_swell_conf_min', 0.5)),
    }

    ##########################
    # Gather signals
    ##########################
    pan_deg      = auto_data_dict.get('pan_deg', 0.0)
    tilt_deg     = auto_data_dict.get('tilt_deg', 0.0)
    pan_min_deg  = auto_data_dict.get('pan_min_deg', -170)
    pan_max_deg  = auto_data_dict.get('pan_max_deg', 170)
    tilt_min_deg = auto_data_dict.get('tilt_min_deg', -50)
    tilt_max_deg = auto_data_dict.get('tilt_max_deg', 50)
    pan_tilt_max_speed_dps = auto_data_dict.get('pan_tilt_max_speed_dps', 0.0)
    roll_deg     = auto_data_dict.get('roll_deg', -999)
    pitch_deg    = auto_data_dict.get('pitch_deg', -999)
    yaw_deg      = auto_data_dict.get('yaw_deg', -999)

    # PAN-axis mode enables (the TILT enables are intentionally ignored; TILT
    # rides whatever vector the active PAN mode produces).
    pan_stab_enabled  = bool(auto_data_dict.get('pan_stab_enabled', False))
    pan_scan_enabled  = bool(auto_data_dict.get('pan_scan_enabled', False))
    pan_track_enabled = bool(auto_data_dict.get('pan_track_enabled', False))

    # --- INPUT snapshot ---
    _in_pan_ratio_update  = auto_data_dict.get('auto_pan_ratio_update', None)
    _in_tilt_ratio_update = auto_data_dict.get('auto_tilt_ratio_update', None)
    _in_click_update      = auto_data_dict.get('auto2_click_update', None)
    _in_data_time         = auto_data_dict.get('data_time', 0.0)
    _in_target_dict       = auto_data_dict.get('target_dict', None)
    _io_log_interval = float(AUTO2_LOG_THROTTLE_SEC)
    _io_log_enabled = (_io_log_interval >= 0.0)
    _io_last_log_time = auto_data_dict.get('auto2_last_io_log_time', None)
    _do_io_log = (_io_log_enabled and
                  (_io_log_interval == 0.0
                   or _io_last_log_time is None
                   or (start_time - float(_io_last_log_time)) >= _io_log_interval))
    if _do_io_log:
        auto_data_dict['auto2_last_io_log_time'] = start_time
        logger.log_warn("nepi_auto_pt:pt_auto_2: INPUT pan=%.2f tilt=%.2f roll=%.1f pitch=%.1f yaw=%.1f stab=%s scan=%s track=%s data_time=%.3f pan_ratio_upd=%s tilt_ratio_upd=%s click_upd=%s target=%s" %
                        (pan_deg, tilt_deg, roll_deg, pitch_deg, yaw_deg,
                         pan_stab_enabled, pan_scan_enabled, pan_track_enabled,
                         _in_data_time,
                         _in_pan_ratio_update, _in_tilt_ratio_update,
                         _in_click_update is not None,
                         _in_target_dict is not None))

    eff_max_pan_dps = pan_gains['max_vel_dps']
    eff_max_tilt_dps = tilt_gains['max_vel_dps']
    if pan_tilt_max_speed_dps and pan_tilt_max_speed_dps > 0:
        eff_max_pan_dps = min(eff_max_pan_dps, float(pan_tilt_max_speed_dps))
        eff_max_tilt_dps = min(eff_max_tilt_dps, float(pan_tilt_max_speed_dps))

    ##########################
    # Persistent per-axis state (nested dicts; self-initializing)
    ##########################
    pan_state  = auto_data_dict.get('auto2_pan_state', None)
    tilt_state = auto_data_dict.get('auto2_tilt_state', None)
    if not isinstance(pan_state, dict) or len(pan_state) == 0:
        pan_state = _blank_axis_state()
    if not isinstance(tilt_state, dict) or len(tilt_state) == 0:
        tilt_state = _blank_axis_state()
    scan_state  = auto_data_dict.get('auto2_scan_state', None)
    track_state = auto_data_dict.get('auto2_track_state', None)
    if not isinstance(scan_state, dict) or len(scan_state) == 0:
        scan_state = _blank_scan_state()
    if not isinstance(track_state, dict) or len(track_state) == 0:
        track_state = _blank_track_state()
    stab_state = auto_data_dict.get('auto2_stab_state', None)
    if not isinstance(stab_state, dict) or len(stab_state) == 0:
        stab_state = _blank_stab_state()
    yaw_lpf_state = auto_data_dict.get('auto2_yaw_lpf_state', None)
    if not isinstance(yaw_lpf_state, dict) or len(yaw_lpf_state) == 0:
        yaw_lpf_state = _blank_yaw_lpf_state()
    last_update_mono = auto_data_dict.get('auto2_last_update_mono', None)
    last_data_time = auto_data_dict.get('auto2_last_data_time', None)
    data_change_t  = auto_data_dict.get('auto2_data_change_time', None)

    # Rate-agnostic dt; first call / long gap -> fresh start. MONOTONIC source.
    dt = 0.0 if last_update_mono is None else (mono_now - last_update_mono)
    first_cycle = (dt <= 0.0) or (dt > AUTO2_RESET_TIMEOUT_SEC)
    loop_hz = (1.0 / dt) if dt > 0.0 else 0.0

    ##########################
    # Frame matrices for this cycle (explicit mount chain)
    ##########################
    # Build the live boat attitude from the RAW IMU orientation and the two RUI
    # mounts, then split out the swing (roll/pitch leveling) and twist (heading):
    #   R_world_imu  = Rz(yaw)Ry(pitch)Rx(roll)      (raw AHRS, intrinsic ZYX)
    #   R_world_body = R_world_imu @ M_imu.T         (strip the IMU mount)
    #   twist/swing  = swing_twist_up(R_world_body)  (about world up = gravity)
    # The look vector stays in boat_level (gravity-leveled, boat-heading azimuth) so
    # az/el, the STAB hold, the SCAN sweep and the sliders all work exactly as
    # before. The only attitude-dependent step is boat_level <-> pt_base, now the
    # explicit M_pt.T @ R_swing.T (inverse R_swing @ M_pt). Leveling uses the swing
    # only, so it is immune to IMU yaw error (proven on the rig) -- a missing/invalid
    # yaw still levels correctly (yaw 0 fallback); yaw-stab just won't engage.
    imu_valid = (roll_deg != -999) and (pitch_deg != -999)
    yaw_valid = imu_valid and (yaw_deg != -999)
    yaw_for_twist_deg = yaw_deg
    yaw_filter_on = bool(yaw_stab_enable) and yaw_valid
    if yaw_filter_on:
        yaw_for_twist_deg = _filter_wrapped_yaw_deg(
            yaw_lpf_state, yaw_deg, start_time, yaw_lpf_tau_sec, reset=first_cycle)
    else:
        yaw_lpf_state = _blank_yaw_lpf_state()
    if imu_valid:
        yaw_for_R = yaw_for_twist_deg if yaw_valid else 0.0
        R_world_body = R_world_imu_from_ahrs(roll_deg, pitch_deg, yaw_for_R) @ M_imu.T
        _R_twist, R_swing, twist_deg = swing_twist_up(R_world_body)
    else:
        R_swing = _I3
        twist_deg = 0.0
    yaw_on = bool(yaw_stab_enable) and yaw_valid
    _lvl2pt = M_pt.T @ R_swing.T          # boat_level -> pt_base (current swing)

    # Digital roll for image stabilization: the PT has no roll axis, so a platform
    # roll rotates the camera image about its boresight. Compute that image roll
    # (deg) from the CURRENT swing + live pan/tilt for telemetry/bench verification.
    # (The image-time-accurate value the downstream image-rotator should consume is
    # image_roll_deg_at(...) evaluated at the frame timestamp -- see that helper.)
    if imu_valid:
        image_roll_deg = image_roll_deg_from_swing(
            R_swing, M_pt, pan_deg, tilt_axis_sign * tilt_deg)
    else:
        image_roll_deg = 0.0
    auto_data_dict['auto2_image_roll_deg'] = image_roll_deg

    def level_to_pt_base(v_level):
        return apply_R(_lvl2pt, v_level)

    def pt_base_to_level(v_pt, R_sw):
        return apply_R(np.asarray(R_sw) @ M_pt, v_pt)

    def swing_at(roll_d, pitch_d):
        # Swing (roll/pitch) at a past instant for image-time detection anchoring.
        # Swing is yaw-independent, so a 0 heading is exact here.
        if (roll_d == -999) or (pitch_d == -999):
            return _I3
        _t, R_sw, _d = swing_twist_up(R_world_imu_from_ahrs(roll_d, pitch_d, 0.0) @ M_imu.T)
        return R_sw

    def anchor_offset_to_level(pan_d, tilt_d, az_off, el_off, R_sw, valid):
        # Camera-frame angular offset (from the current boresight) -> boat_level unit
        # vector, via the PT-base ray and the swing. Mirrors the TRACK ingest so a
        # STAB seed / click lands like a detection. IMU invalid -> leave in pt_base.
        tgt_pan_geom  = pan_d + track_az_sign * az_off
        tgt_tilt_geom = (tilt_axis_sign * tilt_d) + track_el_sign * el_off
        ray_pt = vector_from_az_el(tgt_pan_geom, tgt_tilt_geom)
        if valid:
            return _normalize3(pt_base_to_level(ray_pt, R_sw))
        return _normalize3(ray_pt)

    # Yaw stabilize: hold the persistent boat_level vectors fixed in the WORLD by
    # removing the boat heading increment each cycle (rotate about up by -d_twist).
    # OFF -> vectors stay boat-relative (today's behavior). SCAN always sweeps in
    # boat_level either way. Caveat: relies on IMU yaw, so a held world bearing can
    # slowly drift with the yaw estimate.
    twist_prev = auto_data_dict.get('auto2_twist_prev', None)
    twist_rate_dps = 0.0
    if yaw_on:
        if twist_prev is not None:
            d_twist_deg = _wrap_to_180(twist_deg - twist_prev)
            d_twist = math.radians(d_twist_deg)
            if stab_state.get('los') is not None:
                stab_state['los'] = _normalize3(_rot_z(stab_state['los'], -d_twist))
            if track_state.get('los') is not None:
                track_state['los'] = _normalize3(_rot_z(track_state['los'], -d_twist))
            if dt > 0.0:
                twist_rate_dps = d_twist_deg / dt
        auto_data_dict['auto2_twist_prev'] = twist_deg
    else:
        auto_data_dict['auto2_twist_prev'] = None

    ##########################
    # 1. Resolve operating mode + desired boat_level look vector
    ##########################
    # Precedence LOCK > STAB > SCAN > HOLD. STAB and SCAN are mutually exclusive
    # base modes, so STAB forces SCAN off (write the override back so the app node
    # absorbs it). AI Tracking is an INDEPENDENT OVERLAY: a live detection lock
    # seizes the vector over EITHER base mode (STAB or SCAN) and reverts to it on
    # timeout, so pan_track_enabled is intentionally NOT force-disabled here --
    # otherwise the TRACK ingest below is skipped and STAB can never lock.
    if pan_stab_enabled:
        pan_scan_enabled = False
        auto_data_dict['pan_scan_enabled'] = False

    # TRACK: ingest a fresh detection (v2) and age the lock window. The app node
    # sets target_dict non-None only on cycles with a new detection, and stamps
    # state_at_img = platform (roll/pitch/pan/tilt) reconstructed at the image
    # time. A detection whose state is invalid is REJECTED (no v1 fallback) -- the
    # lock simply ages out if no valid detection lands within track_lost_sec.
    los_step_deg = 0.0
    los_reseed = 0
    if pan_track_enabled:
        target_dict = auto_data_dict.get('target_dict', None)
        if isinstance(target_dict, dict):
            state = target_dict.get('state_at_img', None)
            az_cam = float(target_dict.get('azimuth_deg', -999))
            el_cam = float(target_dict.get('elevation_deg', -999))
            if (isinstance(state, dict) and state.get('valid', False)
                    and az_cam != -999 and el_cam != -999):
                pan_img   = float(state.get('pan_deg', 0.0))
                tilt_img  = float(state.get('tilt_deg', 0.0))
                roll_img  = float(state.get('roll_deg', 0.0))
                pitch_img = float(state.get('pitch_deg', 0.0))
                # Target ray in pt_base (geometry, +up). The camera bore-sights the
                # PT pointing direction at image time; the detection is an angular
                # offset from it. tilt feedback is hardware convention, so map it to
                # geometry (tilt_axis_sign) before adding the +up elevation offset.
                tgt_pan_geom  = pan_img + track_az_sign * az_cam
                tgt_tilt_geom = (tilt_axis_sign * tilt_img) + track_el_sign * el_cam
                ray_pt = vector_from_az_el(tgt_pan_geom, tgt_tilt_geom)
                # Anchor into the stable boat_level frame using the swing (roll/pitch)
                # at image time; swing is yaw-independent so a 0 heading is exact.
                los_raw = pt_base_to_level(ray_pt, swing_at(roll_img, pitch_img))
                # Jitter filter: smooth small bbox wobble, but pass a real target
                # step (large angular jump, or a brand-new acquisition) straight
                # through. Reset the filter on the invalid->valid transition so a
                # fresh lock never smears the first detection.
                if track_jitter_enabled:
                    filt_state = track_state.get('los_filter')
                    if not isinstance(filt_state, dict) or not track_state.get('valid', False):
                        filt_state = _blank_los_filter_state()
                        track_state['los_filter'] = filt_state
                    los_used, los_step_deg, los_reseed_b = _filter_los(
                        filt_state, los_raw, start_time,
                        track_smoothing_sec, track_jump_deg)
                    los_reseed = 1 if los_reseed_b else 0
                else:
                    los_used = _normalize3(los_raw)
                track_state['los'] = los_used
                track_state['last_time'] = start_time
                track_state['valid'] = True
        if track_state.get('valid', False):
            if (start_time - float(track_state.get('last_time', 0.0))) >= track_lost_sec:
                track_state['valid'] = False
                track_state['los'] = None
    else:
        track_state['valid'] = False
        track_state['los'] = None
    target_locked = bool(track_state.get('valid', False) and track_state.get('los') is not None)

    # STAB point-and-hold target vector: seed on the enable rising edge (hold where
    # the camera is pointed) and move it on a user click. The held vector never
    # times out -- only a TRACK detection lock does. Manual-mode clicks never reach
    # here (the app node routes those straight to the driver). The anchor geometry
    # is identical to the TRACK detection ingest, so a click lands just like a
    # detection (smooth, roll/pitch-compensated servo).
    imu_valid_seed = (roll_deg != -999) and (pitch_deg != -999)
    click_update = auto_data_dict.get('auto2_click_update', None)
    if isinstance(click_update, dict) and (pan_stab_enabled or pan_scan_enabled):
        az_off = float(click_update.get('az_off', 0.0))
        el_off = float(click_update.get('el_off', 0.0))
        stab_state['los'] = anchor_offset_to_level(
            pan_deg, tilt_deg, az_off, el_off, R_swing, imu_valid_seed)
        stab_state['seeded'] = True
        # SWEEP + click -> fall back to Stabilize (scan off, stab on). Write the
        # override back so the app node absorbs the mode change next cycle.
        if pan_scan_enabled and not pan_stab_enabled:
            pan_stab_enabled = True
            pan_scan_enabled = False
            auto_data_dict['stab_enabled'] = True
            auto_data_dict['pan_stab_enabled'] = True
            auto_data_dict['tilt_stab_enabled'] = True
            auto_data_dict['scan_enabled'] = False
            auto_data_dict['pan_scan_enabled'] = False
            auto_data_dict['tilt_scan_enabled'] = False
    elif pan_stab_enabled and not stab_state.get('seeded', False):
        # Rising edge: capture current pointing (zero offset) as the held vector.
        stab_state['los'] = anchor_offset_to_level(
            pan_deg, tilt_deg, 0.0, 0.0, R_swing, imu_valid_seed)
        stab_state['seeded'] = True
    if not pan_stab_enabled:
        # STAB disabled -> force a re-seed on the next enable.
        stab_state['seeded'] = False
    # AI Tracking overlay revert: while a detection lock is active over STAB, keep
    # the held stab vector synced to the live lock LOS (both are boat_level unit
    # vectors in the same frame). Then when the lock ends -- AI Tracking disabled or
    # the detection times out -- STAB seamlessly HOLDS THE LAST TRACKED direction
    # (where the camera is now pointed) instead of snapping back to the vector that
    # was seeded when STAB was first enabled. (SCAN deliberately does NOT do this:
    # it resumes sweeping on lock loss, which the user confirmed is correct.)
    if pan_stab_enabled and target_locked and track_state.get('los') is not None:
        stab_state['los'] = list(track_state['los'])
        stab_state['seeded'] = True
    # Consume the one-shot click.
    auto_data_dict['auto2_click_update'] = None

    # Slider drag -> absolute target-vector az/el (one-shot ratios from the app node;
    # manual-mode slider moves never reach here -- the RUI routes those to the driver).
    # Ruler: azimuth = pan soft-limit span; elevation = fixed +/-EL span with the
    # tilt-slider inversion. In STAB a drag moves the held vector on one axis and
    # preserves the other. In SWEEP an AZIMUTH drag reverts to Stabilize (seed the
    # held vector at the dragged az + current sweep elevation, mirroring the click
    # fallback), while an ELEVATION drag keeps sweeping and just retunes the sweep el.
    pan_ratio_drag  = auto_data_dict.get('auto_pan_ratio_update', None)
    tilt_ratio_drag = auto_data_dict.get('auto_tilt_ratio_update', None)
    az_span = (pan_max_deg - pan_min_deg)
    el_span = (_AUTO2_SLIDER_EL_MAX - _AUTO2_SLIDER_EL_MIN)
    if pan_ratio_drag is not None and (pan_stab_enabled or pan_scan_enabled):
        r = max(0.0, min(1.0, float(pan_ratio_drag)))
        # The slider ruler lives in PT-frame azimuth (center = pan 0 = PT home).
        # Convert to boat-level by adding the PT mount heading so the stored LOS
        # vector is in the correct frame.  M_pt_yaw_deg adapts automatically to
        # any PT mount orientation.
        az_pt = pan_min_deg + (1.0 - r) * az_span
        az_new = az_pt + M_pt_yaw_deg
        if pan_scan_enabled and not pan_stab_enabled:
            # SWEEP + azimuth drag -> revert to Stabilize at the dragged az, holding
            # the current sweep elevation. Write the mode override back so the app
            # node absorbs it next cycle (same as the click Sweep->Stab fallback).
            el_hold = scan_state.get('el', None)
            if el_hold is None:
                el_hold = scan_el_deg
            stab_state['los'] = _normalize3(vector_from_az_el(az_new, el_hold))
            stab_state['seeded'] = True
            pan_stab_enabled = True
            pan_scan_enabled = False
            auto_data_dict['stab_enabled'] = True
            auto_data_dict['pan_stab_enabled'] = True
            auto_data_dict['tilt_stab_enabled'] = True
            auto_data_dict['scan_enabled'] = False
            auto_data_dict['pan_scan_enabled'] = False
            auto_data_dict['tilt_scan_enabled'] = False
        elif pan_stab_enabled and stab_state.get('los') is not None:
            # STAB azimuth drag -> move the held vector in az, preserve el.
            _, el_cur = solve_pan_tilt_from_vector(stab_state['los'])
            stab_state['los'] = _normalize3(vector_from_az_el(az_new, el_cur))
            stab_state['seeded'] = True
    if tilt_ratio_drag is not None and (pan_stab_enabled or pan_scan_enabled):
        r = max(0.0, min(1.0, float(tilt_ratio_drag)))
        # Match the driver/manual tilt convention. The driver maps ratio 0 ->
        # hardware tilt_max (look DOWN); elevation here is geometry (+up), so
        # ratio 0 = EL_MIN (down), ratio 1 = EL_MAX (up) on the fixed +/-EL ruler.
        el_new = _AUTO2_SLIDER_EL_MIN + r * el_span
        if pan_scan_enabled and not pan_stab_enabled:
            # SWEEP + elevation drag -> keep sweeping, just retune the sweep elevation.
            scan_state['el'] = el_new
        elif pan_stab_enabled and stab_state.get('los') is not None:
            # STAB elevation drag -> move the held vector in el, preserve az.
            az_cur, _ = solve_pan_tilt_from_vector(stab_state['los'])
            stab_state['los'] = _normalize3(vector_from_az_el(az_cur, el_new))
            stab_state['seeded'] = True
    # Consume the one-shot ratio drags.
    auto_data_dict['auto_pan_ratio_update'] = None
    auto_data_dict['auto_tilt_ratio_update'] = None

    # Mode select. LOCK (a live detection) OVERLAYS the active base mode (STAB or
    # SCAN): it wins while a target is locked and reverts to the base mode when the
    # lock ages out (STAB resumes holding its seeded vector; SCAN resumes sweeping).
    if pan_track_enabled and target_locked:
        mode = _auto2_MODE_LOCK
    elif pan_stab_enabled:
        mode = _auto2_MODE_STAB
    elif pan_scan_enabled:
        mode = _auto2_MODE_SCAN
    else:
        mode = _auto2_MODE_HOLD

    # SCAN integrator: advance only while SCAN owns the axis; seed to the live pan
    # angle on entry so resuming after a lock is bump-free. Bounce at soft limits.
    if mode == _auto2_MODE_SCAN:
        scan_dir = float(scan_state.get('dir', 1.0))
        if not scan_state.get('active', False):
            scan_az = pan_deg
            scan_state['active'] = True
            # Seed sweep elevation on entry (re-seeds each fresh sweep); left intact
            # if a tilt-slider drag already set it this cycle.
            if scan_state.get('el', None) is None:
                scan_state['el'] = scan_el_deg
        else:
            scan_az = float(scan_state.get('az', pan_deg))
            if dt > 0.0 and not first_cycle:
                scan_az += scan_dir * scan_speed_dps * dt
        if scan_az >= pan_max_deg:
            scan_az = pan_max_deg
            scan_dir = -1.0
        elif scan_az <= pan_min_deg:
            scan_az = pan_min_deg
            scan_dir = 1.0
        scan_state['az'] = scan_az
        scan_state['dir'] = scan_dir
    else:
        scan_state['active'] = False
        scan_state['el'] = None

    # Desired boat_level vector for the active mode.
    if mode == _auto2_MODE_STAB:
        # Point-and-hold: chase the seeded/clicked boat_level vector (captured on the
        # STAB enable rising edge, moved by a user click). The seed block above runs
        # before mode select, so los is always set while pan_stab_enabled; the None
        # guard is purely defensive -> hold current pointing (HOLD) if ever unseeded.
        if stab_state.get('los') is not None:
            v_level = stab_state['los']
            az_deg, el_deg = solve_pan_tilt_from_vector(v_level)
        else:
            mode = _auto2_MODE_HOLD
            az_deg = 0.0
            el_deg = 0.0
            v_level = [0.0, 0.0, 0.0]
    elif mode == _auto2_MODE_LOCK:
        v_level = track_state['los']
        az_deg, el_deg = solve_pan_tilt_from_vector(v_level)
    elif mode == _auto2_MODE_SCAN:
        # Scan integrator runs in PT-frame; offset to boat-level for the LOS vector.
        az_deg = float(scan_state['az']) + M_pt_yaw_deg
        scan_el = scan_state.get('el', None)
        el_deg = scan_el_deg if scan_el is None else float(scan_el)
        v_level = vector_from_az_el(az_deg, el_deg)
    else:  # HOLD
        az_deg = 0.0
        el_deg = 0.0
        v_level = [0.0, 0.0, 0.0]

    ##########################
    # 2. Transform boat_level -> pt_base + 3. solve desired pan/tilt
    ##########################
    imu_valid = (roll_deg != -999) and (pitch_deg != -999)
    transform_applied = 0
    desired_tilt_geom = 0.0
    if mode == _auto2_MODE_HOLD:
        # No vector to chase; aim desired = actual so the loop shows no error while
        # the axes are gated off below.
        v_pt = [0.0, 0.0, 0.0]
        desired_pan_deg = pan_deg
        desired_tilt_deg = tilt_deg
    else:
        if imu_valid:
            v_pt = level_to_pt_base(v_level)
            transform_applied = 1
        else:
            v_pt = v_level
        desired_pan_deg, desired_tilt_geom = solve_pan_tilt_from_vector(v_pt)
        # Map geometry tilt (+up) into the hardware tilt-feedback convention the
        # servo regulates (this rig's auto tilt feedback is +down; tilt_axis_sign).
        desired_tilt_deg = tilt_axis_sign * desired_tilt_geom

    ##########################
    # Feedforward rate sources (axis space; fed into _axis_velocity_update)
    ##########################
    # Reference FF: the intended commanded setpoint rate, SCAN-only. Only SCAN
    # drives a continuous setpoint -- STAB holds a fixed vector (any tilt motion
    # there is boat roll/pitch, owned by the gyro disturbance path), LOCK updates
    # are detection snaps (P term owns the step), HOLD is still.
    #
    # The sweep advances az at scan_dir*scan_speed_dps. Each axis' commanded RATE
    # is the az-partial of the solved setpoint with roll/pitch HELD FIXED, so it
    # captures ONLY the sweep (never boat motion or sensor noise -- the gyro path
    # owns those). Get it by perturbing az by a small daz and re-running the exact
    # vector->pt_base->solve chain used for the live setpoint, then differencing.
    # This is what makes tilt track during a sweep through a static roll/pitch
    # offset (a pure-az sweep cuts a cone whose tilt oscillates), and makes the
    # pan rate exact rather than the old d(pan)/d(az)=1 approximation.
    ref_rate_pan = 0.0
    ref_rate_tilt = 0.0
    dpan_daz = 0.0
    dtilt_daz = 0.0
    # Az-partial of the solved setpoint: ∂(motor)/∂(az_level) at the current
    # look direction, with roll/pitch held fixed. Used by SCAN reference FF AND
    # by the yaw-stab disturbance FF (heading rate projects through the same
    # geometry). Computed whenever needed by either path.
    if mode != _auto2_MODE_HOLD and (mode == _auto2_MODE_SCAN or yaw_on):
        daz_deg = 0.5
        v_az2 = vector_from_az_el(az_deg + daz_deg, el_deg)
        if imu_valid:
            v_pt2 = level_to_pt_base(v_az2)
        else:
            v_pt2 = v_az2
        pan2_deg, tilt2_geom = solve_pan_tilt_from_vector(v_pt2)
        dpan_daz = _wrap_to_180(pan2_deg - desired_pan_deg) / daz_deg
        dtilt_daz = (tilt2_geom - desired_tilt_geom) / daz_deg
    if mode == _auto2_MODE_SCAN:
        scan_rate_dps = float(scan_state.get('dir', 1.0)) * scan_speed_dps
        ref_rate_pan = dpan_daz * scan_rate_dps
        # Map geometry-tilt rate into the hardware tilt convention the servo
        # regulates (same tilt_axis_sign applied to the setpoint above).
        ref_rate_tilt = tilt_axis_sign * (dtilt_daz * scan_rate_dps)

    # Disturbance FF: project the measured body roll/pitch/yaw rates onto the pan
    # and tilt motor axes at the current pointing (A = desired pan, E = desired
    # tilt GEOMETRY elevation, +up, before tilt_axis_sign). Three components:
    #   roll/pitch (always): the analytic d/dt(desired_pos) boat motion induces.
    #     tilt_geom_rate = -roll_dps*sin(A) + pitch_dps*cos(A)
    #     pan_rate       =  tan(E) * ( roll_dps*cos(A) + pitch_dps*sin(A) )
    #   yaw (only when yaw_stab_enable): the heading rate re-projects through the
    #     az-partial because yaw-stab actively rotates the held vector by d_twist,
    #     creating setpoint motion at twist_rate_dps in azimuth.
    #     pan_rate  += dpan_daz  * twist_rate_dps
    #     tilt_rate += dtilt_daz * twist_rate_dps  (in hw convention)
    # Per-axis relevance falls straight out of the sin/cos/partial weighting.
    # Rate source: direct MicroStrain gyro (ahrs_rate_enable=1) or legacy
    # differentiated NavPose angles. Either way the projection math is the same.
    if mode != _auto2_MODE_HOLD and imu_valid:
        # Gyro rates arrive in the IMU SENSOR frame; the FF projection below uses
        # A = desired_pan and E = desired_tilt, which live in the PT_base frame
        # (v_pt = M_pt.T @ R_swing.T @ v_level). So the rates must be carried all
        # the way to the PT frame: sensor -> body via M_imu (matching the angle
        # path R_world_body = ... @ M_imu.T), then body -> PT_base via M_pt.T
        # (matching level_to_pt_base = M_pt.T @ ...). Stopping at the body frame
        # inverts roll/pitch whenever the PT mount is non-identity (e.g. 180deg-yaw
        # PT), which is why PT-0 worked but PT-180 came out backwards. Mounts here
        # are yaw-only, so roll/pitch are independent of the yaw rate; include yaw
        # for direct gyro yaw feedforward.
        _omega_body = M_imu @ np.array([
            float(auto_data_dict.get('roll_dps', 0.0)),
            float(auto_data_dict.get('pitch_dps', 0.0)),
            float(auto_data_dict.get('yaw_dps', 0.0)),
        ])
        _omega_pt = M_pt.T @ _omega_body
        roll_dps_ff  = float(_omega_pt[0])
        pitch_dps_ff = float(_omega_pt[1])
        yaw_dps_ff   = float(_omega_pt[2])
        a_rad = math.radians(desired_pan_deg)
        e_rad = math.radians(desired_tilt_geom)
        sin_a = math.sin(a_rad)
        cos_a = math.cos(a_rad)
        tan_e = math.tan(e_rad)
        if tan_e > _AUTO2_TAN_E_CLAMP:
            tan_e = _AUTO2_TAN_E_CLAMP
        elif tan_e < -_AUTO2_TAN_E_CLAMP:
            tan_e = -_AUTO2_TAN_E_CLAMP
        dist_rate_tilt = tilt_axis_sign * (-roll_dps_ff * sin_a + pitch_dps_ff * cos_a)
        dist_rate_pan  = tan_e * (roll_dps_ff * cos_a + pitch_dps_ff * sin_a)
        # Yaw-stab feedforward: use direct gyro yaw rate (transformed to PT frame)
        # with a rate deadband to suppress sensor noise on a still rig.
        if yaw_on and abs(yaw_dps_ff) >= yaw_rate_deadband_dps:
            dist_rate_pan  += dpan_daz * yaw_dps_ff
            dist_rate_tilt += tilt_axis_sign * dtilt_daz * yaw_dps_ff
    else:
        dist_rate_pan = 0.0
        dist_rate_tilt = 0.0

    ##########################
    # Send + telemetry helpers
    ##########################
    def _send_axis(method, axis_state, cmd_dps):
        # Throttle re-sends; force one send when commanding zero so the axis never
        # coasts on a stale velocity (velocity mode is not self-stopping).
        last = axis_state.get('last_cmd_vel_dps', 0.0)
        force = (cmd_dps == 0.0 and last != 0.0)
        if force or abs(cmd_dps - last) > cmd_change_dps:
            direction = _auto2_DIR_POS if cmd_dps >= 0.0 else _auto2_DIR_NEG
            method_name = getattr(method, '__name__', str(method))
            try:
                if _io_log_enabled:
                    logger.log_warn("nepi_auto_pt:pt_auto_2._send_axis: CMD_PATH=JOG_AXIS CMD %s direction=%s speed_dps=%.3f duration_s=-1" %
                                    (method_name, direction, abs(cmd_dps)))
                method(direction, abs(cmd_dps), duration_s=-1)
            except Exception as e:
                logger.log_warn("pt_auto_2 axis jog failed: " + str(e))
            axis_state['last_cmd_vel_dps'] = cmd_dps

    def _finish(cmd_pan_dps, cmd_tilt_dps, pan_dbg, tilt_dbg, mode_code, watchdog_active):
        if pt_connect_if is not None:
            # Falling edge into HOLD: a mode just turned off (-> Manual) but an axis
            # was still commanding a nonzero velocity last cycle. The axes run in
            # velocity-jog mode (run-until-next-command), and a bare speed-0 jog is
            # NOT a reliable stop on this SS109 driver -- it floors speed-0 to a crawl
            # count, so the head keeps drifting toward a soft limit ("it wants to go
            # somewhere"). So on that edge, do EXACTLY what the RUI STOP button does:
            # stop_moving() publishes to the ptx 'stop_moving' topic; because this
            # driver wires stopMovingCb=None, the device IF parks the head with an
            # absolute goto to its CURRENT live position (device_if_ptx stopPanTilt).
            # Critically we do NOT also send the speed-0 jog: that competing velocity
            # command races the stop on a separate driver thread (and re-floors to the
            # crawl), which is what made the head keep moving. Fire once per edge
            # (gated by was_moving) and zero last_cmd so normal sends resume cleanly.
            pan_was_moving  = pan_state.get('last_cmd_vel_dps', 0.0) != 0.0
            tilt_was_moving = tilt_state.get('last_cmd_vel_dps', 0.0) != 0.0
            hold_edge = (mode_code == _auto2_MODE_HOLD) and (pan_was_moving or tilt_was_moving)
            if hold_edge:
                try:
                    logger.log_warn("nepi_auto_pt:pt_auto_2._finish: CMD_PATH=HOLD_EDGE_STOP CMD stop_moving hold_edge=1 mode_code=%s pan_was_moving=%s tilt_was_moving=%s" %
                                    (mode_code, pan_was_moving, tilt_was_moving))
                    pt_connect_if.stop_moving()
                except Exception as e:
                    logger.log_warn("pt_auto_2 hold-on-disable stop failed: " + str(e))
                pan_state['last_cmd_vel_dps'] = 0.0
                tilt_state['last_cmd_vel_dps'] = 0.0
            else:
                _send_axis(pt_connect_if.jog_timed_speed_dps_pan, pan_state, cmd_pan_dps)
                _send_axis(pt_connect_if.jog_timed_speed_dps_tilt, tilt_state, cmd_tilt_dps)
        # Persist controller state
        auto_data_dict['auto2_pan_state'] = pan_state
        auto_data_dict['auto2_tilt_state'] = tilt_state
        auto_data_dict['auto2_scan_state'] = scan_state
        auto_data_dict['auto2_track_state'] = track_state
        auto_data_dict['auto2_stab_state'] = stab_state
        auto_data_dict['auto2_yaw_lpf_state'] = yaw_lpf_state
        auto_data_dict['auto2_last_update_time'] = start_time
        auto_data_dict['auto2_last_update_mono'] = mono_now
        auto_data_dict['auto2_last_data_time'] = last_data_time
        auto_data_dict['auto2_data_change_time'] = data_change_t
        # Status fields read by the app node publish_status()
        auto_data_dict['auto_pan_goal'] = round(desired_pan_deg, 2)
        auto_data_dict['auto_tilt_goal'] = round(desired_tilt_deg, 2)
        auto_data_dict['auto_pan_dps'] = cmd_pan_dps
        auto_data_dict['auto_tilt_dps'] = cmd_tilt_dps
        auto_data_dict['auto_pan_deg'] = round(pan_deg, 2)
        auto_data_dict['auto_tilt_deg'] = round(tilt_deg, 2)
        auto_data_dict['auto_pan_adj'] = round(pan_dbg.get('err', 0.0), 2)
        auto_data_dict['auto_tilt_adj'] = round(tilt_dbg.get('err', 0.0), 2)
        # Slider thumb feedback (Stab/Sweep): map the live boat_level target az/el
        # onto the slider ruler. Display-only clamp to [0,1] -> an out-of-ruler
        # vector (extreme roll) rails the thumb without disturbing the held vector.
        # The RUI only reads these while stabbing/scanning; manual/HOLD uses the
        # driver's own goal ratio.  Convert boat-level az back into PT-frame so
        # the thumb matches the slider ruler (center = pan 0 = PT home).
        if az_span > 1e-6:
            az_pt_disp = _wrap_to_180(az_deg - M_pt_yaw_deg)
            pan_disp = (pan_max_deg - az_pt_disp) / az_span
        else:
            pan_disp = 0.5
        tilt_disp = (el_deg - _AUTO2_SLIDER_EL_MIN) / el_span
        pan_disp = round(max(0.0, min(1.0, pan_disp)), 4)
        tilt_disp = round(max(0.0, min(1.0, tilt_disp)), 4)
        prev_pan_disp = auto_data_dict.get('auto_pan_ratio_display', None)
        prev_tilt_disp = auto_data_dict.get('auto_tilt_ratio_display', None)
        prev_pan_set = auto_data_dict.get('auto_pan_ratio_set', None)
        prev_tilt_set = auto_data_dict.get('auto_tilt_ratio_set', None)
        auto_data_dict['auto_pan_ratio_display'] = pan_disp
        auto_data_dict['auto_tilt_ratio_display'] = tilt_disp
        # The RUI slider thumb reads auto_*_ratio_set; mirror the live display
        # value into _set so the unchanged RUI thumb tracks the held vector.
        auto_data_dict['auto_pan_ratio_set'] = pan_disp
        auto_data_dict['auto_tilt_ratio_set'] = tilt_disp
        if (prev_pan_disp != pan_disp or prev_tilt_disp != tilt_disp
                or prev_pan_set != pan_disp or prev_tilt_set != tilt_disp) and _io_log_enabled:
            logger.log_warn("nepi_auto_pt:pt_auto_2._finish: OUTPUT slider_update mode_code=%s pan_display=%.4f tilt_display=%.4f pan_set=%.4f tilt_set=%.4f" %
                            (mode_code, pan_disp, tilt_disp, pan_disp, tilt_disp),
                            throttle_s=max(0.0, _io_log_interval))
        if telem_enabled:
            _auto2_send_telemetry({
                't': round(start_time, 4),
                'loop_hz': round(loop_hz, 2),
                'cycle_ms': round(float(auto_data_dict.get('auto2_cycle_ms', 0.0)), 2),
                'overrun': int(auto_data_dict.get('auto2_overrun', 0)),
                'mode_code': mode_code,                 # 0 hold,1 stab,2 scan,3 lock,9 watchdog
                'pan_stab_enabled': 1 if pan_stab_enabled else 0,
                'pan_scan_enabled': 1 if pan_scan_enabled else 0,
                'pan_track_enabled': 1 if pan_track_enabled else 0,
                'target_locked': 1 if target_locked else 0,
                'stab_seeded': 1 if stab_state.get('seeded', False) else 0,
                'target_age_sec': round(start_time - float(track_state.get('last_time', 0.0)), 3) if track_state.get('valid', False) else -1.0,
                'track_los_step_deg': round(los_step_deg, 3),
                'track_los_reseed': los_reseed,
                'scan_az_deg': round(float(scan_state.get('az', 0.0)), 3),
                'scan_dir': int(scan_state.get('dir', 1)),
                'pan_on': 1 if pan_on else 0,
                'tilt_on': 1 if tilt_on else 0,
                'watchdog_active': watchdog_active,
                'transform_applied': transform_applied,
                'yaw_on': 1 if yaw_on else 0,
                'twist_deg': round(twist_deg, 2) if imu_valid else 0.0,
                'twist_rate_dps': round(twist_rate_dps, 3),
                'roll_deg': round(roll_deg, 3) if imu_valid else 0.0,
                'pitch_deg': round(pitch_deg, 3) if imu_valid else 0.0,
                'image_roll_deg': round(image_roll_deg, 3) if imu_valid else 0.0,
                'az_deg': round(az_deg, 3),
                'el_deg': round(el_deg, 3),
                'v_level_x': round(v_level[0], 4),
                'v_level_y': round(v_level[1], 4),
                'v_level_z': round(v_level[2], 4),
                'v_pt_x': round(v_pt[0], 4),
                'v_pt_y': round(v_pt[1], 4),
                'v_pt_z': round(v_pt[2], 4),
                'desired_pan_deg': round(desired_pan_deg, 3),
                'desired_tilt_deg': round(desired_tilt_deg, 3),
                'actual_pan_deg': round(pan_deg, 3),
                'actual_tilt_deg': round(tilt_deg, 3),
                'pan_err_deg': round(pan_dbg.get('err', 0.0), 3),
                'tilt_err_deg': round(tilt_dbg.get('err', 0.0), 3),
                'pan_cmd_dps': round(cmd_pan_dps, 3),
                'tilt_cmd_dps': round(cmd_tilt_dps, 3),
                'pan_p_term': round(pan_dbg.get('p_term', 0.0), 3),
                'tilt_p_term': round(tilt_dbg.get('p_term', 0.0), 3),
                'pan_i_term': round(pan_dbg.get('i_term', 0.0), 3),
                'tilt_i_term': round(tilt_dbg.get('i_term', 0.0), 3),
                'pan_ff': round(pan_dbg.get('ff', 0.0), 3),
                'tilt_ff': round(tilt_dbg.get('ff', 0.0), 3),
                'pan_ref_ff': round(pan_dbg.get('ref_ff', 0.0), 3),
                'tilt_ref_ff': round(tilt_dbg.get('ref_ff', 0.0), 3),
                'pan_dist_ff': round(pan_dbg.get('dist_ff', 0.0), 3),
                'tilt_dist_ff': round(tilt_dbg.get('dist_ff', 0.0), 3),
                'pan_rate_filt': round(pan_dbg.get('rate_filt', 0.0), 3),
                'tilt_rate_filt': round(tilt_dbg.get('rate_filt', 0.0), 3),
                'pan_integral': round(pan_dbg.get('integral', 0.0), 4),
                'tilt_integral': round(tilt_dbg.get('integral', 0.0), 4),
                'pan_sw_freq_hz': round(pan_dbg.get('sw_freq_hz', 0.0), 4),
                'tilt_sw_freq_hz': round(tilt_dbg.get('sw_freq_hz', 0.0), 4),
                'pan_sw_conf': round(pan_dbg.get('sw_conf', 0.0), 3),
                'tilt_sw_conf': round(tilt_dbg.get('sw_conf', 0.0), 3),
                'pan_softstop': pan_dbg.get('softstop', 0),
                'tilt_softstop': tilt_dbg.get('softstop', 0),
                'pan_vel_min': pan_dbg.get('vel_min', 0),
                'tilt_vel_min': tilt_dbg.get('vel_min', 0),
                'eff_max_pan_dps': round(eff_max_pan_dps, 2),
                'eff_max_tilt_dps': round(eff_max_tilt_dps, 2),
            }, telem_ip)
        return auto_data_dict, auto_settings_dict

    ##########################
    # Watchdog: data_time must keep changing (app-node liveness).
    # data_time == 0.0 is the "never populated" sentinel; skip the check then.
    ##########################
    data_time = auto_data_dict.get('data_time', 0.0)
    if data_time != last_data_time:
        last_data_time = data_time
        data_change_t = start_time
    if data_change_t is None:
        data_change_t = start_time
    data_is_stamped = data_time not in (0.0, 0, None)
    watchdog_active = 1 if (data_is_stamped and (start_time - data_change_t) > watchdog_sec) else 0

    ##########################
    # 4./5. Gate both axes together -> run the axis servos
    ##########################
    mode_active = (mode != _auto2_MODE_HOLD)
    pan_on = mode_active and not watchdog_active
    tilt_on = mode_active and not watchdog_active

    cmd_pan_dps, pan_dbg = _axis_velocity_update(
        pan_state, desired_pan_deg, pan_deg, pan_min_deg, pan_max_deg, dt,
        first_cycle, True, pan_on, pan_cmd_sign, eff_max_pan_dps, pan_gains,
        ref_rate_pan, dist_rate_pan)
    cmd_tilt_dps, tilt_dbg = _axis_velocity_update(
        tilt_state, desired_tilt_deg, tilt_deg, tilt_min_deg, tilt_max_deg, dt,
        first_cycle, False, tilt_on, tilt_cmd_sign, eff_max_tilt_dps, tilt_gains,
        ref_rate_tilt, dist_rate_tilt)

    mode_code = _auto2_MODE_WATCHDOG if watchdog_active else mode

    # --- OUTPUT snapshot ---
    if _do_io_log:
        logger.log_warn("nepi_auto_pt:pt_auto_2: OUTPUT mode_code=%s cmd_pan_dps=%.3f cmd_tilt_dps=%.3f goal_pan=%.2f goal_tilt=%.2f actual_pan=%.2f actual_tilt=%.2f watchdog=%s" %
                        (mode_code, cmd_pan_dps, cmd_tilt_dps,
                         desired_pan_deg, desired_tilt_deg,
                         pan_deg, tilt_deg, watchdog_active))

    ##########################
    # 6./7. Output commands (gated), persist, telemetry
    ##########################
    return _finish(cmd_pan_dps, cmd_tilt_dps, pan_dbg, tilt_dbg, mode_code, watchdog_active)


PROCESSES_DICT['pt_auto_2'] = {'process_function': pt_auto_2, 
                                             'default_settings_dict': pt_auto_2_settings}



#########################
# Auto Utility Functions
#########################

def create_processes_dict():
    processes_dict = dict()
    for process_name in PROCESSES_DICT.keys():
        processes_dict[process_name] = PROCESSES_DICT[process_name]['default_settings_dict']
    return processes_dict

def update_processes_dict(auto_processes_dict):
    clean_auto_dict = create_processes_dict()
    for auto_process in clean_auto_dict.keys():
        if auto_process in auto_processes_dict.keys():
            for key in clean_auto_dict[auto_process].keys():
                if key in auto_processes_dict[auto_process].keys() and key != 'auto_controls_dict':
                    clean_auto_dict[auto_process][key] = auto_processes_dict[auto_process][key]
            for key in clean_auto_dict[auto_process]['auto_controls_dict'].keys():
                if key in auto_processes_dict[auto_process]['auto_controls_dict'].keys():
                    clean_auto_dict[auto_process]['auto_controls_dict'][key] = auto_processes_dict[auto_process]['auto_controls_dict'][key]
    return clean_auto_dict

def get_blank_data_dict():
    return copy.deepcopy(DATA_DICT)
