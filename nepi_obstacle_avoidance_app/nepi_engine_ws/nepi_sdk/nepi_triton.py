#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# Triton ICD helper logic: pull PLI (position/location info) from the Ocean Aero
# Triton, derive the sun's position from the vehicle's lat/lon/time, and pick the
# camera with the best light (sun behind it, no glare) for the current heading.
#
# PURE LOGIC ONLY -- no ROS / rospy / nepi_sdk imports, stdlib only (math,
# datetime, json, urllib). This mirrors nepi_auto_pt.py: the math/parsing lives
# here (hot-reloadable + unit-testable with no boat and no ROS master), and the
# automation node (triton_sun_camera_node.py) owns the ROS wiring and calls
# analyze_pli() on a timer.
#
# Field scaling and the two assumptions below are PROVEN/inferred from a real PLI
# sample (see repo notes); they are isolated as labeled constants so they are
# trivial to flip when the client confirms against the formal ICD document.

import json
import math
import urllib.request
import urllib.error
from datetime import datetime


# -----------------------------------------------------------------------------
# Triton connection defaults (from the client's triton_icd.py reference client).
# -----------------------------------------------------------------------------
DEFAULT_TRITON_HOST = "10.10.30.1"
DEFAULT_CLIENT_API_KEY = "caucus-caterer-engulf"
DEFAULT_BOAT_ID = 30
DATA_OUT_PATH = "/api/client/data/out/"

# -----------------------------------------------------------------------------
# boatlog integer-field scaling. The client's DB schema (2026-06-27) confirms
# the shape: hdg/cog/roll/pit_a/sog/tws are smallint integers (so they ARE
# scaled), while lat/lon are decimal(10,8)/(11,8) and used natively (NOT scaled).
# Use the fused 'lat'/'lon' (per client) -- NOT dlat/dlon (GPS) or ilat/ilon
# (INS). The x10 multiplier is proven by hdg=3305 > 360 (must be scaled; /10 =
# 330.5 deg, with cog/twd a consistent cluster). The client did not state the
# multiplier explicitly; ask if a non-x10 field ever shows up wrong.
# -----------------------------------------------------------------------------
ANGLE_SCALE = 10.0   # hdg, cog, roll, pit_a, twd  (raw int -> degrees)
SPEED_SCALE = 10.0   # sog, tws                    (raw int -> knots)

# -----------------------------------------------------------------------------
# Frame/time conventions:
#   BRG_IS_BOAT_RELATIVE CONFIRMED by client (2026-06-27): camera 'brg' is
#                        relative to the boat, so world bearing = boat hdg + brg.
#   TIME_IS_UTC          boatlog 'time' is UTC -- CONFIRMED by client (2026-06-29).
#                        (Also corroborated by sun geometry: only UTC produced a
#                        plausible high midday sun on the sample.) If a future
#                        deployment ever reports local time, set False and supply
#                        LOCAL_UTC_OFFSET_HOURS.
# -----------------------------------------------------------------------------
TIME_IS_UTC = True
LOCAL_UTC_OFFSET_HOURS = 0.0       # used only when TIME_IS_UTC is False
BRG_IS_BOAT_RELATIVE = True

# -----------------------------------------------------------------------------
# Camera-selection tuning.
#   NIGHT_ELEVATION_DEG  sun elevation at/below which it is "night"; the
#                        sun-glare criterion no longer applies (see analyze_pli).
#   GLARE_EXCLUDE        drop any camera with the sun inside its FOV from
#                        daytime selection (sun in frame washes out the image).
# -----------------------------------------------------------------------------
NIGHT_ELEVATION_DEG = 0.0
GLARE_EXCLUDE = True


# =============================================================================
# Triton ICD client (read-only "Data Out" pull). Stdlib urllib only.
# =============================================================================
def post_json(url, payload, headers=None, timeout=10):
    """POST a JSON body, return decoded JSON (dict/list) or raw text. Raises on
    HTTP/connection errors so the caller can decide how to degrade."""
    body = json.dumps(payload).encode("utf-8")
    req_headers = {"Content-Type": "application/json"}
    if headers:
        req_headers.update(headers)
    request = urllib.request.Request(url, data=body, headers=req_headers, method="POST")
    with urllib.request.urlopen(request, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8")
    if not raw:
        return None
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return raw


def fetch_pli(host=DEFAULT_TRITON_HOST, api_key=DEFAULT_CLIENT_API_KEY, timeout=10):
    """Pull live PLI ("Data Out") from the Triton. Returns the parsed dict."""
    url = "http://" + str(host).rstrip("/") + DATA_OUT_PATH
    return post_json(url, {"client_api_key": api_key}, timeout=timeout)


def load_pli_text(text):
    """Parse PLI from a string, tolerating a leading banner (the client's saved
    'output.json' is prefixed with a '=== PLI Data Out ===' header). Returns the
    parsed dict, or raises ValueError if no JSON object is found."""
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end <= start:
        raise ValueError("no JSON object found in PLI text")
    return json.loads(text[start:end + 1])


def load_pli_file(path):
    """Read and parse a PLI fixture/JSON file (for offline dev with no boat)."""
    with open(path, "r") as f:
        return load_pli_text(f.read())


# =============================================================================
# Solar position (NOAA equations; ~0.1 deg). Pure stdlib, matches the validated
# scratchpad demo. dt MUST be UTC.
# =============================================================================
def _julian_day(dt):
    Y, M, D = dt.year, dt.month, dt.day
    frac = (dt.hour + dt.minute / 60.0 + dt.second / 3600.0) / 24.0
    if M <= 2:
        Y -= 1
        M += 12
    A = Y // 100
    B = 2 - A + A // 4
    return (math.floor(365.25 * (Y + 4716)) + math.floor(30.6001 * (M + 1))
            + D + B - 1524.5 + frac)


def sun_az_el(lat_deg, lon_deg, dt_utc):
    """Solar azimuth (deg, from true north, clockwise) and elevation (deg) for a
    lat/lon (east-positive lon) at a UTC datetime."""
    jd = _julian_day(dt_utc)
    T = (jd - 2451545.0) / 36525.0
    L0 = (280.46646 + T * (36000.76983 + T * 0.0003032)) % 360.0
    M = 357.52911 + T * (35999.05029 - 0.0001537 * T)
    e = 0.016708634 - T * (0.000042037 + 0.0000001267 * T)
    Mr = math.radians(M)
    C = (math.sin(Mr) * (1.914602 - T * (0.004817 + 0.000014 * T))
         + math.sin(2 * Mr) * (0.019993 - 0.000101 * T)
         + math.sin(3 * Mr) * 0.000289)
    true_long = L0 + C
    omega = 125.04 - 1934.136 * T
    app_long = true_long - 0.00569 - 0.00478 * math.sin(math.radians(omega))
    e0 = 23 + (26 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60) / 60
    e_corr = e0 + 0.00256 * math.cos(math.radians(omega))
    decl = math.degrees(math.asin(math.sin(math.radians(e_corr))
                                  * math.sin(math.radians(app_long))))
    y = math.tan(math.radians(e_corr / 2.0)) ** 2
    L0r = math.radians(L0)
    eot = 4 * math.degrees(
        y * math.sin(2 * L0r) - 2 * e * math.sin(Mr)
        + 4 * e * y * math.sin(Mr) * math.cos(2 * L0r)
        - 0.5 * y * y * math.sin(4 * L0r) - 1.25 * e * e * math.sin(2 * Mr))
    mins = dt_utc.hour * 60 + dt_utc.minute + dt_utc.second / 60.0
    tst = (mins + eot + 4 * lon_deg) % 1440.0          # true solar time, tz=0 (UTC)
    ha = tst / 4.0 - 180.0 if tst / 4.0 >= 0 else tst / 4.0 + 180.0
    latr = math.radians(lat_deg)
    declr = math.radians(decl)
    har = math.radians(ha)
    cos_zen = (math.sin(latr) * math.sin(declr)
               + math.cos(latr) * math.cos(declr) * math.cos(har))
    cos_zen = max(-1.0, min(1.0, cos_zen))
    zenith = math.degrees(math.acos(cos_zen))
    elev = 90.0 - zenith
    den = math.cos(latr) * math.sin(math.radians(zenith))
    if abs(den) < 1e-9:
        az = 0.0
    else:
        ca = (math.sin(latr) * math.cos(math.radians(zenith)) - math.sin(declr)) / den
        ca = max(-1.0, min(1.0, ca))
        az = math.degrees(math.acos(ca))
        az = (az + 180.0) % 360.0 if ha > 0 else (540.0 - az) % 360.0
    return az, elev


def wrap180(a):
    """Wrap an angle (deg) to (-180, 180]."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


# =============================================================================
# PLI parsing
# =============================================================================
def _to_float(x, default=None):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def parse_boatlog(pli):
    """Pull the fields we need from PLI 'boatlog', applying scaling. Returns a
    dict with lat/lon (deg), time_str, dt_utc, hdg_deg -- or None if the
    essential nav fields are missing/invalid."""
    bl = (pli or {}).get("boatlog")
    if not isinstance(bl, dict):
        return None
    lat = _to_float(bl.get("lat"))
    lon = _to_float(bl.get("lon"))
    time_str = bl.get("time")
    if lat is None or lon is None or not time_str:
        return None
    try:
        dt_naive = datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S")
    except (TypeError, ValueError):
        return None
    # Normalize to UTC for the solar calc. We never use the device clock.
    if TIME_IS_UTC:
        dt_utc = dt_naive
    else:
        dt_utc = dt_naive - _timedelta_hours(LOCAL_UTC_OFFSET_HOURS)
    hdg_raw = _to_float(bl.get("hdg"))
    hdg_deg = (hdg_raw / ANGLE_SCALE) % 360.0 if hdg_raw is not None else None
    cog_raw = _to_float(bl.get("cog"))
    sog_raw = _to_float(bl.get("sog"))
    return {
        "lat": lat,
        "lon": lon,
        "time_str": time_str,
        "dt_utc": dt_utc,
        "hdg_deg": hdg_deg,
        "cog_deg": (cog_raw / ANGLE_SCALE) % 360.0 if cog_raw is not None else None,
        "sog_knots": sog_raw / SPEED_SCALE if sog_raw is not None else None,
    }


def _timedelta_hours(hours):
    from datetime import timedelta
    return timedelta(hours=hours)


def parse_cameras(pli):
    """Return the fixed cameras from PLI 'camera[]' as dicts with brg/fov in
    degrees (camera brg/fov are plain degrees, NOT scaled). Skips soft-deleted
    entries. The separate movable 'ptz' object is intentionally excluded."""
    out = []
    for cam in (pli or {}).get("camera", []) or []:
        if not isinstance(cam, dict):
            continue
        if cam.get("deleted_at"):
            continue
        brg = _to_float(cam.get("brg"))
        fov = _to_float(cam.get("fov"))
        if brg is None or fov is None:
            continue
        out.append({
            "id": cam.get("id"),
            "name": cam.get("name", ""),
            "type": cam.get("type"),
            "brg_deg": brg,
            "fov_deg": fov,
            "url_rtsp": cam.get("url_rtsp", ""),
        })
    return out


# =============================================================================
# Camera selection
# =============================================================================
def select_camera(sun_az_deg, sun_el_deg, hdg_deg, cameras):
    """Score each camera by how well the sun lights its scene and pick the best.

    world_brg = hdg + brg (boat-relative) or brg (already world).
    off_sun   = wrap180(world_brg - sun_az): 0 = staring into the sun, +/-180 = sun
                directly behind.
    glare     = sun inside the camera's FOV (|off_sun| < fov/2).
    score     = -cos(off_sun): +1 = sun fully behind (best light), -1 = into sun.

    Daytime: pick the max-score camera, excluding glare cameras when possible.
    Night (sun at/below NIGHT_ELEVATION_DEG): the sun criterion does not apply --
    returns selected=None with a note (night handling, e.g. thermal, is an open
    design question pending client input)."""
    is_day = sun_el_deg > NIGHT_ELEVATION_DEG
    scored = []
    for cam in cameras:
        if BRG_IS_BOAT_RELATIVE and hdg_deg is not None:
            world_brg = (hdg_deg + cam["brg_deg"]) % 360.0
        else:
            world_brg = cam["brg_deg"] % 360.0
        off_sun = wrap180(world_brg - sun_az_deg)
        glare = abs(off_sun) < (cam["fov_deg"] / 2.0)
        score = -math.cos(math.radians(off_sun))
        scored.append({
            "id": cam["id"], "name": cam["name"],
            "brg_deg": cam["brg_deg"], "fov_deg": cam["fov_deg"],
            "world_brg_deg": round(world_brg, 1),
            "off_sun_deg": round(off_sun, 1),
            "glare": glare, "score": round(score, 3),
            "url_rtsp": cam.get("url_rtsp", ""),
        })

    selected = None
    note = ""
    if not scored:
        note = "no cameras in PLI"
    elif not is_day:
        note = "night: sun at/below horizon, glare-based selection not applicable"
    else:
        candidates = [c for c in scored if not (GLARE_EXCLUDE and c["glare"])]
        if not candidates:
            candidates = scored
            note = "all cameras have the sun in FOV; picked least-bad"
        best = max(candidates, key=lambda c: c["score"])
        selected = {"id": best["id"], "name": best["name"],
                    "url_rtsp": best.get("url_rtsp", "")}
    return selected, scored, is_day, note


# =============================================================================
# Top-level entry point -- the node and the offline test both call this.
# =============================================================================
def analyze_pli(pli):
    """Full pipeline: PLI dict -> sun position -> optimal-camera recommendation.

    Returns a result dict; 'ok' is False (with 'reason') if the PLI lacks the
    nav fields needed to compute a recommendation."""
    result = {
        "ok": False, "reason": "",
        "time_str": None, "lat": None, "lon": None, "boat_hdg_deg": None,
        "sun_az_deg": None, "sun_el_deg": None, "is_day": None,
        "selected_camera": None, "cameras": [],
        "assumptions": {"time_is_utc": TIME_IS_UTC,
                        "brg_is_boat_relative": BRG_IS_BOAT_RELATIVE},
    }
    nav = parse_boatlog(pli)
    if nav is None:
        result["reason"] = "boatlog missing lat/lon/time"
        return result
    if nav["hdg_deg"] is None and BRG_IS_BOAT_RELATIVE:
        result["reason"] = "boatlog missing hdg (needed for boat-relative camera bearings)"
        return result

    sun_az, sun_el = sun_az_el(nav["lat"], nav["lon"], nav["dt_utc"])
    cameras = parse_cameras(pli)
    selected, scored, is_day, note = select_camera(sun_az, sun_el, nav["hdg_deg"], cameras)

    result.update({
        "ok": True, "reason": note,
        "time_str": nav["time_str"], "lat": nav["lat"], "lon": nav["lon"],
        "boat_hdg_deg": round(nav["hdg_deg"], 1) if nav["hdg_deg"] is not None else None,
        "sun_az_deg": round(sun_az, 1), "sun_el_deg": round(sun_el, 1),
        "is_day": is_day, "selected_camera": selected, "cameras": scored,
    })
    return result


# =============================================================================
# Triton notifications (Argus UI log endpoint). Used by triton_notify_node.py to
# push NEPI detections to the interface so operators are alerted off-screen.
# Endpoint: POST http://<host>:9230/airlock/local/logs/in  body {type,source,data}
# UI shows "<source>: <data>". Behind LocalhostOrApiKey middleware -- loopback
# needs no key, but NEPI is remote to the Triton so we send client_api_key.
# Ported from the client's triton_notify.py reference client.
# =============================================================================
NOTIFY_PORT = 9230
LOGS_IN_PATH = "/airlock/local/logs/in"
NOTIFY_LOG_TYPES = ("debug", "error", "timer", "user", "info", "health")

# Default detection-notification mapping. TUNABLE pending the client's answer on
# alert levels: routine detections -> "info"; source shown before the message.
NOTIFY_DETECTION_TYPE = "info"
NOTIFY_DETECTION_SOURCE = "detection"

# Target.msg unset sentinels (skip these when formatting the message).
_FLOAT_UNSET = -999.0


def send_notification(log_type, source, message,
                      host=DEFAULT_TRITON_HOST, port=NOTIFY_PORT,
                      api_key=DEFAULT_CLIENT_API_KEY, timeout=10):
    """POST one notification to the Triton UI. Raises on HTTP/connection error."""
    if log_type not in NOTIFY_LOG_TYPES:
        log_type = "info"
    url = "http://%s:%d%s" % (str(host), int(port), LOGS_IN_PATH)
    payload = {"type": log_type, "source": source, "data": message}
    headers = {"client_api_key": api_key} if api_key else {}
    return post_json(url, payload, headers=headers, timeout=timeout)


def build_detection_notification(target_name, confidence=None, azimuth_deg=None,
                                 range_m=None, log_type=NOTIFY_DETECTION_TYPE,
                                 source=NOTIFY_DETECTION_SOURCE):
    """Format a detection into {type, source, message}. Skips unset (-999) fields.
    Pure/ROS-free so the node can extract primitives from the Target msg and the
    formatting stays unit-testable."""
    parts = ["Detected " + str(target_name or "target")]
    c = _to_float(confidence)
    if c is not None and 0.0 <= c <= 1.0:
        parts.append("conf %.2f" % c)
    az = _to_float(azimuth_deg)
    if az is not None and az != _FLOAT_UNSET:
        parts.append("brg %.0f deg" % az)
    rng = _to_float(range_m)
    if rng is not None and rng > 0.0:
        parts.append("rng %.0f m" % rng)
    if log_type not in NOTIFY_LOG_TYPES:
        log_type = "info"
    return {"type": log_type, "source": source, "message": ", ".join(parts)}


def build_detection_batch_notification(detections, log_type=NOTIFY_DETECTION_TYPE,
                                       source=NOTIFY_DETECTION_SOURCE):
    """Coalesce one debounce window's worth of detections into a SINGLE
    notification so the operator gets at most one UI alert per window.

    'detections' is a list of dicts: {name, confidence, azimuth_deg, range_m}.
    0 -> None (caller sends nothing); 1 -> the detailed single-target format
    (with bearing/range); N -> a compact "Detected N targets: name (conf), ..."
    summary (per-target bearing/range omitted to keep the line readable)."""
    items = list(detections or [])
    if len(items) == 0:
        return None
    if len(items) == 1:
        d = items[0]
        return build_detection_notification(
            d.get("name"), confidence=d.get("confidence"),
            azimuth_deg=d.get("azimuth_deg"), range_m=d.get("range_m"),
            log_type=log_type, source=source)
    labels = []
    for d in items:
        label = str(d.get("name") or "target")
        c = _to_float(d.get("confidence"))
        if c is not None and 0.0 <= c <= 1.0:
            label += " (%.2f)" % c
        labels.append(label)
    if log_type not in NOTIFY_LOG_TYPES:
        log_type = "info"
    message = "Detected %d targets: %s" % (len(items), ", ".join(labels))
    return {"type": log_type, "source": source, "message": message}
