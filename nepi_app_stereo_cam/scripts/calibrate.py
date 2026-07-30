#!/usr/bin/env python3
"""
CLAUDE GENERATED.
Stereo calibration + rectification for the custom stereo driver.

Library only -- no command line. Calibration is driven from the NEPI RUI
(Custom Stereo Depth app -> "Stereo Calibration" panel), which talks to the
app node topics wired in custom_stereo_app_node.py:

    set_calib_board_value   (UpdateFloat: board_cols / board_rows / square_mm)
    set_calib_file          (String: where the .npz is written / read)
    capture_calib_frame     (Empty: grab the live L/R pair, find the board)
    solve_calib             (Empty: solve + save + start rectifying)
    clear_calib             (Empty: drop captures)
    load_calib              (Empty: reload the saved .npz)

Three jobs:
  1. StereoCalibrator -- the RUI capture/solve flow. Hold a chessboard up,
     hit Capture ~10-20 times at varied distances/angles/image corners, hit
     Solve. Writes the .npz and reports RMS + rectified epipolar error.
  2. Rectifier        -- per frame, in the node. Loads the .npz and warps raw
     L/R frames so features line up on the same row for block matching.
  3. run_tuner()      -- OPTIONAL developer-workstation helper: OpenCV sliders
     over saved images, for finding good block-match params offline. Needs a
     desktop (cv2 GUI), so it is not used on the device -- on the device the
     RUI's process-settings panel does this live against the depth viewer.
     Run it from a dev box with:
         python3 -c "import calibrate; \\
             calibrate.run_tuner('L.png','R.png','stereo_calib.npz')"

solve_from_globs() is the same solve applied to image files already on disk
(e.g. pairs saved off the RUI image viewers) instead of live captures.
"""

from __future__ import annotations

import glob
import os

import numpy as np
import cv2

from stereo_library import DEFAULT_SETTINGS, compute_depth_map, colorize_depth


# Chessboard defaults. cols/rows are INNER CORNER counts (a board of 10x7
# squares has 9x6 inner corners), square_mm is the printed square size.
DEFAULT_BOARD_COLS = 9
DEFAULT_BOARD_ROWS = 6
DEFAULT_SQUARE_MM = 25.0

# Corner detection tuning.
_FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Fewer pairs than this and the solve is under-constrained / garbage.
MIN_PAIRS = 5
# Rectified rows must line up this well or block matching will produce mush.
GOOD_EPIPOLAR_RMS_PX = 1.0

# Where calibration files live on a NEPI device: the user config tier on the
# storage mount, 'cals' subfolder. That is the standard NEPI camera-calibration
# location -- system_mgr creates user_cfg/cals on boot, and the ZED driver backs
# its factory cal files up to exactly this folder
# (idx_zed_node.ZedCamNode.CAL_BACKUP_PATH), so a calibration written here
# survives a software update along with the rest of the user config.
USER_CFG_PATH = "/mnt/nepi_storage/user_cfg"
CAL_PATH = USER_CFG_PATH + "/cals"
CALIB_FILENAME = "stereo_calib.npz"


def default_calib_folder():
    """First writable calibration folder: env override, device cals, home.

    The home fallback keeps this usable on a dev box, where the device storage
    mount does not exist.
    """
    candidates = [
        os.environ.get("NEPI_STEREO_CALIB_DIR"),
        CAL_PATH,
        os.path.join(os.path.expanduser("~"), ".nepi", "cals"),
    ]
    for folder in candidates:
        if not folder:
            continue
        # Only claim a folder we can actually create/write into.
        if os.path.isdir(folder) and os.access(folder, os.W_OK):
            return folder
        parent = os.path.dirname(os.path.normpath(folder))
        if os.path.isdir(parent) and os.access(parent, os.W_OK):
            return folder
    return os.path.join(os.path.expanduser("~"), "cals")


def default_calib_path():
    return os.path.join(default_calib_folder(), CALIB_FILENAME)


# Shared helpers
def _as_grayscale(image):
    if image is not None and image.ndim == 3:
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image


def _as_bgr(image):
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def board_object_points(cols, rows, square_mm):
    """One board's 3D corner coords, z=0, in mm (so T/baseline come out in mm)."""
    objp = np.zeros((rows * cols, 3), np.float32)
    # Matches findChessboardCorners ordering: cols corners per row, row-major.
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_mm)
    return objp


def _canonical_order(corners):
    """Force a deterministic corner order.

    findChessboardCorners can return a board 180-degree rotated depending on
    which end it started scanning from. Left and right frames disagreeing on
    that would pair corner i in one image with the opposite physical corner in
    the other -- a silently wrong calibration. Anchoring index 0 to the
    top-left-most corner makes both images agree. (A per-view 180 flip that
    BOTH images share is harmless: it is just a different board pose.)
    """
    pts = corners.reshape(-1, 2)
    if (pts[0, 0] + pts[0, 1]) > (pts[-1, 0] + pts[-1, 1]):
        return corners[::-1].copy()
    return corners


def find_board(image, cols, rows):
    """Locate + refine chessboard corners. Returns (found, corners)."""
    gray = _as_grayscale(image)
    if gray is None:
        return False, None
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=_FIND_FLAGS)
    if not found:
        return False, None
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _SUBPIX_CRITERIA)
    return True, _canonical_order(corners)


def _epipolar_rms_px(imgpoints_l, imgpoints_r, KL, DL, KR, DR, R1, R2, P1, P2):
    """RMS row (y) mismatch of the captured corners AFTER rectification.

    This is the number that actually says whether block matching will work:
    rectification puts matching features on the same image row, so this should
    land well under a pixel. A big value means the calibration is bad no matter
    how good the reprojection RMS looked.
    """
    diffs = []
    for corners_l, corners_r in zip(imgpoints_l, imgpoints_r):
        rect_l = cv2.undistortPoints(corners_l, KL, DL, R=R1, P=P1).reshape(-1, 2)
        rect_r = cv2.undistortPoints(corners_r, KR, DR, R=R2, P=P2).reshape(-1, 2)
        diffs.append(rect_l[:, 1] - rect_r[:, 1])
    if not diffs:
        return float("nan")
    return float(np.sqrt(np.mean(np.concatenate(diffs) ** 2)))


# Stage 1: the solve (shared by the RUI capture flow and the on-disk flow)
def solve_stereo(objpoints, imgpoints_l, imgpoints_r, image_size, out_path, alpha=0.0):
    """Solve a stereo pair from matched corner sets and save the .npz.

    Produces exactly what Rectifier() loads: map1x/1y, map2x/2y, Q, and the
    scalars focal_length_px + baseline_mm.

    image_size is (w, h). alpha is the stereoRectify zoom: 0 crops to
    all-valid pixels, 1 keeps every source pixel (with black wedges).
    Returns a diagnostics dict.
    """
    pairs = len(objpoints)
    if pairs < MIN_PAIRS:
        raise ValueError(
            f"Only {pairs} usable pair(s); need at least {MIN_PAIRS}. Capture "
            "more views with the board at varied distances, angles and image "
            "corners."
        )

    rms_l, KL, DL, _, _ = cv2.calibrateCamera(objpoints, imgpoints_l, image_size, None, None)
    rms_r, KR, DR, _, _ = cv2.calibrateCamera(objpoints, imgpoints_r, image_size, None, None)

    # Intrinsics are already solved above, so only R/T are estimated here.
    rms_s, KL, DL, KR, DR, R, T, _, _ = cv2.stereoCalibrate(
        objpoints, imgpoints_l, imgpoints_r,
        KL, DL, KR, DR,
        image_size,
        flags=cv2.CALIB_FIX_INTRINSIC
    )

    # rectification transforms:
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        KL, DL, KR, DR, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha
    )

    # precompute the remap lookup tables (what makes per-frame rectify cheap):
    map1x, map1y = cv2.initUndistortRectifyMap(KL, DL, R1, P1, image_size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(KR, DR, R2, P2, image_size, cv2.CV_32FC1)

    # Geometry for stereo_library. NOTE: depth is computed on RECTIFIED images,
    # so the focal length that matters is the RECTIFIED one (P1), which alpha
    # rescales -- not the raw KL fx.
    #   P2 = [fx, 0, cx, -fx*baseline; ...]  ->  baseline = |P2[0,3]| / fx
    focal_length_px = float(P1[0, 0])
    baseline_mm = float(abs(P2[0, 3]) / P1[0, 0])
    epipolar_rms = _epipolar_rms_px(imgpoints_l, imgpoints_r,
                                    KL, DL, KR, DR, R1, R2, P1, P2)

    folder = os.path.dirname(os.path.abspath(out_path))
    if folder and not os.path.isdir(folder):
        os.makedirs(folder, exist_ok=True)
    np.savez(out_path,
             map1x=map1x, map1y=map1y, map2x=map2x, map2y=map2y,
             Q=Q, focal_length_px=focal_length_px, baseline_mm=baseline_mm,
             # Kept so the maps can be rebuilt at another alpha, and for debug.
             KL=KL, DL=DL, KR=KR, DR=DR, R=R, T=T,
             R1=R1, R2=R2, P1=P1, P2=P2,
             roi1=np.array(roi1), roi2=np.array(roi2),
             image_size=np.array(image_size))

    return {
        "pairs_used": pairs,
        "image_size": tuple(image_size),
        "rms_left": float(rms_l),
        "rms_right": float(rms_r),
        "rms_stereo": float(rms_s),
        "focal_length_px": focal_length_px,
        "baseline_mm": baseline_mm,
        "epipolar_rms_px": epipolar_rms,
        "good": bool(epipolar_rms < GOOD_EPIPOLAR_RMS_PX),
        "out_path": out_path,
    }


class StereoCalibrator:
    """Live capture/solve flow the app node drives from the RUI.

    Usage from the node:
        cal = StereoCalibrator(cols, rows, square_mm)
        ok, message = cal.capture(left_bgr, right_bgr)   # per Capture press
        ok, message, info = cal.solve(out_path)          # per Solve press
    Every call returns a human-readable message; the node forwards it to the
    RUI as status_msg.calib_message so the operator sees what happened.
    """

    def __init__(self, cols=DEFAULT_BOARD_COLS, rows=DEFAULT_BOARD_ROWS,
                 square_mm=DEFAULT_SQUARE_MM):
        self.cols = int(cols)
        self.rows = int(rows)
        self.square_mm = float(square_mm)
        self.clear()

    #### board geometry
    def set_board(self, cols=None, rows=None, square_mm=None):
        """Change the board description. Captures are dropped if the corner
        counts change (old captures no longer describe this board)."""
        cols = self.cols if cols is None else int(cols)
        rows = self.rows if rows is None else int(rows)
        square_mm = self.square_mm if square_mm is None else float(square_mm)
        if cols < 3 or rows < 3:
            return False, f"board must be at least 3x3 inner corners, got {cols}x{rows}"
        if square_mm <= 0.0:
            return False, f"square_mm must be > 0, got {square_mm}"
        shape_changed = (cols != self.cols or rows != self.rows)
        self.cols, self.rows, self.square_mm = cols, rows, square_mm
        if shape_changed and self.count:
            self.clear()
            return True, (f"board now {cols}x{rows} corners @ {square_mm:g} mm "
                          "-- captures cleared")
        return True, f"board {cols}x{rows} corners @ {square_mm:g} mm"

    #### captures
    def clear(self):
        self._objpoints = []
        self._imgpoints_l = []
        self._imgpoints_r = []
        self.image_size = None
        return True, "captures cleared"

    @property
    def count(self):
        return len(self._objpoints)

    def capture(self, left_image, right_image):
        """Find the board in a live L/R pair and keep it. Returns (ok, message)."""
        if left_image is None or right_image is None:
            return False, "no camera frames -- select both cameras first"
        if left_image.shape[:2] != right_image.shape[:2]:
            return False, (f"L/R resolution mismatch: {left_image.shape[:2]} vs "
                           f"{right_image.shape[:2]}")

        size = (left_image.shape[1], left_image.shape[0])   # (w, h)
        if self.image_size is not None and size != self.image_size:
            # Mixed resolutions would silently produce nonsense intrinsics.
            return False, (f"frame size {size} != captured {self.image_size}; "
                           "clear captures before changing resolution")

        found_l, corners_l = find_board(left_image, self.cols, self.rows)
        found_r, corners_r = find_board(right_image, self.cols, self.rows)
        if not (found_l and found_r):
            if not found_l and not found_r:
                where = "either image"
            else:
                where = "the right image" if found_l else "the left image"
            return False, (f"{self.cols}x{self.rows} board not found in {where} "
                           f"-- kept {self.count}")

        self._objpoints.append(board_object_points(self.cols, self.rows, self.square_mm))
        self._imgpoints_l.append(corners_l)
        self._imgpoints_r.append(corners_r)
        self.image_size = size
        remaining = MIN_PAIRS - self.count
        hint = f" (need {remaining} more)" if remaining > 0 else " (ready to solve)"
        return True, f"captured pair {self.count}{hint}"

    #### solve
    def solve(self, out_path, alpha=0.0):
        """Solve + save. Returns (ok, message, info_dict_or_None)."""
        try:
            info = solve_stereo(self._objpoints, self._imgpoints_l, self._imgpoints_r,
                                self.image_size, out_path, alpha=alpha)
        except (ValueError, cv2.error) as exc:
            return False, f"calibration failed: {str(exc).splitlines()[-1]}", None
        quality = "good" if info["good"] else "TOO HIGH -- recapture"
        message = (f"solved {info['pairs_used']} pairs: "
                   f"rms {info['rms_stereo']:.3f} px, "
                   f"epipolar {info['epipolar_rms_px']:.3f} px ({quality}), "
                   f"f {info['focal_length_px']:.1f} px, "
                   f"baseline {info['baseline_mm']:.1f} mm")
        return True, message, info


def solve_from_globs(left_glob, right_glob, cols, rows, square_mm, out_path, alpha=0.0):
    """Same solve, from image PAIRS ALREADY ON DISK (sorted glob order).

    Name the files so a sort pairs them (left_000.png / right_000.png).
    Returns (info_dict, [(path, reason), ...] for skipped pairs).
    """
    left_paths = sorted(glob.glob(left_glob))
    right_paths = sorted(glob.glob(right_glob))
    if not left_paths:
        raise ValueError(f"No images matched left glob: {left_glob!r}")
    if len(left_paths) != len(right_paths):
        raise ValueError(f"Left and right image counts do not match "
                         f"({len(left_paths)} vs {len(right_paths)}).")

    cal = StereoCalibrator(cols, rows, square_mm)
    skipped = []
    for left_path, right_path in zip(left_paths, right_paths):
        left_img = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        right_img = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
        if left_img is None or right_img is None:
            skipped.append((left_path, "unreadable"))
            continue
        ok, message = cal.capture(left_img, right_img)
        if not ok:
            skipped.append((left_path, message))
    info = solve_stereo(cal._objpoints, cal._imgpoints_l, cal._imgpoints_r,
                        cal.image_size, out_path, alpha=alpha)
    return info, skipped


# Stage 2: per-frame rectification
class Rectifier:
    """Applies saved rectification maps to raw L/R frames."""

    def __init__(self, calib_path):
        data = np.load(calib_path)
        self.map1x, self.map1y = data["map1x"], data["map1y"]
        self.map2x, self.map2y = data["map2x"], data["map2y"]
        self.Q = data["Q"]
        # These two feed stereo_library settings so depth uses true geometry.
        self.focal_length_px = float(data["focal_length_px"])
        self.baseline_mm = float(data["baseline_mm"])
        # (w, h) the maps were built for; raw frames must match this.
        self.image_size = (int(self.map1x.shape[1]), int(self.map1x.shape[0]))
        self.calib_path = calib_path

    def matches(self, image):
        """True if this frame is the resolution the maps were built for."""
        if image is None:
            return False
        return (image.shape[1], image.shape[0]) == self.image_size

    def rectify(self, left_image, right_image):
        """Return (rect_left, rect_right), ready for compute_depth_map()."""
        left = cv2.remap(left_image, self.map1x, self.map1y, cv2.INTER_LINEAR)
        right = cv2.remap(right_image, self.map2x, self.map2y, cv2.INTER_LINEAR)
        return left, right

    def settings_overrides(self):
        """Geometry overrides to merge into a stereo_library settings dict."""
        return {
            "focal_length_px": self.focal_length_px,
            "baseline_mm": self.baseline_mm,
        }


def epipolar_overlay(rect_left, rect_right, line_spacing=40):
    """Side-by-side rectified pair with horizontal rules drawn across both.

    Eyeball check: the same feature should sit on the same rule in both halves.
    If it does not, the calibration is wrong and depth will be mush.
    """
    canvas = np.hstack([_as_bgr(rect_left), _as_bgr(rect_right)])
    for y in range(0, canvas.shape[0], line_spacing):
        cv2.line(canvas, (0, y), (canvas.shape[1], y), (0, 0, 255), 1)
    split = rect_left.shape[1]
    cv2.line(canvas, (split, 0), (split, canvas.shape[0]), (0, 255, 0), 2)
    return canvas


# Stage 3: OPTIONAL offline block-match tuning (developer workstation only)
WINDOW = "tuner"

# Each entry: (slider_name, initial_value, max_value)
TRACKBARS = [
    ("matcher_sgbm",          1,   1),   # 0 = bm (fast/noisy), 1 = sgbm
    ("min_disparity",         0,  64),
    ("num_disparities",     128, 512),   # snapped to a multiple of 16
    ("block_size",            5,  31),   # forced odd
    ("uniqueness_ratio",     10,  50),
    ("speckle_window_size", 100, 400),
    ("speckle_range",         2, 100),
    ("disp12_max_diff",       2,  26),   # slider 0 means -1 (see sanitize)
    ("pre_filter_cap",       63,  63),
    # medianBlur on float32 only supports ksize 3 or 5, so cap at 5 (0 = off).
    ("median_filter_size",    5,   5),
]

# Keys the tuner drives; what gets printed at the end.
TUNED_KEYS = ("matcher", "min_disparity", "num_disparities", "block_size",
              "uniqueness_ratio", "speckle_window_size", "speckle_range",
              "disp12_max_diff", "pre_filter_cap", "median_filter_size")


def build_settings_from_sliders(base=None):
    """Read every trackbar, sanitize, and return a settings dict.

    base holds values the sliders do not own (the calibrated focal_length_px /
    baseline_mm), applied on top of DEFAULT_SETTINGS.
    """
    def pos(name):
        return cv2.getTrackbarPos(name, WINDOW)

    matcher = "sgbm" if pos("matcher_sgbm") else "bm"
    min_disparity = pos("min_disparity")
    num_disparities = pos("num_disparities")
    block_size = pos("block_size")
    uniqueness_ratio = pos("uniqueness_ratio")
    speckle_window_size = pos("speckle_window_size")
    speckle_range = pos("speckle_range")
    disp12_max_diff = pos("disp12_max_diff")
    pre_filter_cap = pos("pre_filter_cap")
    median_filter_size = pos("median_filter_size")

    # sanitize (trackbars are int and >= 0):
    num_disparities = max(16, (num_disparities // 16) * 16)  # positive multiple of 16
    block_size = max(3, block_size | 1)                      # force odd, >= 3
    if matcher == "bm":
        block_size = max(5, block_size)                      # BM rejects 3
    disp12_max_diff = disp12_max_diff - 1                    # lets you reach -1
    pre_filter_cap = max(1, pre_filter_cap)                  # 0 is rejected
    median_filter_size = 0 if median_filter_size < 3 else (median_filter_size | 1)

    return {
        **DEFAULT_SETTINGS,
        **(base or {}),
        "matcher": matcher,
        "convert_to_grayscale": True,
        "min_disparity": min_disparity,
        "num_disparities": num_disparities,
        "block_size": block_size,
        "uniqueness_ratio": uniqueness_ratio,
        "speckle_window_size": speckle_window_size,
        "speckle_range": speckle_range,
        "disp12_max_diff": disp12_max_diff,
        "pre_filter_cap": pre_filter_cap,
        "median_filter_size": median_filter_size,
    }


def format_settings(settings):
    """Render the tuned keys as a paste-able stereo_settings.py block."""
    return "\n".join(f"    {key!r}: {settings[key]!r}," for key in TUNED_KEYS)


def _banner(image, text):
    """Overlay a status line on a preview image (returns the same array)."""
    for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
        cv2.putText(image, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    color, thickness, cv2.LINE_AA)
    return image


def run_tuner(left_path, right_path, calib_path=None, max_width=1600):
    """Slider UI over compute_depth_map; q/ESC quits, s prints current settings.

    Developer workstation only (needs a cv2 GUI). On the device, tune from the
    RUI process-settings panel with the live depth viewer instead.
    """
    left = cv2.imread(left_path, cv2.IMREAD_COLOR)
    right = cv2.imread(right_path, cv2.IMREAD_COLOR)
    if left is None or right is None:
        raise ValueError(f"Could not read {left_path!r} / {right_path!r}")
    if left.shape[:2] != right.shape[:2]:
        raise ValueError(f"L/R resolution mismatch: {left.shape[:2]} vs {right.shape[:2]}")

    # Block matching needs RECTIFIED input.
    base = {}
    if calib_path:
        rectifier = Rectifier(calib_path)
        if not rectifier.matches(left):
            raise ValueError(f"Image size {(left.shape[1], left.shape[0])} does not "
                             f"match the calibration size {rectifier.image_size}.")
        left, right = rectifier.rectify(left, right)
        base = rectifier.settings_overrides()   # true focal/baseline -> real mm
        print(f"Rectified with {calib_path}: "
              f"focal_length_px={base['focal_length_px']:.3f} "
              f"baseline_mm={base['baseline_mm']:.3f}")
    else:
        print("WARNING: no calib_path. Inputs are assumed ALREADY RECTIFIED, and "
              "depth uses DEFAULT_SETTINGS geometry (not your cameras').")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    for name, initial, maximum in TRACKBARS:
        cv2.createTrackbar(name, WINDOW, min(initial, maximum), maximum, lambda x: None)
    print("q / ESC = quit, s = print current settings")

    last = None  # cache last settings so we only recompute when something changed
    settings = None
    while True:
        settings = build_settings_from_sliders(base)
        if settings != last:
            try:
                depth = compute_depth_map(left, right, settings)
                valid_mask = depth > 0
                valid = float(np.count_nonzero(valid_mask)) / depth.size
                near = depth[valid_mask].min() if valid else 0.0
                far = depth[valid_mask].max() if valid else 0.0
                view = np.hstack([_as_bgr(left), colorize_depth(depth, settings)])
                _banner(view, f"{settings['matcher']} nd={settings['num_disparities']} "
                              f"bs={settings['block_size']} | valid {valid * 100:.1f}% "
                              f"| {near / 1000:.2f}-{far / 1000:.2f} m")
            except (cv2.error, ValueError) as exc:
                view = _banner(np.hstack([_as_bgr(left), np.zeros_like(_as_bgr(left))]),
                               f"invalid params: {str(exc).splitlines()[-1][:90]}")
            if view.shape[1] > max_width:                     # keep the window sane
                scale = max_width / view.shape[1]
                view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            cv2.imshow(WINDOW, view)
            last = settings

        key = cv2.waitKey(50) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            print("current settings:\n" + format_settings(settings))

    cv2.destroyAllWindows()
    print("final settings (paste into stereo_settings.py):\n" + format_settings(settings))
    return settings
