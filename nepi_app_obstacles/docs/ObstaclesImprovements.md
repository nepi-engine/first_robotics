# Obstacle Detection — Measured Improvements

Five proposals, ordered by measured benefit per unit of risk. Two improve the
obstacle data, three improve speed. Every number below was measured against the
TUM `freiburg2_pioneer_360` set through an offline harness that calls the real
`process_results`; the harness is in the appendix so the measurements can be
re-run.

Unless stated otherwise: 104 frames, every 8th of the 830-frame sequence, at the
calibrated settings from `TUMPioneerData.md` (Principal Point 0.5080 / 0.5202,
Mount Pitch −6.0, source 63.1° × 49.5°). Timings are on an x86 development
machine with the geometry cache warm.

**Nothing here has been applied.** Where a prototype was built it was built in
the harness, and the fact is stated at the proposal.

Mount pitch calibration is out of scope — that is settled in `TUMPioneerData.md`
and it is a calibration matter, not a code one. These proposals all assume the
pitch is already right, and one of them is worth having precisely because it
stops assuming that.

---

## 1. The ground threshold should track range, because the floor does

**Effort: medium. Risk: medium. Category: data quality.**

### The mechanism

`process_results` compares every pixel's levelled height against one fixed
number, `ground_max_height_m`. That is correct only if a flat floor reads the
same height at every range. It does not. Measured across the sequence, the mean
height of a ground pixel drifts upward with range:

| range band | mean ground height | margin to the −0.30 threshold | band sd |
|---|---|---|---|
| 1–2 m | −0.5169 m | 0.217 m | 0.0132 |
| 2–3 m | −0.5109 m | 0.211 m | 0.0228 |
| 3–4 m | −0.4872 m | 0.187 m | 0.0328 |
| 4–6 m | −0.4450 m | 0.145 m | 0.0444 |
| 6–10 m | −0.3946 m | **0.097 m** | 0.0489 |

A per-frame regression of ground height on range gives **+0.0208 m/m** (sd
0.0077, median +0.0216, 104 frames). `TUMPioneerData.md` notes +0.013 m/m on one
frame; across the sequence it is +0.021.

Two things move in the wrong direction at once. **55% of the classification
margin is gone by 6–10 m**, and the scatter within a band grows **3.7×** over the
same span. Shrinking margin against growing noise is exactly what produces a
speckled band at the crossover instead of a clean edge — and it is why the far
field is where a small calibration error turns into a wall of false obstacles.

### What was tried first, and why it was rejected

The obvious fix is a per-frame RANSAC ground plane, and the plane is genuinely
there. Fitted in the levelled frame over the lowest 40% of valid returns, it
converges on every frame tested: residual tilt **0.272°** mean (max 1.588°),
offset **0.531 m** below the sensor (sd 0.013 m), **97.5%** inliers (min 88.9%).

Substituting plane distance for the fixed threshold reduces the drift from
+0.1223 m to +0.1144 m. That is a **6.5% reduction**, reclassifying 0.24% of
pixels at a median range of 5.96 m. It does not fix the problem.

The reason is worth keeping: **the reconstructed floor is not planar — it bends
upward with range**, and no plane fits a curve. The plane recovers the floor's
orientation, which is what the mount-pitch control already describes. It does
not recover the drift, because the drift is not an orientation error.

### The change

Fit the drift where it lives: height against **range**. A robust line, seeded on
the pixels the current threshold already calls ground, then refit and
reclassified twice more:

```
fitFloorLine(np_ranged, np_height, ground_max_height_m) -> (slope, intercept, offset)
```

A new private helper in `sdk/nepi_obstacles.py`, camelCase and no docstring per
the package convention, called from `process_results` between `getHeightMap`
(line 352) and the ground/obstacle split (line 362). The split then compares
against `slope * np_ranged + intercept + offset` instead of a scalar. The offset
is chosen so the threshold gives the same clearance at the near band that the
fixed value gives today, which is what keeps `ground_max_height_m` meaning what
`ObstaclesControls.md` says it means.

It needs a control to enable it — a `Toggle`, defaulting off until it has run on
hardware. `PROCESS_CONTROLS_DICT` is the place, and note the warning already in
that file: an out-of-bounds default does not clamp, it makes the control vanish.

### The gain

Measured in the harness, not in the app:

- Range drift **+0.1223 m → +0.0226 m, an 82% reduction**.
- Fitted slope +0.0279 m/m (sd 0.0105).
- 1.85% of pixels reclassified; ground share **+1.63 percentage points**.
- **All five `TUMPioneerData.md` anchors preserved**: 398 → 4 components
  including the pillar at 1.77 m; 788 → 2 including 1.74 m; 342 → 4 including
  2.18 m; 469 and 424 → no obstacles.

Where it earns its keep is robustness to a pitch fit that is not perfect. TUM
does not publish the mount angle and `−6.0` is an empirical fit; a real vehicle's
mount angle is measured with a protractor. Far-band (4–10 m) true-floor pixels
called obstacle, against a floor truth frozen at the calibrated pitch and
independent of anything being evaluated:

| pitch error | current | range-tracking |
|---|---|---|
| 0° | 4.5% | **0.0%** |
| 1° | 25.7% | **0.0%** |
| 2° | 64.6% | **0.0%** |
| 3° | 93.6% | 2.5% |
| 4° | 99.7% | 7.5% |
| 6° | 100.0% | 30.0% |

`ObstaclesControls.md` says one degree of unmodelled tilt is enough to start the
floor band. With this, two degrees is not.

### The risk

The fit can be captured by a scene with no floor in it — a sensor facing a wall
at close range, a vehicle nose-down over an edge. The seed set is the pixels the
fixed threshold already calls ground, so a frame with no ground has nothing to
seed from and must fall back to the fixed threshold rather than fit noise. The
existing guard shape is the right one: return None below a pixel count and let
the caller use the scalar.

It also makes an operator-visible number implicit. `ground_max_height_m` stops
being the whole answer, and a support conversation that currently ends at "check
your ground height" gains a step. That is what the Toggle is for, and why the
control's description has to say plainly that it is on.

The 82% figure is one dataset, one sensor. A Kinect's depth bias is not a ZED's.
The mechanism — a fitted per-frame correction rather than an assumed constant —
is general; the size of the correction is not, which is the argument for fitting
it per frame rather than shipping a constant.

---

## 2. Two byte-identical speed changes worth 26% of `process_results`

**Landed 2026-09-18 in `c341229`.**

**Effort: low. Risk: low. Category: speed.**

### The mechanism

Profiled stage by stage, with every intermediate built fresh per frame and freed
— which is what the app does — `process_results` costs 11.4–11.6 ms/frame:

| stage | ms | share |
|---|---|---|
| **getRangeEdgeMask** | **3.32** | **37.2%** |
| buildObstacleDict ×≤10 | 1.35 | 15.1% |
| 2× segmentation map | 1.02 | 11.4% |
| getHeightMap | 0.91 | 10.2% |
| getComponents | 0.73 | 8.2% |
| getNoReturnRange | 0.61 | 6.8% |
| range gate | 0.46 | 5.2% |
| ground/obstacle split | 0.28 | 3.2% |
| getGeometry (cache hit) | 0.01 | 0.1% |
| staged total | 8.93 | |

The 2.5 ms between the staged total and the whole function is `matchPrevious`,
`recordCycle` and dict building. It is not the control reads — 13
`getControlValue` calls cost 0.005 ms/frame, which is nothing.

**`getRangeEdgeMask` is over a third of the cost, and it computes edges
everywhere.** Its result is consumed exactly once, as
`obstacle_mask & ~edge` (line 394). An edge on a pixel outside the obstacle mask
is never read. Every 8-neighbour of an obstacle pixel lies inside the obstacle
bounding box grown by one, so the four diff passes only ever need to run on that
box — which covers a median 43% of the frame on this data (p90 52%).

**The two segmentation maps allocate a full NaN raster and scatter into it**
(lines 370 and 373). `np.full` then `a[mask] = f[mask]` is five passes and two
temporaries: allocate, fill with NaN, build an index array from the mask, gather
the members, scatter them back. `np.where` is one vectorised select.

### The change

In `sdk/nepi_obstacles.py`:

- A new private `getSegmentMask(np_ranged, obstacle_mask, range_step_m)` that
  derives the obstacle bounding box from two boolean reductions
  (`obstacle_mask.any(axis=1)` / `any(axis=0)` plus `np.argmax`), calls the
  existing `getRangeEdgeMask` on that slice, and writes the result back into a
  zeroed full-frame mask. Replaces line 394. `getRangeEdgeMask` itself is
  unchanged — it is called with a smaller array.
- Lines 370 and 373 become `np.where(mask, np_depth_map, nan)`.

The `any()` reduction is load-bearing. `np.nonzero` for the same bounding box
measured **2.35 ms** — slower than the 3.28 ms stage it was meant to save, because
it materialises two index arrays over the whole mask.

### The gain

Measured in the harness, not in the app:

| change | before | after | identical output? |
|---|---|---|---|
| edge mask on the obstacle bbox | 3.28 ms | **0.79 ms** | yes, 104/104 frames |
| segmentation maps via `np.where` | 1.16 ms | **0.07 ms** | yes, 104/104 frames |
| **combined, whole function** | **11.63 ms** | **8.61 ms (−26%)** | **0 mismatches** |

The identity check compared every `Obstacle` field (`xmin_pixel` … `range_m`,
`azimuth_deg`, `elevation_deg`, `confidence`, `area_ratio`) and both
segmentation rasters element by element, on all 104 frames. Zero differences.
This is a pure cost change, not a behaviour change.

### The risk

Low, and bounded by that identity check. The bounding-box argument is exact
rather than approximate: the mask is zero outside the box by construction, and
inside the box the shipped helper runs on the same values. The one thing that
would break it is a future consumer of the edge mask that reads it somewhere
other than through `obstacle_mask &`, which is why the reduction belongs in a
named helper with the reason in a comment rather than inline at the call site.

---

## 3. Report the nearest return, not the median

**Effort: low. Risk: low. Category: data quality.**

### The mechanism

`buildObstacleDict` reports `range_m` as the **median** of the component's member
ranges, with a good reason recorded at the call site: a depth-map edge pixel
straddling the obstacle and the background reads as a far outlier, and the median
keeps one such pixel from pushing the reported range past the obstacle.

That reasoning is sound and the wrong conclusion follows from it. A median is the
right defence against outliers and the wrong statistic for an obstacle, because a
reported component is not a point. Measured over the 173 reported components:

- Range span inside one component (p95 − p05): median **0.75 m**, mean 1.15 m,
  p90 **2.96 m**, max 4.36 m.
- Reported range minus the component's own nearest return: median **0.62 m**,
  mean 0.83 m, p90 1.99 m, max **2.95 m**.
- **64%** of components report a range more than 0.5 m beyond their nearest
  return; **31%** report more than 1.0 m beyond.

20% of reported components cover more than 20% of the frame, so these are not
edge cases — they are the wall sections and large structures the app is built to
find. For anything that has to decide whether to stop, the distance to the
nearest part of an obstacle is the number that matters, and the app reports one
that is over a metre too far in a third of cases.

### The change

In `buildObstacleDict` (`sdk/nepi_obstacles.py`), replace the `np.median(ranges)`
call with a single `np.partition` that yields both the 5th percentile and the
median, and report the 5th percentile as `range_m`. The percentile, not the
minimum: the minimum is exactly the single straddling edge pixel the median was
chosen to reject, and at a 6,144-pixel floor a 5% tail is ~300 pixels — far more
than any outlier population.

`Obstacle.msg` has one range field, so this changes the meaning of `range_m`
rather than adding a field. That is the right call — a consumer steering around
an obstacle wants the near face, and nothing in the engine currently consumes the
median in a way that depends on it being a median. If a second number is wanted
later, `range_m` plus a `range_far_m` is the shape, and both fall out of the same
partition.

### The gain

Measured in the harness: the reported range moves **0.33 m nearer (median)**,
0.54 m mean, 1.30 m at p90, 2.30 m at worst, and stops being behind the obstacle
in the 36% of components where the median-to-p05 gap exceeds 0.5 m.

It is also **faster**. `np.partition` for both values costs 0.2855 ms per
component against 0.3318 ms for `np.median` alone — **−0.46 ms/frame** at 10
components, because the median's full sort is more work than two partial ones.

### The risk

It is a semantic change to a published field, so anything already tuned against
the median reads nearer after it. On this data the shift is 0.33 m median. It
belongs in `ObstaclesControls.md` and in the `Obstacle.msg` comment, and it is
the one proposal here that a downstream consumer could notice without being
told — which is an argument for doing it early rather than late.

---

## 4. Size the minimum obstacle in metres, not in pixels

**Effort: medium. Risk: medium. Category: data quality.**

### The mechanism

`min_obstacle_size_ratio` is a fraction of the depth map area, so at the 0.02
default a component must hold 6,144 pixels on 640×480. A pixel subtends a fixed
**angle**, so the physical size of 6,144 pixels is a function of range:

| range | physical area of a 6,144 px component | side of an equivalent square |
|---|---|---|
| 1 m | 0.023 m² | 0.15 m |
| 2 m | 0.091 m² | 0.30 m |
| 4 m | 0.362 m² | 0.60 m |
| 8 m | 1.449 m² | 1.20 m |
| 10 m | **2.264 m²** | **1.50 m** |

One control value means two different things at the two ends of the default range
gate — a 98× swing in the size of object it dismisses.

Measured against a fixed-physical-area rule anchored at 2 m (0.091 m², the same
threshold where the two agree): the pixel rule reports **173** components, the
physical rule **343**. The 170 the pixel rule drops are all at range ≥ 2.41 m
(median 6.71 m), and include a **1.37 m² object at 9.43 m** and a 1.23 m² object
at 8.77 m.

**Zero components go the other way.** The pixel rule never admits something
physically small. The error is entirely one-directional: it is a far-field blind
spot, not a noise filter that occasionally overreaches.

The derived geometry is accurate enough to price this: the focal lengths the app
recovers from the reported field of view are fx 521.2 / fy 520.6 px against the
TUM calibration's 520.9 / 521.0 — 0.06%.

### The change

In `getComponents` (`sdk/nepi_obstacles.py`), the `area_pixels < min_pixels` test
becomes a physical-area test using the component's own median range and the focal
lengths derivable from the frame size and field of view, both already available
in `getGeometry`. The threshold is best expressed as the physical area the
existing ratio implies at a declared reference range, so an operator who never
touches it gets today's behaviour at that range.

`getComponents` currently takes `min_pixels` and nothing else; it would need the
range array and the geometry, or the size test moves up into `process_results`
where both are in scope. The second is the smaller change and keeps
`getComponents` a pure labeller.

### The gain

**+170 components over 104 frames** — roughly a doubling of far-field detections,
all of them physically large enough to matter, with no measured increase in small
false positives.

### The risk

The highest of the five, and the reason it is fourth rather than second. The 170
new detections are real objects by the physical-area test, but that test is
itself the proposal — nothing here confirms frame by frame against the overlay
that all 170 are things a vehicle should stop for. `TUMPioneerData.md` already
records the same open question about the 44 empty frames at the corrected
settings. This needs eyes on the overlay before it ships.

It also weakens a load-bearing property. `ObstaclesControls.md` says
`min_obstacle_size_ratio` "is also what absorbs sub-degree geometry errors: a
residual band too small to form a qualifying component never becomes a
detection." A range-scaled threshold is more permissive in exactly the far field
where the residual bias of proposal 1 lives. **These two should land together, or
this one should wait** — the whole argument for lowering the far-field size bar
is that the far-field classification can be trusted, and today it cannot.

---

## 5. Stop paying for the segmentation rasters nobody is watching

**Landed 2026-09-18 in `feac413`.**

**Effort: low. Risk: low. Category: speed.**

### The mechanism

`publishDepthMapCb` guards its two 32FC1 encodes on
`pub_has_subscribers('obstacles_depth_map_pub')`, with a comment saying that no
subscriber means the encodes are skipped outright.

The guard never fires. `obstacles_app_img_pub_node.py` registers
`obstacles_depth_map_sub` **unconditionally in its `SUBS_DICT` at construction**,
and `ObstaclesIF.__init__` launches that node. So there is always a subscriber
for as long as the app is enabled, and the parent encodes and publishes two
1.23 MB float32 rasters on every cycle whether or not any overlay or segmentation
image has a consumer — **24.6 MB/s on the wire at 10 Hz, each way**, plus a
re-decode at the far end in `getDepthMapFromMsg`.

The img pub node already knows the answer. `processObstaclesImage` computes
`needs_overlay` and `needs_segments` per product on every render and returns
early when all three are false — after the maps have crossed the transport.

### The change

Two halves, either of which helps:

- In `obstacles_app_img_pub_node.py`, register and unregister
  `obstacles_depth_map_sub` in step with the `needsImgCheck` state the node
  already tracks, rather than at construction. `NodeClassIF` exposes
  `unregister_subs`, and `unsubscribeImgTopic` already uses that shape.
- In `obstacles_if.py`, skip `queueDepthMapData` entirely when imaging is
  disabled, so the slot is not written on a cycle whose product cannot be wanted.

### The gain

Serialization itself is small — 0.057 ms per raster, measured. The gain is the
**24.6 MB/s of transport in each direction and the matching decode**, removed on
every cycle where nothing is rendering. That part is reasoned from reading the
code and from the message sizes, **not measured on the device**, and it is the
honest limit of what can be said from a development machine.

### The risk

Low in cost, moderate in fiddliness. Late subscription means the first frame or
two after an operator opens the overlay arrives without segmentation, and the
render path already handles that — `obstaclesDepthMapCb`'s maps persist until the
next pair arrives, so the overlay draws the last maps it had rather than
flickering off. The failure mode to avoid is a subscribe/unsubscribe oscillation
at the `needsImgCheck` boundary, which wants hysteresis or a hold-down.

---

## Considered and rejected

Each of these was measured. The number is why it is not a proposal.

**Speckle filtering (erosion, or a minimum-width test).** Not a real problem.
**94.1%** of obstacle-mask pixels survive a 3×3 erosion, and only **0.6%**
(44,134 of 7,251,682) have a ground pixel anywhere in their 3×3 neighbourhood.
The component histogram agrees: 1,017 of 1,190 components fall below
`min_pixels`, and together they hold **5.4%** of segment pixels — already
discarded by the existing size test. Adding a morphological pass would cost a
full-frame operation to remove what is already removed.

**A per-frame RANSAC ground plane.** The plane is real — tilt 0.272°, offset
0.531 m, 97.5% inliers — and it fixes **6.5%** of the range drift, reclassifying
0.24% of pixels. The floor bends rather than tilts, and a plane cannot fit a
curve. Proposal 1 is what remains after this was measured and discarded.

**Range-dependent or otherwise smarter `range_step_m`.** The premise does not
hold on this data. Only **0.21%** of ground pixels — a continuous surface, so
every one is a false cut — are flagged as a range edge. The floor's vertical
neighbour gradient is median 0.0089 m and 99.9th percentile 0.366 m against the
0.50 m threshold. More decisive: both pillar anchors are **identical at every
`range_step_m` from 0.15 m to 2.0 m** ([5.13, 5.36, 4.40, 1.77] throughout). The
control has no sensitivity inside its useful band; only on-versus-off matters
(173 components at 0.5 m against 141 with the test effectively disabled, 29/104
frames differing). There is nothing to tune.

**Rewriting `getRangeEdgeMask` with OpenCV.** `cv2.absdiff` + `cv2.compare` +
`cv2.bitwise_or` for the identical pairwise test measured **3.40 ms** against the
shipped 3.28 ms — no gain, and a dependency on OpenCV's dtype handling in a
function whose NaN-comparison semantics are load-bearing. A 3×3 morphological
max−min spread (`cv2.dilate` − `cv2.erode`) is faster at 4.03 ms in the same
harness pass but is a **superset** of the pairwise test — it differs on 92/104
frames and 14,703 pixels. Not equivalent, and not proposed.

**`np.nonzero` for the obstacle bounding box.** The right idea with the wrong
primitive: **2.35 ms**, slower than the 3.28 ms stage it was meant to save,
because it materialises two index arrays over the whole mask. The `any()`
reduction in proposal 2 is what makes the bounding box pay.

**Feeding `getRangeEdgeMask` an obstacle-masked range array.** Arguably more
correct — a step between an obstacle pixel and a ground pixel cannot fuse two
obstacles through a pixel that is not in the mask — but it measured **5.60 ms**
against 3.28 ms (the `np.where` to build the masked input costs more than the
edges it removes), changed the segment mask on 84/104 frames and 5,390 pixels,
and produced **exactly the same 173 components**. All cost, no benefit.

**Declaring `no_return_range_m` instead of detecting it.** Setting the control to
10.0 makes `getNoReturnRange` free — 0.61 ms → 0.00. It is a control change, not
a code change, it costs the per-source generality the detection exists for, and
`ObstaclesControls.md` and `TUMPioneerData.md` both already settle this in favour
of detection. Recorded only so the 0.61 ms is not rediscovered as an opportunity.

**Poll/source beating as the cause of the 6.68 Hz device rate.** Simulated
against the exact re-chain rule in `processObstaclesCb`
(`delay = 1/max_rate − cycle_time`, floored at 0.01, `obstacles_if.py:1572-1580`):
the achieved rate is **9.87–10.00 Hz** across process costs of 20–80 ms, with and
without timer jitter. The poll does not cost rate. It costs **latency** — see the
note below.

**The per-cycle `copy.deepcopy` calls.** `get_controls_dict` deep-copies the live
controls dict once per cycle at **0.142 ms**; `updateNextTopicCb` deep-copies
`sources_info_dict` on a 100 Hz chain at 0.051 ms/call — **5.1 ms/s** of CPU at
one source, 23.5 ms/s at four. Real, and too small to propose against an 11.6 ms
cycle. Worth knowing before anyone spends a day on it.

---

## A note on the device rate, and how to settle it

The Jetson reports 6.68 Hz against a 10 Hz setting and 0.195 s of process
latency. `process_results` does not explain that on its own: it is 11.6 ms here,
and even at a 4× ARM penalty that is 47 ms, or 21 Hz.

Two readings survive, and they cannot be separated from a development machine:

1. **Per-cycle work on the Jetson is ~140 ms**, so `delay_time` hits its 0.01
   floor and the period becomes `T + 0.01`. 1/0.15 = 6.67 Hz, which matches the
   reported 6.68 Hz almost exactly — but it implies a ~12× ARM penalty, which is
   high for this workload.
2. **The source only delivers 6.68 Hz** and the process loop is faithfully
   following it.

**The app already publishes the discriminator.** `getProcessStatus` sets
`max_process_rate = 1.0 / avg_process_time` and `avg_source_rate`
(`obstacles_if.py:1830-1841`). One look at the status message on the device
decides which reading is right, with no new instrumentation. Do that before
acting on either.

Independently of which it is: the poll **does** cost latency. Simulated against a
10 Hz source, mean slot-wait-plus-process is **100 ms at a 10 Hz
`max_process_rate_hz` setting and 67 ms at 30 Hz**, with the achieved output rate
unchanged at 10 Hz in both cases. `max_process_rate_hz` is a ceiling, not a
target; setting it above the source rate costs nothing and removes a third of the
handoff latency. That is a settings note, not a code change, and it belongs in
`ObstaclesControls.md`.

---

## Two things found along the way

**The frame tags in `TUMPioneerData.md` are not unique.** `398TzUTC` names four
frames in the sequence and `788TzUTC` names three, seconds apart. The regression
anchors are `D2011-07-28T18-13-20p398TzUTC` and `D2011-07-28T18-13-53p788TzUTC`;
matching on the short tag alone selects `18-13-27p788TzUTC`, which genuinely
reports no obstacles and reads as a regression failure. Anchors should be keyed
on the full stamp.

**A profiling trap.** Pre-computing every frame's intermediates and holding them
live (~150 MB for 104 frames) inflates the array stages by 2–3×:
`getRangeEdgeMask` measures **5.26 ms** under that working set and **3.32 ms**
when intermediates are built and freed per frame, as the app does. Every timing
in this document is the latter. A stage profile that holds its own inputs is
measuring the allocator.

---

## Appendix — the harness

`rospy` is not installed on a development machine, so `nepi_obstacles` cannot be
imported directly. This installs stub modules for the three `nepi_sdk` imports it
makes, then imports and calls the **real** `process_results`. It lives outside the
package on purpose — it is a measurement tool, not a shipped file.

Save as `obstacles_harness.py` anywhere, adjust `APP_DIR` and `DATA_DIR`, and run
it to reproduce the anchor check. The measurement scripts in this document are
short programs that import it; each is described at the proposal it supports.

```python
#!/usr/bin/env python3
"""Offline harness for nepi_obstacles.process_results."""

import math
import os
import sys
import time
import types

import numpy as np

APP_DIR = os.path.expanduser('~/first_robotics/nepi_app_obstacles')
DATA_DIR = os.path.expanduser('~/tum_pioneer_360')


def _install_stubs():
    pkg = types.ModuleType('nepi_sdk')
    pkg.__path__ = []

    mod_sdk = types.ModuleType('nepi_sdk.nepi_sdk')

    class logger(object):
        def __init__(self, log_name = ''):
            self.log_name = log_name

        def log_warn(self, msg, throttle_s = 0.0):
            sys.stderr.write('WARN ' + str(msg) + '\n')

        def log_info(self, msg, throttle_s = 0.0):
            sys.stderr.write('INFO ' + str(msg) + '\n')

    mod_sdk.logger = logger

    mod_utils = types.ModuleType('nepi_sdk.nepi_utils')
    mod_utils.get_time = time.time

    mod_controls = types.ModuleType('nepi_sdk.nepi_controls')

    def get_value(controls_dict, control_name, index = None):
        # The harness hands process_results a plain name -> value dict rather
        # than a live nepi_controls dict; every control this module reads is a
        # single scalar, so a dict lookup is the whole contract.
        if controls_dict is None:
            return None
        return controls_dict.get(control_name, None)

    mod_controls.get_value = get_value

    sys.modules['nepi_sdk'] = pkg
    sys.modules['nepi_sdk.nepi_sdk'] = mod_sdk
    sys.modules['nepi_sdk.nepi_utils'] = mod_utils
    sys.modules['nepi_sdk.nepi_controls'] = mod_controls
    pkg.nepi_sdk = mod_sdk
    pkg.nepi_utils = mod_utils
    pkg.nepi_controls = mod_controls


_install_stubs()
sys.path.insert(0, os.path.join(APP_DIR, 'sdk'))
import nepi_obstacles  # noqa: E402


STATUS_DICT = {
    'width_deg': 63.1,
    'height_deg': 49.5,
    'depth_map_topic': '/harness/tum_pioneer_360/depth_map',
}

NAVPOSE_DICT = {'has_orientation': False, 'roll_deg': 0.0, 'pitch_deg': 0.0}


def default_controls():
    """Return the control defaults straight out of PROCESS_CONTROLS_DICT."""
    controls = {}
    for name, spec in nepi_obstacles.PROCESS_CONTROLS_DICT.items():
        controls[name] = spec['default']
    return controls


def calibrated_controls():
    """Return the TUMPioneerData.md calibrated settings."""
    controls = default_controls()
    controls['principal_x_ratio'] = 0.5080
    controls['principal_y_ratio'] = 0.5202
    controls['mount_pitch_deg'] = -6.0
    return controls


def frame_paths(step = 1, limit = None):
    """Return depth map .npy paths in sequence order."""
    names = sorted(n for n in os.listdir(DATA_DIR) if n.endswith('-depth_map.npy'))
    paths = [os.path.join(DATA_DIR, n) for n in names[::step]]
    if limit is not None:
        paths = paths[:limit]
    return paths


def load_frame(path):
    """Load one depth map as float32 millimetres."""
    return np.load(path).astype(np.float32)


def run_frame(path, controls):
    """Run the real process_results on one frame."""
    np_depth_map = load_frame(path)
    data_dict = nepi_obstacles.init_process_data_dict()
    return nepi_obstacles.process_results(np_depth_map, STATUS_DICT, NAVPOSE_DICT,
                                          data_dict, controls)


def stage_masks(np_depth_map, controls):
    """Rebuild the pipeline's intermediate arrays for a frame.

    Mirrors process_results stage by stage using the module's own helpers, so
    the masks and heights measured here are the ones the app computes.
    """
    mm = nepi_obstacles.MM_PER_M
    np_ranged = np.asarray(np_depth_map, dtype = np.float32) / mm
    h, w = np_ranged.shape

    no_return = nepi_obstacles.getNoReturnRange(np_ranged, controls['no_return_range_m'])
    with np.errstate(invalid = 'ignore'):
        valid = ((np_ranged >= controls['min_range_m'])
                 & (np_ranged <= controls['max_range_m']))
        if no_return is not None:
            valid &= (np_ranged < no_return)
    np.copyto(np_ranged, np.float32(np.nan), where = np.logical_not(valid))

    geom = nepi_obstacles.getGeometry(w, h, STATUS_DICT['width_deg'], STATUS_DICT['height_deg'],
                                      controls['principal_x_ratio'], controls['principal_y_ratio'])
    roll, pitch = nepi_obstacles.getLevelAngles(NAVPOSE_DICT, controls['use_navpose'],
                                                controls['mount_roll_deg'],
                                                controls['mount_pitch_deg'])
    np_height = nepi_obstacles.getHeightMap(np_ranged, geom, roll, pitch)

    g_max = controls['ground_max_height_m']
    o_max = controls['obstacle_max_height_m']
    ground = valid & (np_height <= g_max)
    obstacle = valid & (np_height > g_max) & (np_height <= o_max)
    edge = nepi_obstacles.getRangeEdgeMask(np_ranged, controls['range_step_m'])
    segment = obstacle & np.logical_not(edge)

    return {
        'np_ranged': np_ranged,
        'np_height': np_height,
        'valid': valid,
        'ground': ground,
        'obstacle': obstacle,
        'edge': edge,
        'segment': segment,
        'geom': geom,
    }


def anchor_check():
    """Reproduce the TUMPioneerData.md regression anchors."""
    controls = calibrated_controls()
    out = []
    # The TzUTC tag alone is NOT unique -- 788TzUTC and 398TzUTC each name
    # several frames seconds apart in the sequence, so the anchors are keyed
    # on the full stamp here.
    for tag, expect in [('18-13-20p398TzUTC', 'pillar ~1.77 m, 4 components'),
                        ('18-13-53p788TzUTC', 'pillar ~1.74 m'),
                        ('18-14-14p342TzUTC', 'pillar ~2.18 m'),
                        ('469TzUTC', 'no obstacles'),
                        ('424TzUTC', 'no obstacles')]:
        path = [p for p in frame_paths() if tag in os.path.basename(p)]
        if not path:
            out.append((tag, 'MISSING', expect))
            continue
        obstacles, ground_map, obstacle_map, _ = run_frame(path[0], controls)
        masks = stage_masks(load_frame(path[0]), controls)
        ground_pct = 100.0 * float(masks['ground'].sum()) / float(masks['ground'].size)
        ranges = [round(o['range_m'], 2) for o in obstacles]
        out.append((tag, 'n=%d ranges=%s ground=%.1f%%' % (len(obstacles), ranges, ground_pct),
                    expect))
    return out


if __name__ == '__main__':
    print('frames found: %d' % len(frame_paths()))
    for tag, got, expect in anchor_check():
        print('%-18s %-58s expect: %s' % (tag, got, expect))
```

Expected output, and the gate every measurement in this document passed before it
was trusted:

```
frames found: 830
18-13-20p398TzUTC  n=4 ranges=[5.13, 5.36, 4.4, 1.77] ground=45.4%   expect: pillar ~1.77 m, 4 components
18-13-53p788TzUTC  n=2 ranges=[4.93, 1.74] ground=45.5%              expect: pillar ~1.74 m
18-14-14p342TzUTC  n=4 ranges=[5.36, 4.93, 5.51, 2.18] ground=46.0%  expect: pillar ~2.18 m
469TzUTC           n=0 ranges=[] ground=46.1%                        expect: no obstacles
424TzUTC           n=0 ranges=[] ground=44.8%                        expect: no obstacles
```

`ground=45.4%` on the 398 anchor is the figure `TUMPioneerData.md` records for
the whole 208-frame calibrated run, to the decimal.
