# Obstacle Detection Controls

Every control the obstacles process reads, in the order it appears in the RUI.
The authoritative definitions are `PROCESS_CONTROLS_DICT` in
`sdk/nepi_obstacles.py`; this document explains what each one does and how to
choose a value.

The pipeline runs in this order, and the controls are grouped below the same way:

```
depth map (mm)
  -> range gate            min_range_m, max_range_m, no_return_range_m
  -> per-pixel geometry    principal_x_ratio, principal_y_ratio
  -> gravity levelling     use_navpose, mount_pitch_deg, mount_roll_deg
  -> ground/obstacle split ground_max_height_m, obstacle_max_height_m
  -> grouping              range_step_m
  -> reporting             min_obstacle_size_ratio, max_obstacles
```

A control early in that list changes the meaning of every control after it. If
detections look wrong, work down the list in order rather than starting with the
one whose name matches the symptom.

---

## Range gate

### Min Range (m) — `min_range_m`
`Float`, default `0.5`, range `0.1` to `100.0`

Returns closer than this are discarded. Use it to reject the sensor's blind zone
and any part of the vehicle in the field of view.

### Max Range (m) — `max_range_m`
`Float`, default `10.0`, range `0.2` to `100.0`

Returns farther than this are discarded. Set it to the distance you actually
need to react to, not to the sensor's optimistic spec — range noise grows with
range, and every distant return still costs a threshold decision.

If `max_range_m` is set at or below `min_range_m` the module raises it to
`min_range_m + 0.1` rather than gating everything away.

### No Return Range (m) — `no_return_range_m`
`Float`, default `0.0`, range `0.0` to `200.0`

**Leave this at 0 unless you have a reason not to.**

Depth sensors do not report "no data" for a pixel that got no return. They clamp
it to the sensor's maximum range, which is indistinguishable from a real surface
at that distance. A stereo camera pointed at a window, a dark surface, or open
sky produces a large plateau of pixels all sitting at exactly one value.

Left uncorrected this is the single worst failure mode in the app: the plateau
passes the range gate as a wall of real returns, the height threshold cuts it
along one horizontal row, and you get a large phantom obstacle above that row and
phantom ground below it — in every frame, in the same place, regardless of scene.

- **`0` (default)** — detect it per frame. The frame maximum is taken as the
  clamp if it holds at least 2% of the frame, and everything at or above it is
  discarded. A clamp is the maximum by construction and no single real surface
  occupies one exact float value that heavily, so the two separate cleanly.
- **A positive value** — the clamp value is known; use it directly.
- **Above the sensor's maximum range** — disables the mechanism and keeps every
  return.

Detection has a 1e-4 relative tolerance, so a value just under the clamp is still
excluded.

---

## Camera geometry

These describe the sensor's optics. They are calibration facts, not tuning knobs
— set them once from the camera's intrinsics and leave them.

### Principal Point X — `principal_x_ratio`
### Principal Point Y — `principal_y_ratio`
`Float`, default `0.5`, range `0.0` to `1.0`

Where the optical axis actually pierces the image, as a fraction of frame width
and height. `0.5, 0.5` is the exact centre of the frame.

From a camera calibration reporting `cx` and `cy` in pixels:

```
principal_x_ratio = cx / image_width
principal_y_ratio = cy / image_height
```

Held as a fraction rather than in pixels so the value survives a resolution
change on the source.

This matters more than its size suggests. Elevation is measured from this row,
so an error here is a constant bias on **every** elevation in the frame, and
height is `range × sin(elevation)` — a bias that grows with range and lands
hardest near the horizon, which is exactly where the ground threshold is being
applied. A principal point 1% of the frame off centre is roughly half a degree,
and half a degree is enough to start flipping distant floor into obstacles.

If you have no calibration, leave both at `0.5`. That is an approximation, not a
neutral choice, but it is the best available one.

The field of view comes from the source's status message (`width_deg`,
`height_deg`), not from a control. The app falls back to 110° × 70° if the source
reports none.

---

## Gravity levelling

Height is measured against gravity, not against the sensor housing. These three
controls establish which way is down.

### Use NavPose — `use_navpose`
`Bool`, default `True`

Apply the connected NavPose roll and pitch. Has no effect on a source that
publishes no orientation — which includes every file-published source. When it
has no effect, the mount angles below are the only levelling information the app
has.

### Mount Pitch (deg) — `mount_pitch_deg`
`Float`, default `0.0`, range `-90.0` to `90.0`

The sensor's fixed pitch on its mount. **Positive is nose up, so a camera
tilted downward takes a negative value.**

This is the control most likely to be wrong, and its failure mode is specific and
recognisable: a ragged full-width band of floor immediately above the ground line
gets classified as obstacle, varying from a thin sliver to most of the upper
frame. Measured on a 49.5° vertical field of view, one degree of unmodelled
downward tilt is enough to start it; eight degrees flips roughly a hundred rows.

The mechanism is that a floor pixel's computed height is
`-camera_height × sin(el) / sin(el - tilt)`, which tends to **zero** as the pixel
approaches the boresight row, independent of how high the camera is mounted. Zero
is well above any sensible ground threshold, so distant floor reads as an
obstacle.

Getting the sign wrong doubles the error rather than correcting it. If setting a
value makes the band worse, reverse the sign before changing the magnitude.

### Mount Roll (deg) — `mount_roll_deg`
`Float`, default `0.0`, range `-90.0` to `90.0`

The sensor's fixed roll on its mount, positive right side down. Same idea as
pitch; the symptom of an error is a ground line that is not level in the overlay.

**Precedence is additive, not either/or.** The mount angles describe the sensor's
attitude within the frame the NavPose describes, and the NavPose describes that
frame's attitude against gravity — so both are applied. If your NavPose is
already published for the sensor frame rather than for a vehicle the sensor is
bolted to, leave the mount angles at zero or you will count the mount twice.

---

## Ground and obstacle split

### Ground Height (m) — `ground_max_height_m`
`Float`, default `-0.3`, range `-10.0` to `10.0`

Returns at or below this height, **relative to the sensor**, are ground.

Because it is relative to the sensor, the value is normally negative and roughly
the negative of the sensor's mounting height above the floor, raised by whatever
step height the vehicle can safely drive over. A sensor 1.2 m up on a vehicle that
can climb a 0.1 m lip wants about `-1.1`.

Too low and the floor becomes an obstacle. Too high and real low obstacles are
dismissed as ground.

### Obstacle Max Height (m) — `obstacle_max_height_m`
`Float`, default `100.0`, range `-10.0` to `100.0`

Returns above this height are **overhead** — a ceiling, a rafter, a gantry the
vehicle drives under — and are neither ground nor obstacle. They appear in
neither segmentation map and cannot join a component.

The default of `100.0` is deliberately neutral: `max_range_m` also tops out at
100, so nothing can be clipped until you lower it. Set it to your vehicle's
clearance height to stop reporting structure you will pass safely beneath.

There are three outcomes here, not two: ground, obstacle, and overhead.

---

## Grouping

### Range Step (m) — `range_step_m`
`Float`, default `0.5`, range `0.05` to `20.0`

Neighbouring returns differing by more than this in range are treated as separate
obstacles.

Height thresholding alone puts everything standing above the ground into one
connected region, so a near object and the wall behind it merge into a single
blob. Breaking connectivity across range discontinuities is what separates them,
and it keeps a continuous surface whole — unlike range banding, which would slice
an object in two wherever it crossed a band edge.

Smaller values split more aggressively. If a near object is being swallowed by
the background behind it, lower this before touching anything else. Too small and
a single sloped surface fragments.

The test covers all eight neighbours, matching the 8-connectivity used to label
components. These two must agree: a 4-neighbour test feeding an 8-connectivity
labeller leaves diagonal steps invisible to the only mechanism that breaks
connectivity, and one such pixel pair is enough to fuse a pillar into the wall
behind it.

---

## Reporting

### Min Obstacle Size — `min_obstacle_size_ratio`
`FloatSlider`, default `0.02`, range `0.0` to `1.0`

Smallest obstacle to report, as a fraction of the depth map area. At the default,
a component must cover 2% of the frame — about 6,100 pixels on 640×480.

This is also what absorbs sub-degree geometry errors: a residual band too small
to form a qualifying component never becomes a detection. Lowering it surfaces
smaller real obstacles and, at the same time, surfaces noise and any residual
misclassification.

### Max Obstacles — `max_obstacles`
`Int`, default `10`, range `1` to `50`

Report at most this many obstacles per cycle, largest first by pixel area.

---

## Diagnosing a bad detection

Work down the pipeline, not from the symptom:

| symptom | first control to check |
|---|---|
| Large box in the same place every frame, ground line pinned to a fixed row | `no_return_range_m` — set it to 0 |
| Ragged full-width band of floor just above the ground line | `mount_pitch_deg` — negative for a downward tilt |
| Ground line not level in the overlay | `mount_roll_deg` |
| Near object merged into the wall behind it | `range_step_m` — lower it |
| Whole scene reported as one large obstacle | levelling is wrong; check pitch sign before anything else |
| Ceiling or rafters reported as obstacles | `obstacle_max_height_m` |
| Floor reported as obstacle everywhere | `ground_max_height_m` too low, or pitch sign inverted |
| Nothing detected at all | `max_range_m`, then `min_obstacle_size_ratio` |

A useful sanity check is the share of the frame classified as ground. On a
ground vehicle in a room it should be a substantial fraction — tens of percent.
If it collapses when you change a levelling control, that control moved the wrong
way.
