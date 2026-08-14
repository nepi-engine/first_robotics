# TUM `freiburg2_pioneer_360` Test Data

Notes for running the obstacles app against the TUM RGB-D sequence
`freiburg2_pioneer_360`, replayed through `nepi_app_file_pub_depthmap`.

## Settings

Only the controls that need a specific value for this data. Everything not
listed stays at its default — see `ObstaclesControls.md`.

| Control | Value | Why |
|---|---|---|
| Principal Point X | `0.5080` | `cx / width = 325.1 / 640`, from the fr2 calibration |
| Principal Point Y | `0.5202` | `cy / height = 249.7 / 480`, from the fr2 calibration |
| Mount Pitch (deg) | `-6.0` | The Kinect is mounted nose-down on the Pioneer. Negative is nose-down. |

Confirm the source is publishing `width_deg 63.1` and `height_deg 49.5`.

Leave **No Return Range at 0**. This data needs the detection, and 0 is what
enables it — see the clamp section below.

`Mount Pitch -6.0` is an empirical fit, not a measured value. TUM does not
publish the mounting angle. See "What is not known" at the end.

---

## The dataset

Part of the [TUM RGB-D SLAM Dataset and Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
from the Computer Vision Group at TU Munich.

- Recorded from a Microsoft Kinect **mounted on top of a Pioneer robot**, in an
  industrial hall.
- The robot was joysticked on the spot through a **360°+ turn** — almost pure
  rotation, very little translation.
- 72.75 s, ground-truth trajectory length 16.118 m.
- 640×480 at 30 Hz.
- The original bag files also carry laser scan and wheel odometry.

The pure-rotation motion matters for interpretation: consecutive frames sweep
across the whole hall, so scene content changes completely while the camera
geometry stays fixed. That makes it a good test for distinguishing real
scene-dependent detections from geometry artifacts — anything that appears in the
same place across frames of totally different content is an artifact, not an
obstacle.

## Camera intrinsics (freiburg2)

```
fx 520.9    fy 521.0    cx 325.1    cy 249.7        on 640x480
```

The field of view follows from the focal lengths:

```
width_deg  = 2 * atan(320 / 520.9) = 63.1 deg
height_deg = 2 * atan(240 / 521.0) = 49.5 deg
```

which is what `file_pub_depthmap_app_node.py` publishes for this data.

The principal point is the part that is easy to miss. `cy = 249.7` sits **9.7
pixels below** the centre row, worth a constant `atan(9.7 / 521) = 1.067°` of
elevation on every pixel in the frame. Before the app had principal point
controls it assumed the optical axis pierced the middle pixel, so that 1.067° was
silently baked into every reported elevation and every computed height.

Correcting it *raises* elevation. On its own it slightly increases floor
misclassification — it is a real calibration correction, not a fix for the floor
band, and the two should not be confused.

## Depth encoding — and the clamp

TUM's own format is documented as:

- 640×480 16-bit monochrome PNG
- scale factor **5000** — a pixel value of 5000 is 1 metre
- **a pixel value of 0 means missing value / no data**

**The `.npy` files in this test set do not follow that convention.** They encode
invalid pixels as exactly **10000.0 mm**, with zero NaN, zero inf and zero zeros
in the array. On a sampled frame, **18.8%** of all pixels (57,901 of 307,200) sit
at exactly that value.

The clamp was introduced by whatever converted the PNGs to `.npy`; it is not in
the source data. TUM uses 0 for no-data.

This is worth understanding because of what it did before the app could detect
it. `max_range_m` defaults to exactly `10.0` and the range gate was inclusive at
its upper bound, so every clamped pixel entered the pipeline as a genuine 10 m
return. Height is `range × sin(elevation)`, which crosses a `-0.30 m` ground
threshold at `-1.70°` — row 256 of 480. The result was a plateau guillotined
along a fixed horizontal line: a large phantom obstacle above it, phantom ground
below it, in the same place in every frame.

The signature was unmistakable once looked for — ground rows `257..479` were
byte-identical across five frames of completely different scenes.

Setting `max_range_m` to 9.9 also removes it, but that is a workaround. Leave
`No Return Range` at 0 and let the app detect the plateau; it then works for any
source regardless of where its clamp sits.

## Measured results

208 frames, every 4th of the 830-frame sequence.

| config | obs/frame | ground% | largest component | floor band | pillars | empty |
|---|---|---|---|---|---|---|
| defaults | 1.48 | 31.8% | 0.319 | 45/208 | 36/208 | 0 |
| pitch **+5.0** (wrong sign) | 1.15 | 11.3% | 0.548 | 5/208 | **0/208** | 0 |
| calibrated + pitch **-6.0** | 1.63 | **45.4%** | 0.163 | **0/208** | **58/208** | 44 |
| calibrated + pitch **-5.0** | 1.60 | 44.1% | 0.174 | 0/208 | 58/208 | 44 |

- *ground%* — share of frame correctly classified as ground
- *largest component* — area ratio of the biggest detection; ~0.7 means the whole
  scene collapsed into one useless blob
- *floor band* — frames containing a full-width low-elevation floor strip
- *pillars* — frames containing a near (<3 m) taller-than-wide component
- *empty* — frames reporting no obstacles at all

The wrong-sign row is included deliberately. Applying `+5.0` to a nose-down
camera doubles the error instead of correcting it: ground collapses to 11.3% and
**every pillar detection in the sequence is lost**. Its floor-band count looks
good only because the band has grown into blobs too large to match a band test.
If a levelling change collapses ground%, reverse the sign before adjusting the
magnitude.

## Known-good reference detections

Useful as regression anchors. Frame names are abbreviated from
`D2011-07-28T18-13-20p398TzUTC_freiburg2_pioneer_360-depth_map.npy`.

| frame | expect |
|---|---|
| `398TzUTC` | a pillar at ~1.77 m, ~46 px wide, taller than wide |
| `788TzUTC` | a pillar at ~1.74 m, ~55 px wide |
| `342TzUTC` | a pillar at ~2.18 m |
| `469TzUTC` | no obstacles at the corrected settings |
| `424TzUTC` | no obstacles at the corrected settings |

`398TzUTC` is the most informative single frame: at the corrected settings it
decomposes into four components — three wall sections at 4.4–5.4 m plus the
pillar at 1.77 m. If it comes back as one blob, levelling is wrong. If the pillar
is missing but the walls are present, the range-step grouping is fusing it into
the background.

## What is not known

1. **The actual mount angle.** TUM documents only that the Kinect was "mounted on
   top of a Pioneer robot" — no tilt, no height. `-6.0` was fitted by sweeping the
   control and maximising correctly-classified ground while keeping pillar
   detections. Anywhere from `-5` to `-8` behaves similarly; past `-8` pillar
   detections start dropping, which is the over-correction boundary.

2. **Whether the 44 empty frames are correct.** At the corrected settings, 21% of
   frames report no obstacles, against 0% at defaults. A 360° spin will
   legitimately sweep across open floor with nothing in range, but this has not
   been confirmed frame by frame against the overlay and could indicate
   over-correction.

3. **The camera height.** Not published, and not needed by the app — height is
   measured relative to the sensor, so `ground_max_height_m` at its `-0.3` default
   works without knowing it. It would be needed to verify the pitch fit
   independently.

## Source data

830 frames. Each frame has `-color_image.png`, `-depth_map_image.png` and
`-depth_map.npy`.

```
/mnt/nepi_storage/sample_data/Depthmap/tum_pioneer_360
```

## References

- [TUM RGB-D Dataset and Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)
- [TUM RGB-D file formats — intrinsics and depth encoding](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)
- [Sequence downloads and descriptions](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download)
