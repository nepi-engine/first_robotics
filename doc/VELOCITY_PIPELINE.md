# Velocity Command Pipeline — Auto Move to RoboRIO

Audience: whoever is debugging a velocity move end to end and needs to know which
component to look at. Companion to `ROBORIO_VELOCITY_CONTRACT.md`, which covers
only the last hop and is written for the WPILib developer. This document covers
everything upstream of that.

The short version: an operator click in the Auto Move app becomes one ROS
message, which becomes a stream of NetworkTables writes at 10 Hz for the
commanded duration, followed by one stop.

---

## The hops

| # | Where | What happens |
|---|---|---|
| 1 | `nepi_app_auto_move` RUI | Operator clicks the image. `NepiAppAutoMove.js` sends the click ratios. |
| 2 | `auto_move_if.py` `applyClick` | Click plus depth map resolves to a body-frame offset `x_m, y_m, z_m`, clamped to `max_move_m`. |
| 3 | `auto_move_if.py:1410` `applyClickVelocity` | Offset becomes `x_mps, y_mps, z_mps` and `duration_s`. **Yaw is 0.0** — the centring math is parked here, see below. |
| 4 | `auto_move_if.py:1553` `gotoTriggerCb` | Operator hits GoTo. Separate action from the click. |
| 5 | `auto_move_if.py:1640` `runPlanning` | Builds `goto_dict`; velocity mode selects the `planVelocityMove` planner. |
| 6 | `sdk/nepi_auto_move.py` `plan_velocity_move` | Returns exactly one step. No splitting, no obstacle awareness. An all-zero or zero-duration request returns no steps. |
| 7 | `auto_move_if.py:1701` `runMoving` | Calls `self.rbx_if.goto_velocity(x, y, z, yaw, duration)`. |
| 8 | `nepi_api/connect_device_if_rbx.py:653` | Fills a `GotoVelocity` msg and publishes it. |
| — | **`<rbx_device_ns>/goto_velocity`** | **Process boundary.** Everything above is the Auto Move app; everything below is the WPILib app. |
| 9 | `nepi_api/device_if_rbx.py:1356` `gotoVelocityCb` | Stock engine class. Calls the injected function and expects it to block. |
| 10 | `wpilib_rbx_if.py:646` `gotoVelocity` | Drops z, clamps speed and yaw against the robot's reported maxima, converts deg/s → rad/s, then streams. |
| 11 | `wpilib_rbx_if.py:464` `sendCommandRequest` | One request per stream tick, via the injected write callable. |
| 12 | `wpilib_if_app_node.py:1610` `writeRbxCommandRequest` | Adapter. Logs in test mode, then writes. |
| 13 | `nepi_wpilib.py:911` `write_rbx_command_request` | The only ntcore code in the app. Writes every field, `request_id` last, then flushes. |
| 14 | RoboRIO | Triggers on `request_id` changing. |

Two things that hold across the whole chain and are worth not re-deriving:

**Auto Move never learns what robot it is driving.** It reads one generic flag,
`has_goto_velocity`, off the RBX capabilities report. Swapping the RoboRIO for an
ArduPilot rover changes nothing above the boundary.

**All RoboRIO knowledge lives below hop 10.** `wpilib_rbx_if.py` is deliberately
transport-blind — six injected callables, no ntcore import. Even the
`command_type` integers live in `nepi_wpilib.py` and are injected by name.

---

## The stream

Hop 10 is not fire-and-forget. `gotoVelocity` blocks for the whole move:

- `VELOCITY_STREAM_RATE_HZ = 10.0`, so one CHASSIS_SPEEDS request every ~100 ms
  with the same velocities and a fresh `request_id` and `timestamp`
- exits on stop, shutdown, elapsed duration, or a write that did not land
- **one STOP on every exit path**, including the aborted ones
  (`wpilib_rbx_if.py:751`)

Duration is clamped to `VELOCITY_MAX_DURATION_SEC = 60.0`.

---

## Reading the log

A real capture, decoded:

```
RBX command request out: id 21  GoTo Velocity  command_type 1  chassis_speeds
{'velocity_x_mps': 0.949917733669281, 'velocity_y_mps': -0.17820130288600922,
 'angular_velocity_radps': 0.0, 'duration_s': 5.263613700866699}
```

- `command_type 1` — CHASSIS_SPEEDS
- `velocity_y_mps` negative — NWU, so +y is left; negative is a click to
  starboard, and with yaw at zero the robot strafes rather than turning
- `angular_velocity_radps: 0.0` — the click-centring math is parked, so a click
  commands translation only
- `TEST MODE SENT` — test mode logs the request and then **sends it down the same
  path to the same keys**. It is not a dry run. A dropped write says
  `RBX command request dropped: no NetworkTables client` instead.

### Why the ground speed is under `move_speed_mps`

Above, `sqrt(0.9499² + 0.1782²)` = **0.966 m/s** — short of `move_speed_mps`,
which defaults to 1.0. That is correct, not a rounding artifact.

`applyClickVelocity` divides the **3D** distance by the speed to get the
duration, so `duration_s` covers a vector that includes `z_m`. Hop 10 then drops
z, because a ground robot has no vertical axis. What survives is the horizontal
projection — and horizontal speed times duration still equals the horizontal
distance to the clicked point, so the robot lands in the right place. It simply
takes the time the full 3D vector would have taken.

For this capture: 5.26 s of commanded duration and 5.09 m of ground travel, which
puts the clicked point about 1.35 m off the camera axis vertically — assuming
`move_speed_mps` was still at its 1.0 default, since the vertical component can
only be recovered from the log by working back through that setting.

### Timing margin against the watchdog

Observed spacing between consecutive requests in that capture was ~136 ms
(id 20 at `...638.846`, id 21 at `...638.982`), not 100 ms. The loop sleeps
100 ms **and then** pays for the write, so the effective rate is under 10 Hz.

The RoboRIO watchdog fires at 250 ms. That leaves **1.8x of margin, not 2.5x**.
Worth knowing before anyone lowers the watchdog or raises the write cost.

---

## The parked yaw math

`applyClickVelocity` used to compute a yaw rate that turned the clicked point
onto the image centre line over the same duration. It is commented out, not
deleted, and `MIN_TURN_RAD` is parked with it.

Reverted to translation-only for on-robot bring-up, so a click commands one thing
and a wrong heading cannot be blamed on math nobody asked for. The operator can
still set a yaw rate by hand from the RUI.

Nothing downstream changed — `goto_dict`, `plan_velocity_move`, `goto_velocity`
and the `angular_velocity_radps` key all still carry a yaw rate and simply carry
zero. Restoring it is uncommenting four lines.

The reason it is parked rather than tuned: vx and vy are robot-relative and the
frame rotates under them while the turn runs, so the path is an arc and the robot
does not finish exactly on the clicked point — off by roughly half the turn angle
in bearing, order of 0.8 m sideways on a 3 m move at 30 degrees. The heading
itself always lands correctly.

---

## Where it breaks

| Symptom | Look at |
|---|---|
| No velocity controls in the RUI | RoboRIO is not advertising `GOTO_VELOCITY` in `supported_capabilities`. Hop 10 passes `None` for the callback, `has_goto_velocity` goes False, and the command surface is never published. |
| Request logged, robot does not move | Hops 13–14. Check the RoboRIO is triggering on `request_id` and not on `timestamp`. |
| `command request dropped: no NetworkTables client` | `nt_instance` is None — the app never connected. Nothing above hop 12 is at fault. |
| Robot moves then stops early | The 250 ms watchdog, not `duration_s`. Something stalled the stream; see the timing margin above. |
| Robot strafes the wrong way | Sign convention. +y is **left**. No inversion belongs anywhere; see the contract doc. |
| Speed lower than commanded | Expected when the click has a vertical component — see above. Also check the `max_velocity_mps` clamp, which logs when it bites. |
| Move commanded but nothing planned | `plan_velocity_move` drops all-zero and zero-duration requests, and logs when it does. |
