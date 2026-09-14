# RoboRIO Velocity Command Contract — Addendum

Audience: the WPILib developer writing the RoboRIO side. This document is
self-contained. You do not need to read any NEPI source to implement it.

This addendum adds a **timed chassis-velocity command** to the existing
NEPI-to-RoboRIO NetworkTables contract. It does not replace the existing
position command. A REV MAXSwerve drivetrain is holonomic and its natural input
is a chassis velocity, not a waypoint, so NEPI needs a way to say "drive at this
velocity for this long".

---

## What already exists

The RBX Command Request group at `/NEPI/RBX/Command` already carries:

| Key | Type | Meaning |
|---|---|---|
| `/NEPI/RBX/Command/request_id` | int | Increases on every request. Trigger on this changing. |
| `/NEPI/RBX/Command/command_type` | int | 1 = CHASSIS_SPEEDS, 2 = TARGET_POSE, 3 = NAMED_ACTION, 4 = STOP |
| `/NEPI/RBX/Command/named_action` | string | Action name when `command_type` is 3 |
| `/NEPI/RBX/Command/timestamp` | double | Request time in seconds |
| `/NEPI/RBX/Command/chassis_speeds/velocity_x_mps` | double | Forward velocity, m/s |
| `/NEPI/RBX/Command/chassis_speeds/velocity_y_mps` | double | Left velocity, m/s |
| `/NEPI/RBX/Command/chassis_speeds/angular_velocity_radps` | double | Yaw rate, rad/s |
| `/NEPI/RBX/Command/target_pose/x_m` | double | Target x, field frame, m |
| `/NEPI/RBX/Command/target_pose/y_m` | double | Target y, field frame, m |
| `/NEPI/RBX/Command/target_pose/heading_rad` | double | Target heading, rad |

NEPI writes every field on every request, zeros and empty strings included, and
writes `request_id` last and then flushes. So a RoboRIO that triggers on
`request_id` changing always reads a complete request and can never read a new
`command_type` against the previous command's payload.

**`command_type` 1, CHASSIS_SPEEDS, is already in the contract and is currently
written only by `go_stop`, with all three speeds zero. This feature is the first
real use of it.** Nothing about the existing keys changes.

---

## What to add

Four additions. Three new keys and one new string in an existing list.

| Key | Type | Direction | Meaning |
|---|---|---|---|
| `/NEPI/RBX/Command/chassis_speeds/duration_s` | double | NEPI → RoboRIO | Seconds. Hard upper bound on how long this velocity command may run. |
| `/NEPI/RBX/Feedback/max_velocity_mps` | double | RoboRIO → NEPI | The drivetrain's maximum linear speed, m/s. |
| `/NEPI/RBX/Feedback/max_angular_velocity_radps` | double | RoboRIO → NEPI | The drivetrain's maximum yaw rate, rad/s. |
| `GOTO_VELOCITY` in `/NEPI/RBX/Feedback/supported_capabilities` | string in list | RoboRIO → NEPI | Advertises that this robot accepts timed velocity commands. |

### `duration_s`

`duration_s` is a **hard upper bound**, not a hint. The RoboRIO stops the
drivetrain when `duration_s` has elapsed since the request `timestamp`, whether
or not NEPI has said anything since. NEPI writes it on every request, including
the ones where it means nothing (it is zero on a TARGET_POSE or a STOP), so the
zero-fill rule above still holds.

### `GOTO_VELOCITY` in `supported_capabilities`

NEPI does not offer velocity control at all unless the robot advertises it. With
`GOTO_VELOCITY` absent from `supported_capabilities`, the velocity command
surface is not published, the operator UI reports that the connected robot does
not support velocity moves, and no CHASSIS_SPEEDS request with a non-zero speed
is ever written. Add the string when the RoboRIO side is implemented, not
before.

### The two max fields

**Today** they are a clamp. NEPI reads them and clamps an outgoing command down
to them before writing it. That is the floor under an operator mistyping a
speed — the current design asks a human for the robot's speed in a text box (see
below), and a fat-fingered `30` instead of `3` should be caught somewhere.

**Later** they are the value NEPI reads *instead of* asking an operator for it.
That change is planned and deferred, not speculative, and these fields are what
it will read.

Report them honestly even if your real limit is conservative. A conservative
number that is true is more useful than an optimistic one that is not: NEPI
clamps to what you report, so an inflated value removes the protection and a
deflated one only makes NEPI's commands shorter than they need to be. Reporting
`0.0` disables the clamp on NEPI's side entirely — that is the "not reported"
value, not "no motion allowed".

---

## The command lifecycle NEPI drives

For one operator-initiated velocity move:

1. NEPI writes one CHASSIS_SPEEDS request: `command_type` 1, the three chassis
   speeds, `duration_s`, a fresh `timestamp`, and a new `request_id`.
2. NEPI **re-writes the same velocities at 10 Hz for the whole duration**, with
   a fresh `timestamp` and an incremented `request_id` on each write. The
   velocities do not change between re-writes; only `request_id` and `timestamp`
   do.
3. When `duration_s` has elapsed, NEPI writes one `COMMAND_TYPE_STOP` request
   (`command_type` 4) with all three speeds zeroed.

An operator cancel cuts the loop short and goes straight to step 3.

---

## Watchdog — required, not optional

**While a CHASSIS_SPEEDS command is active, the RoboRIO must zero the drivetrain
if no fresh request has arrived within 250 ms.**

This is a safety requirement. The concrete failure it prevents: the network link
drops mid-move — NEPI crashes, the radio fails, someone unplugs the Ethernet —
and the last thing the RoboRIO received was "drive forward at 2 m/s". Without a
watchdog the robot keeps driving at 2 m/s until it hits something.

`duration_s` and the watchdog are **two independent stops and both are
required**. They fail differently:

- The watchdog stops the robot when NEPI stops talking. It does nothing if NEPI
  keeps talking but is wrong.
- `duration_s` stops the robot when the commanded time is up. It does nothing if
  the request that carried it never arrived, or arrived corrupt.

Implementing one and not the other leaves a real failure uncovered.

---

## Frames, signs and units

Get any of these wrong and the robot moves, correctly, in the wrong direction.
Each is stated with what breaks.

### Robot-relative, not field-relative

The three velocities go straight into:

```java
ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
```

**Do not call `ChassisSpeeds.fromFieldRelativeSpeeds`.** NEPI computes these
velocities from a point the operator clicked in the robot's own camera image, so
they are already in the robot's frame. Converting from field-relative would
rotate the command by the robot's current heading, and the robot would drive off
at an angle that changes every time it turns.

### NWU sign convention

- `+x` is forward
- `+y` is **left**
- `+omega` is counter-clockwise (positive yaw to port)

This is the same convention as the NEPI body frame and the same convention WPILib
uses. **No sign inversion is needed anywhere.** This is stated explicitly so
nobody adds one: a negation on `velocity_y_mps` "to fix the handedness" makes the
robot strafe the wrong way on every command.

### Units on the wire are SI

`velocity_x_mps` and `velocity_y_mps` are metres per second.
`angular_velocity_radps` is **radians** per second. `duration_s` is seconds.

NEPI's operator-facing rotational field is in degrees per second, and NEPI
converts to rad/s before writing. The RoboRIO never sees degrees.

### There is no vertical velocity

`velocity_z_mps` is **not** part of this group and must not be added to it. NEPI
carries a vertical velocity component internally, because the same command
surface serves drones and underwater vehicles, but for a ground robot it writes
nothing vertical to NetworkTables.

---

## What NEPI reads back while the move runs

Nothing new. The existing RBX Feedback fields are what NEPI watches:

| Key | Used for |
|---|---|
| `/NEPI/RBX/Feedback/active_request_id` | Confirms which request the RoboRIO is acting on |
| `/NEPI/RBX/Feedback/active_request_type` | Reported to the operator as the current process |
| `/NEPI/RBX/Feedback/request_status` | `IDLE`, `ACCEPTED`, `EXECUTING`, `COMPLETE`, `FAILED`, `REJECTED` |
| `/NEPI/RBX/Feedback/status_message` | Shown verbatim to the operator on a failure |

`FAILED` and `REJECTED` surface `status_message` as the device's last error
message. Any other `request_status` value is passed through to the operator
verbatim rather than reinterpreted, so a robot-specific status string is safe to
report.

---

## Keys to implement, as a checklist

Read (NEPI writes, RoboRIO consumes):

```
/NEPI/RBX/Command/chassis_speeds/duration_s          double
```

Write (RoboRIO produces, NEPI consumes):

```
/NEPI/RBX/Feedback/max_velocity_mps                  double
/NEPI/RBX/Feedback/max_angular_velocity_radps        double
/NEPI/RBX/Feedback/supported_capabilities            string[]  -- add "GOTO_VELOCITY"
```

Behaviour:

```
On CHASSIS_SPEEDS (command_type 1):
  - drive robot-relative ChassisSpeeds(vx, vy, omega), NWU, SI units
  - stop when (now - timestamp) >= duration_s
  - stop if no request has arrived in the last 250 ms
On COMMAND_TYPE_STOP (command_type 4):
  - stop
```
