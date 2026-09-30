# Feedback Pipeline — NetworkTables to Auto Move

Audience: whoever is debugging why the Auto Move app does or does not know
something about the robot. The mirror of `VELOCITY_PIPELINE.md`, which covers
commands going out; this covers telemetry coming in.

The important thing up front: **this is not one chain.** One NetworkTables poll
fans out into four streams that arrive by different mechanisms, at different
rates, and one of them does not arrive at all. Treating it as a single path is
what makes the gap invisible.

---

## The four streams

| Stream | NT source | Mechanism | Reaches Auto Move? |
|---|---|---|---|
| Capabilities | `/NEPI/RBX/Feedback/supported_capabilities` | Gates device construction, then a service query | **Yes** — as `has_goto_velocity` |
| Request state | `/NEPI/RBX/Feedback/request_status`, `status_message`, `active_request_type` | 2 Hz timer folds into the device's status | **Yes** — inside `status_dict` |
| Pose | `/NEPI/Position`, `/NEPI/Velocity`, `/NEPI/Orientation` | 10 Hz navpose publish | Published, but Auto Move does not subscribe |
| **Speed limits** | `/NEPI/RBX/Feedback/max_velocity_mps`, `max_angular_velocity_radps` | Read on demand inside `gotoVelocity` | **No — dead end.** See below. |

Everything starts from one poll: `wpilib_if_app_node.py:955`, at
`NT_POLL_RATE_HZ = 10.0`, caching the whole group into `rbx_feedback_dict`. The
RBX module never reads NetworkTables itself — the app node owns the cache and
hands the whole dict over through the injected `getRbxFeedbackFunction`.

---

## Stream 1 — capabilities

```
supported_capabilities (NT)
  -> wpilib_if_app_node.py:1524  updateRbxIF        gates construction
  -> wpilib_rbx_if.py:256        gotoVelocityFunction=... if hasCapability(...) else None
  -> device_if_rbx.py:445        has_goto_velocity = (function is not None)
  -> RBXCapabilitiesQuery service
  -> auto_move_if.py:1603        get_goto_capabilities()
  -> auto_move_if.py:1609        self.rbx_has_goto_velocity
```

**Decided once, at construction, and cached.** `updateRbxIF` deliberately refuses
to build the device until `supported_capabilities` is non-empty, because building
early would permanently advertise a robot with no controls. If the RoboRIO starts
advertising `GOTO_VELOCITY` after the device exists, the changed list is what
triggers a rebuild — nothing mutates the flag in place.

Auto Move reads this on a **service call**, not a subscription, and only on a
selection change or first connection of a selection
(`auto_move_if.py:967-973`) — not every tick. An unanswered query reports
nothing rather than defaulting to supported: an unanswered query is not a yes.

## Stream 2 — request state

```
request_status / status_message / active_request_type (NT)
  -> wpilib_rbx_if.py:779  feedbackUpdateCb, FEEDBACK_UPDATE_RATE_HZ = 2.0
  -> RBXRobotIF            update_error_msg()  -> status.last_error_message
                           setProcessNameCb()  -> status.process_current
  -> DeviceRBXStatus topic
  -> connect_device_if_rbx.py:253  get_status_dict()
  -> auto_move_if.py:1788          robot_dict['status_dict']
```

Two deliberate restrictions in `feedbackUpdateCb` worth knowing before you
"fix" them:

- **Error messages are edge-triggered** on `request_status` changing, so one
  failed request does not overwrite the field every tick.
- **`process_current` is only written while the device is idle.** During a goto,
  RBXRobotIF's own blocking convergence loop owns that field, and two writers
  would fight over it.

## Stream 3 — pose

Position, velocity and orientation come from three *different* tables, not from
the RBX Feedback group, and go out as navpose at `navpose_update_rate=10`
(`wpilib_rbx_if.py:262-263`). Each is gated by `isGroupLive` — connection, the
group's own `valid` flag, **and** an age inside `NAVPOSE_STALE_SEC = 1.0`. All
three are required, because an absent group reads back as its defaults.

Auto Move does not subscribe to this. It does not need it for open-loop velocity
moves. Any closed-loop work would start here.

---

## The gap: speed limits stop at the clamp

`max_velocity_mps` and `max_angular_velocity_radps` arrive from the RoboRIO and
are read by `wpilib_rbx_if.py:624` `getMaxSpeeds()` — which is called **only**
from inside `gotoVelocity`, to clamp an outgoing command. They go no further.

Verified: neither `DeviceRBXStatus.msg` nor `RBXCapabilitiesQuery.srv` has any
max-speed field, and `device_if_rbx.py` and `connect_device_if_rbx.py` contain no
reference to one. There is no path by which Auto Move could learn them.

This is why Auto Move asks a human for the robot's speed in a text box
(`move_speed_mps`, `auto_move_if.py:297`) while the robot is sitting there
reporting its own limit. `ROBORIO_VELOCITY_CONTRACT.md` names this as planned and
deferred, not speculative — these are the fields it will read.

Consequence today: the clamp is the only protection, and it lives at the far end
of the chain. An operator who types `30` instead of `3` gets a plan built around
30 m/s, a duration derived from 30 m/s, and then a command silently scaled down
at hop 10 — so the move takes the duration of a 30 m/s run at the robot's actual
speed, and stops far short of where they clicked.

### What has to be added

Four changes, each following a pattern already in the tree. No new mechanism.

**1. Two fields on the status message** — `nepi_interfaces/msg/DeviceRBXStatus.msg`:

```
float32 max_velocity_mps
float32 max_angular_velocity_radps
```

Status, not `RBXCapabilitiesQuery`. Capabilities are derived once and cached by
design (see the 2026-08 decided-once entry in the root CLAUDE.md); these are live
telemetry that can change while connected. Putting them in the capabilities
report would freeze them at construction.

**2. One constructor callback on `RBXRobotIF`** — same shape as the
`getHomeFunction` / `getMotorControlRatios` callbacks it already takes:

```python
getMaxSpeedsFunction = None     # returns [max_velocity_mps, max_angular_velocity_radps]
```

written into the two status fields on each status publish, and deriving a
`has_max_speeds` flag from whether the callback is non-None — which is the
existing convention for every other optional capability on this class.

**3. One line in the WPILib app** — `wpilib_rbx_if.py`, in the `RBXRobotIF`
construction:

```python
getMaxSpeedsFunction = self.getMaxSpeeds,
```

`getMaxSpeeds()` already exists and already returns exactly that pair, in exactly
that order. Nothing on the WPILib side needs writing.

**4. Auto Move reads them** — it already calls `get_status_dict()` every tick in
`getRobotDict` (`auto_move_if.py:1788`), so the values arrive with no new
subscription. What is needed is the use: clamp `move_speed_mps` to the reported
maximum when one is reported, and report it in the status so the RUI can show the
operator the robot's real limit instead of an empty box.

Keep `0.0` meaning **"not reported"** at every step, disabling the clamp rather
than forbidding motion. That is the convention the contract states and the one
`getMaxSpeeds()` already implements; a step that reads `0.0` as a real limit
stops the robot dead.

Scope note: items 1 and 2 are in `nepi_engine_ws` (`nepi_interfaces` and
`nepi_api`), items 3 and 4 in `first_robotics`. A message field change means a
full rebuild, not an app deploy.

---

## Latency, end to end

Nothing here is event-driven. Each stage polls.

| Stage | Period |
|---|---|
| NT poll into the cache | 100 ms |
| `feedbackUpdateCb` folds it into device state | 500 ms |
| Auto Move `updaterCb` | 1000 ms |

So a `request_status` change on the RoboRIO takes up to **~1.6 s** plus the RBX
status publish period to become visible in Auto Move. That is fine for operator
display and far too slow for control — which is why the velocity command carries
its own duration and the RoboRIO runs its own 250 ms watchdog, rather than either
side closing a loop through this path.

---

## Where it breaks

| Symptom | Look at |
|---|---|
| RBX device never appears | `supported_capabilities` is empty or unread. `updateRbxIF` returns without building, by design. |
| Device appears with no velocity controls | It was built before `GOTO_VELOCITY` was advertised. Flags are cached at construction; the list has to change to force a rebuild. |
| First poll shows all defaults | Expected. NT4 delivers only to an existing subscriber and the entry is created on first read, so the first read of a group always returns defaults — one poll period, correctly reported as invalid rather than as zeros (`nepi_wpilib.py:303-309`). |
| Feedback present but ignored | `hasRbxFeedback()` requires `timestamp > 0.0`. The group has no `valid` flag, so an unstamped group is indistinguishable from an absent one. |
| Pose stale, everything else fine | Pose comes from three different tables and is gated on `valid` **and** `age_s <= 1.0`. RBX Feedback is gated differently and will keep flowing. |
| Operator speed ignored / move falls short | The clamp at hop 10 is silent to Auto Move. It logs when it bites — check the app node log for `GoTo Velocity clamped`. |
