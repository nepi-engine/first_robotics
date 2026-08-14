# WPILib IF App — Design Memo

Target repo: `nepi_app_wpilib_if` (standalone app repo, not a submodule of `nepi_engine_ws`).

This memo records four decisions that turn the app scaffold into the real
NEPI-to-RoboRIO interface, with the evidence each rests on. The app is the only
NEPI process holding a WPILib NetworkTables (NT) client. Everything NEPI sees of
the robot passes through it.

## NetworkTables groups covered

Inputs (RoboRIO to NEPI): Motor-Control Feedback, Robot Position, Robot
Velocity, Robot Orientation, RBX Feedback.
Output (NEPI to RoboRIO): RBX Command Request.

The field names inside each group are fixed by the project doc. The NT **key
paths** are not — the project doc names one concrete path, the heartbeat at
`/NEPI/System/heartbeat`. Every group path in this app therefore follows that
`/NEPI/<Group>/...` shape and is declared once, as a module constant in
`scripts/nepi_wpilib.py`. Those constants are the NEPI half of the contract; the
RoboRIO code must use the same strings.

One output key group is added beyond the project doc's list, and it is called
out here rather than buried: **Motor Command**
(`/NEPI/MotorCommand/<motor_id>/{speed_ratio,timestamp}`). The doc's only output
group, RBX Command Request, is chassis-level — it has no per-motor field — but
`RBXRobotIF` exposes per-motor manual control through `set_motor_control`
(`MotorControl.motor_ind` + `speed_ratio`), and that command has to reach a
motor somehow. Either the RoboRIO implements this key group or per-motor manual
control does not exist for this robot. This app implements the write; the
RoboRIO side is a stated dependency, not an assumption.

---

## Decision 1 — Where motor feedback lives

**Recommendation: configuration in ROS params, live feedback in memory on the
node and republished as `nepi_interfaces/MotorsStatus`. The working assumption is
validated, not refuted.**

Evidence that streaming telemetry does not belong in params:

- `config_mgr` is a "ROS param server ↔ filesystem bridge"
  (`nepi_engine/CLAUDE.md`). Params are the persistence tier, written through to
  YAML under `/mnt/nepi_config/` and `/mnt/nepi_storage/user_cfg/`. Nine motors
  reporting position and current at 10 Hz through that path is a disk write
  loop, not a data product.
- Nothing in the tree reads telemetry from params. Every producer publishes:
  `MotorsDeviceIF` (`device_if_motor.py`) publishes `MotorsStatus` on a
  dedicated `motor_status` topic, and its own header comment names the consumer
  — `ConnectMotorsDeviceIF` discovers devices *by that topic*, stripping
  `/motor_status` to recover the device namespace. A param-based design is
  invisible to that consumer.
- `RBXRobotIF` already does exactly this split: `motor_status_pub` publishes
  `MotorsStatus` every status cycle (`device_if_rbx.py:2145`) from an in-memory
  ratio list, while its params hold only configuration
  (`cmd_timeout`, `home_location`, error bounds, image source).

So:

| Kind | Home |
|---|---|
| Slot count, per-slot RoboRIO `motor_id`, per-slot display name | ROS params (persisted) |
| Team number, `rbx_enabled` | ROS params (persisted) |
| `measured_output`, `position`, `velocity`, per-motor timestamp | In-memory cache, republished as `MotorsStatus` |
| `control_mode`, `commanded_output`, `current_amps`, RoboRIO `motor_name` | In-memory cache, republished in this app's own status message |

The residual RoboRIO fields have no `MotorStatus` home (see the Step 5 mapping
table). They ride in a new app-local `WpilibMotorFeedback.msg`, carried as an
array inside `NepiAppWpilibIFStatus`. They are operator-facing diagnostics, so
the 1 Hz latched status message is the right cadence for them; no second topic
is created. Nested app-local message types are the established pattern in this
family — `nepi_app_obstacles` carries `Obstacle[]` inside `Obstacles.msg` the
same way.

**Per-slot enable is derived, not configured.** A slot reports
`motor_enable = False` unless its `motor_id` is both mapped and has actually
been seen on NT. An operator-settable enable flag on top of that would give two
ways to say the same thing and two ways for them to disagree; setting a slot's
`motor_id` to `-1` (unmapped) is the single way to turn a slot off.

## Decision 2 — The motor index mapping

**Recommendation: an ordered list of motor slots, default length four, any
length allowed. Slot order is the NEPI motor index.**

- `motor_slot_count` (int, default 4) and `motor_ids` (ordered list of ints,
  `-1` = unmapped) are params. Slot *k* is NEPI motor index *k*.
- Slot order defines `MotorControl.motor_ind` — `RBXRobotIF.setMotorControl`
  bounds-checks `motor_ind` against `len(self.getMotorControlRatios())`
  (`device_if_rbx.py:1181`), so the ratio list this app returns *is* the index
  space.
- Slot order defines `MotorStatus.motor_name`: `motor_0`, `motor_1`, ... The
  message file states the rule directly ("motor_name is assigned motor_0,
  motor_1, ... in the order motors are added"), and both existing producers
  follow it (`device_if_rbx.py:2024`, `device_if_motor.py`).
- `motor_names` (ordered list of strings) holds the operator's display names.
  These are *not* `MotorStatus.motor_name` — that field is reserved for the
  positional identity above. Display names surface in the RUI and in this app's
  own status message.

**RUI editor: free-entry integer field per slot, with the live list of
`motor_id`s seen on NetworkTables shown read-only beside it.** Not a selection
list populated from discovered ids, and not both as competing editors.

Rationale. A discovered-ids dropdown cannot express the two states that matter
most during bring-up: configuring slots *before* the RoboRIO is publishing
anything (the normal pit workflow — NEPI is up, the robot code is not), and
reserving a slot for a motor that is not on the bus yet. A selector whose only
options come from live data makes the app unconfigurable in exactly the window
when someone is trying to configure it. The discovered-id readout gives the
operator the information a dropdown would have carried — which ids are real —
without making configuration depend on the robot being alive.

## Decision 3 — How the motors become RBX

**Recommendation: Option A. One `RBXRobotIF` in this app node, representing the
whole robot, with the mapped motors as that device's motor channels.**

### Option A — app node hosts one RBXRobotIF

Precedent: `nepi_app_sim_connector` hosts `SimDeviceIF`, and the 2026-08
DECISION LOG entry sanctions an app owning a device contract when the app owns
the transport. That is exactly this case: the app owns the only NT client.

Evidence that one robot is one RBX device with N motor channels:

- `RBXRobotIF` namespaces itself at
  `create_namespace(node_namespace, 'rbx')` (`device_if_rbx.py:222`) — a fixed
  literal, with no per-instance name. **One RBX device per ROS node.**
- `RBXRobotIF` already models N motors *inside* one device:
  `getMotorControlRatios()` returns an ordered ratio list,
  `setMotorControlRatio(ind, ratio)` writes one channel,
  `MotorControl.motor_ind` selects the channel, and `get_motors_status_msg()`
  emits one `MotorStatus` per channel on a single `motor_status` topic.
- The goto surface is chassis-level: `goto_position`, `goto_pose`,
  `goto_location`, `go_home`, `go_stop`. None of it is per-motor.
- Downstream already expects one device per robot.
  `nepi_app_auto_move` selects a robot with `ConnectRBXDeviceIF` and drives it
  with `rbx_if.goto_position(x, y, z, yaw)` plus `check_ready()`
  (`auto_move_if.py:796`, `:1430`, `:1443`). An RBX device published by this app
  appears in that selector with no change to `auto_move`.

### Option B — separate `rbx_wpilib` driver package under nepi_drivers

This is the "Custom FR Robot RBX Driver" box in the project diagram, and it is
where this should end up eventually — drivers, not apps, are where NEPI's
hardware abstraction lives (`NEPI-CODEX.md`, HARDWARE ABSTRACTION FIRST).

Cost today: a driver process is a *second* NT client, or else a NEPI-side
transport from this app to the driver. Two NT clients on one device means two
sets of NT entries, two connection lifecycles and a duplicated
`/NEPI/System/heartbeat` writer, all for one robot. A NEPI-side transport means
inventing a second protocol (socket or ROS) to carry data that is already in
this process. `rbx_gazebo_discovery.py` shows the shape of the discovery work
too: a liveness probe, launch backoff, a `dont_retry_list`, and a `drv_dict`
written to the param server — all of which is real work with no benefit while
the NT client lives here.

Rejected for now, not rejected on principle.

### Option C — one RBXRobotIF per motor

A literal reading of "four RBX devices". Stated plainly, the cost is:

- **It cannot be done in this node at all.** `RBXRobotIF`'s namespace is
  hardcoded to `<node>/rbx`. Four instances in one process would advertise four
  identical topic sets at the same names — the wire-level collision the 2026-07
  DECISION LOG entry describes. Four devices means four ROS nodes, so this app
  would have to launch and supervise four child processes.
- **Every chassis command would have to be faked four times.** `goto_position`
  on a single motor is meaningless, so each of the four devices would either
  refuse goto (making the robot uncommandable through RBX) or all four would
  accept it and each forward the same chassis command, giving four competing
  request ids for one physical maneuver.
- **It would misreport the system.** Four RBX devices in
  `auto_move`'s selector, each claiming to be a robot, for one robot. NavPose
  would be published four times over.

Rejected.

### How Option A is later relocated to Option B

The whole RBX surface lives in one self-contained module, `scripts/wpilib_rbx_if.py`:

- The module owns the `RBXRobotIF` construction and every callback it needs.
  Nothing RBX-shaped is left in `wpilib_if_app_node.py`.
- Its only inputs are four injected callables — read motor slots, read the fused
  navpose, read RBX feedback, read connection state — plus one injected writer
  for the RBX Command Request. It never imports the NT layer and never touches
  `ntcore`.
- The app node's entire job is: construct it when `rbx_enabled` is true, hand it
  those callables, tear it down when `rbx_enabled` goes false.

Relocating to a driver package is therefore a file move plus a transport swap:
the file becomes `rbx_wpilib_node.py`, and the five injected callables get
rebound to whatever transport the driver holds (its own NT client, or a link
back to this app). The RBX callbacks themselves do not change.

## Decision 4 — Where NavPose comes from

**Recommendation: the RBX device's own navpose path is the single owner. This
app creates no `NavPoseIF` of its own.**

`RBXRobotIF` already publishes NavPose whenever `getNavPoseCb` is not None: it
constructs an `NPXDeviceIF` (`device_if_rbx.py:884-896`), which constructs a
`NavPoseIF` at `<npx namespace>/navpose` (`device_if_npx.py:593-613`) and
publishes the driver-supplied dict through it. So a second app-owned publisher
would be a competing publisher of the same pose, and per the constraint it is
not created.

`getNavPoseCb` is not optional here, which is what forces the choice:

- `RBXRobotIF`'s goto commands are blocking convergence loops.
  `setpoint_position_local_body` and `setpoint_attitude_ned` compare a target
  against `self.current_position_enu_m` / `current_orientation_enu_degs`, and
  those attributes are refreshed *only* from `getNavPoseCb`, on a 10 Hz timer
  started only when `getNavPoseCb` is not None (`device_if_rbx.py:907-908`).
- The fallback source is dead. The comment at `device_if_rbx.py:898-906`
  records that the aggregated system `navposes` topic `navposesSysCb` consumes
  "is not published in this build", which left `current_*` at `[0,0,0]` and
  drove goto to lat/lon 0.

With no `getNavPoseCb`, every goto would run to `cmd_timeout` and report
`cmd_success = False` while the RoboRIO actually completed the move. So the RBX
device gets the navpose, and the RBX device is therefore the owner.

Consequence, stated rather than hidden: **NavPose exists only while
`rbx_enabled` is true.** That is coherent — a goto needs pose and pose needs the
robot device, so both track the same telemetry and the same switch — but it does
mean the factory default (`rbx_enabled = False`) publishes no pose. The fusion
and staleness logic still runs unconditionally and is reported in this app's own
status, so pose health is visible with RBX off; only the NavPose *topic* is
gated. If a deployment needs pose without command authority, the fix belongs
upstream in `nepi_api` (an `RBXRobotIF` option to publish navpose without the
command surface, or an `NPXDeviceIF` that a host can own directly), not a second
publisher in this app. Surfaced as a proposed DECISION LOG entry.

Fusion, in one place: Robot Position, Robot Orientation and Robot Velocity are
three NT groups and one NavPose. Each group contributes only when its own
`valid` flag is true and its last observed update is fresh; a group that fails
either test sets its `has_*` flag false rather than contributing zeros, and when
no group qualifies nothing is published at all. Details and the exact staleness
rule are in the Step 6 implementation.

---

## Summary of recommendations

1. **Motor feedback** — configuration in params, live feedback in memory,
   republished as `MotorsStatus` on the standard `motor_status` topic; the four
   RoboRIO fields with no `MotorStatus` home ride in an app-local
   `WpilibMotorFeedback[]` inside the app status message.
2. **Motor index mapping** — ordered slot list, default four, slot order is the
   NEPI motor index and the `motor_N` name; RUI uses a free-entry integer field
   per slot with the live discovered-`motor_id` list shown read-only beside it.
3. **RBX** — Option A: one `RBXRobotIF` for the whole robot, hosted by this app
   node, with the mapped motors as its motor channels, built inside one
   self-contained `scripts/wpilib_rbx_if.py` so the move to a driver package is
   a file move plus a transport swap.
4. **NavPose** — one owner: the RBX device's own navpose path via
   `getNavPoseCb`. No app-owned `NavPoseIF`. NavPose is published only while
   `rbx_enabled` is true, and the reason is `RBXRobotIF`'s goto math, not
   preference.
