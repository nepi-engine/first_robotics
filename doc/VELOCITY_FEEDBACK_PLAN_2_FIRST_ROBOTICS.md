# Velocity Feedback — Plan 2: first_robotics changes

**Start only after Plan 1 is deployed.** Plan 1 is the NavPose `has_velocity`
change across nepi_engine_ws and nepi_drones
(`VELOCITY_FEEDBACK_PLAN_1_ENGINE.md`). To confirm it is in place, on the device
run:

```
rosmsg show nepi_interfaces/NavPose | grep has_velocity         # must print a line
rostopic echo -n1 <wpilib node ns>/rbx/status | grep navpose_topic   # must not be empty
```

The second check needs the WPILib app running with `rbx_enabled` on and
telemetry flowing. If either check fails, stop: Plan 1 is not on the device yet.

**What you are building.** The Auto Move app will be able to read the robot's
**measured** velocity through a getter that any process inside the app can call.
The velocity already leaves the RoboRIO and travels inside the robot's NavPose.
This plan makes that value correct at the source, and adds the subscriber and
getter in Auto Move.

Every design decision is already made (see the table below). Nothing here should
need a judgment call. If something doesn't match what this plan describes, stop
and raise it rather than improvising.

---

## The path

| # | Where | What happens |
|---|---|---|
| 0 | RoboRIO | Writes `/NEPI/Velocity/*`, field-relative (Task 1) |
| 1 | `nepi_wpilib.py:780` `read_robot_velocity` | Reads the group and stamps `age_s` from its last observed change |
| 2 | `wpilib_if_app_node.py:953` `ntPollCb` | Polls at 10 Hz into `self.velocity_dict` |
| 3 | `wpilib_if_app_node.py:1417` `get_navpose_dict` | Live if connected, `valid`, and `age_s` ≤ 1.0 s. Sets `has_velocity` explicitly (Task 2) |
| 4 | RBX device → NPX child → `NavPoseIF` | Unchanged. Publishes about 30 Hz on `<wpilib node ns>/npx/navpose` |
| — | **`<wpilib node ns>/npx/navpose`**, `nepi_interfaces/NavPose` | Process boundary |
| 5 | `auto_move_if.py` `updateRobotState` | Reads the topic from the device status and subscribes (Task 3) |
| 6 | `auto_move_if.py` `robotNavPoseCb` | Validity check, field → body rotation, cache (Task 3) |
| 7 | `auto_move_if.py` `get_robot_velocity_dict()` | **Where it ends.** Live value for any process (Task 3) |

## Settled decisions — do not reopen

| Decision | Chosen |
|---|---|
| Frame on the wire | **Field-relative.** The RoboRIO rotates using its gyro, and Auto Move rotates back into the body frame. This matches every other NEPI robot (gazebo, sim, webots, mujoco), so Auto Move never needs to know which robot it is driving |
| Marking stale velocity | **`NavPose.has_velocity`** (Plan 1). It defaults to False, so the WPILib app must set it, and it sets it from `velocity_live` on every navpose |
| Finding the navpose topic | **`DeviceRBXStatus.navpose_topic`** (Plan 1). No namespace guessing |
| What Auto Move exposes | **The getter only.** No planner `robot_dict` key, no status message fields, no RUI |
| Yaw-rate fallback in `get_navpose_dict` | **Leave it, documented as dead.** Yaw rate comes only from `/NEPI/Orientation` |
| No yaw available | **Speed only.** Planar speed stays valid; the body components are `None` |
| Invalid values in the getter | **`None`.** Never `0.0`, never `-999` |
| Getter staleness timeout | **1.0 s**, matching the WPILib app's `NAVPOSE_STALE_SEC`, and measured from when the NavPose **arrives** |
| `time_velocity` in the getter | **Not passed through.** Every NavPose `time_*` field is float32, which at today's epoch only resolves to 128 s steps, so it can't date a sample. The WPILib app still fills it (Task 2), because the message contract asks for it |
| Test mode | **Unchanged.** It must not respond to commands (see its file header), so it reports zero velocity |
| `get_navpose_dict`'s `None` return | **Unchanged.** See the note under Task 2 |
| Mount transform on the WPILib device's `npx` | **Leave it blank.** A non-blank transform zeroes the yaw rate (engine bug, noted in Plan 1) |

---

## Task 1 — RoboRIO contract: `nepi_app_wpilib_if/docs/ROBORIO_VELOCITY_CONTRACT.md`

Make three edits, then give the document to the RoboRIO developer. Task 5's
frame check can't pass until the RoboRIO side implements this.

**1a.** At the end of the *Robot-relative, not field-relative* subsection,
append:

> This applies to commands only. The velocity the RoboRIO reports back is
> field-relative — see *Robot Velocity feedback* below.

**1b.** Insert this section directly before *Keys to implement, as a
checklist*:

````markdown
## Robot Velocity feedback

NEPI reads the robot's measured velocity from the Robot Velocity group and hands
it to the Auto Move app. These rules are what make that reading correct.

| Key | Type | Meaning |
|---|---|---|
| `/NEPI/Velocity/velocity_x_mps` | double | Field-relative x velocity, m/s |
| `/NEPI/Velocity/velocity_y_mps` | double | Field-relative y velocity, m/s |
| `/NEPI/Velocity/velocity_z_mps` | double | Always 0.0 on a ground robot |
| `/NEPI/Velocity/angular_velocity_radps` | double | Not used by NEPI. Yaw rate is read from `/NEPI/Orientation/yaw_rate_radps` |
| `/NEPI/Velocity/timestamp` | double | Sample time, seconds. Must change on every publish |
| `/NEPI/Velocity/valid` | boolean | True while the values are a real measurement |

### Field-relative: the opposite of the command

```java
ChassisSpeeds robotRelative = kinematics.toChassisSpeeds(getModuleStates());
ChassisSpeeds fieldRelative =
    ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getGyroRotation());
```

Commands from NEPI are robot-relative (see *Robot-relative, not field-relative*
above). The velocity you report back is **field-relative**. This is deliberate:
NEPI reports every robot's velocity in the field (navigation) frame, so the same
NEPI code reads a RoboRIO, a simulator or a drone. Do not change either side to
match the other.

`getGyroRotation()` must be **the same angle you publish as
`/NEPI/Orientation/yaw_rad`**. NEPI rotates the velocity back into the robot's
frame using that angle. If the two differ, forward speed shows up as sideways
speed whenever the robot is turned.

WPILib versions before 2024 have no `fromRobotRelativeSpeeds`. On those, rotate
by hand, where `theta` is that same gyro angle in radians:

```java
double vxField = vx * Math.cos(theta) - vy * Math.sin(theta);
double vyField = vx * Math.sin(theta) + vy * Math.cos(theta);
```

Signs and units are NWU. +x runs along field x, +y is 90° counter-clockwise from
it, and counter-clockwise rotation is positive. Speeds are in metres per second.

### What else has to be live

- **`/NEPI/Orientation` valid and fresh.** Without it, NEPI still has the robot's
  speed but not its direction relative to the robot, and it has no yaw rate.
- **`/NEPI/Position` or `/NEPI/Orientation` valid.** When both are invalid, NEPI
  publishes no pose at all, and the velocity goes with it.
- **NEPI treats a group as stale** when its values haven't changed for 1.0 s. A
  stationary robot reports a constant velocity of zero, so `timestamp` is what
  keeps the group fresh: advance it on every publish.
````

**1c.** In the checklist's *Write (RoboRIO produces, NEPI consumes)* block, add:

```
/NEPI/Velocity/velocity_x_mps                        double    field-relative
/NEPI/Velocity/velocity_y_mps                        double    field-relative
/NEPI/Velocity/velocity_z_mps                        double    0.0
/NEPI/Velocity/timestamp                             double    advance every publish
/NEPI/Velocity/valid                                 boolean
```

---

## Task 2 — WPILib app: `nepi_app_wpilib_if/scripts/wpilib_if_app_node.py`

In `get_navpose_dict`, replace the whole `if velocity_live is True:` block
(`:1458-1474`) with:

```python
        # Velocity is FIELD-relative: the RoboRIO rotates it with the same angle
        # it publishes as Robot Orientation yaw_rad, which is the navigation
        # frame every other NEPI navpose producer uses. See
        # docs/ROBORIO_VELOCITY_CONTRACT.md, Robot Velocity feedback.
        #
        # has_velocity defaults False (NavPose.msg), so it must be set here or
        # the robot's velocity is never published. Setting it from
        # velocity_live is also what makes a stale Velocity group go out as
        # -999 instead of a confident 0.0 m/s while Position is still live.
        navpose_dict['has_velocity'] = velocity_live
        if velocity_live is True:
            navpose_dict['time_velocity'] = self.getGroupTime(self.velocity_dict)
            navpose_dict['x_m_per_sec'] = float(self.velocity_dict['velocity_x_mps'])
            navpose_dict['y_m_per_sec'] = float(self.velocity_dict['velocity_y_mps'])
            navpose_dict['z_m_per_sec'] = float(self.velocity_dict['velocity_z_mps'])
            navpose_dict['altitude_m_per_sec'] = 0.0
            # Ground speed along the heading, from the planar velocity.
            navpose_dict['heading_m_per_sec'] = math.sqrt(
                float(self.velocity_dict['velocity_x_mps']) ** 2 +
                float(self.velocity_dict['velocity_y_mps']) ** 2)
            navpose_dict['location_m_per_sec'] = navpose_dict['heading_m_per_sec']
            # Yaw rate appears in BOTH input groups (angular_velocity_radps here,
            # yaw_rate_radps in Robot Orientation). The orientation group is the
            # authority on orientation rates, so this only fills the field when
            # that group is not contributing.
            #
            # DEAD ON THE WIRE -- kept deliberately, do not rely on it.
            # convert_navpose_dict2msg writes yaw_deg_per_sec only when
            # has_orientation is True, and it is False in exactly this branch, so
            # this value is replaced by -999 before publish. Yaw rate reaches NEPI
            # only from Robot Orientation yaw_rate_radps.
            if orientation_live is False:
                navpose_dict['yaw_deg_per_sec'] = math.degrees(
                    float(self.velocity_dict['angular_velocity_radps']))
        else:
            # heading_m_per_sec is computed from velocity here but rides on
            # has_heading, which Position keeps True. Without this it would carry
            # BLANK_NAVPOSE_DICT's 0.0 as a real ground speed.
            navpose_dict['heading_m_per_sec'] = -999
```

**Leave the `return None` gate at `:1433` unchanged.** It returns None when
neither Position nor Orientation is live. `autonomousControlsReady`
(`wpilib_rbx_if.py:521-531`) treats a non-None navpose as "pose is valid, gotos
may run". Letting a velocity-only dict through would arm gotos with no pose. The
consequence is the contract rule in Task 1: velocity arrives only while Position
or Orientation is also live.

---

## Task 3 — Auto Move: `nepi_app_auto_move/api/auto_move_if.py`

**3a. Import.** After `from nepi_interfaces.msg import MgrSystemStatus` (`:35`):

```python
from nepi_interfaces.msg import NavPose
```

**3b. Constants.** After `CONNECTED_TIMEOUT_SEC = 2` (`:167`):

```python
# How long a robot velocity reading is trusted after its NavPose arrives.
# Matches the WPILib app's NAVPOSE_STALE_SEC. This only catches a publisher that
# has stopped; a stale reading from a live publisher is already marked by the
# producer through NavPose.has_velocity.
ROBOT_VELOCITY_STALE_SEC = 1.0

# The robot velocity report get_robot_velocity_dict() returns. Every value is
# None unless its flag is True -- never 0.0, never -999 -- so a caller that
# skips the flag gets a TypeError instead of arithmetic on a fake number.
BLANK_ROBOT_VELOCITY_DICT = {
    'speed_valid': False,     # planar speed is trustworthy
    'body_valid': False,      # body-frame x/y are trustworthy (needs yaw)
    'yaw_rate_valid': False,  # yaw rate is trustworthy
    'x_mps': None,            # body frame, +x forward, m/s
    'y_mps': None,            # body frame, +y LEFT, m/s
    'z_mps': None,            # up, m/s
    'speed_mps': None,        # sqrt(x^2 + y^2), m/s, frame independent
    'yaw_degps': None,        # deg/s, positive counter-clockwise (to port)
    'age_s': None,            # seconds since the NavPose arrived
}
```

**3c. Class attributes.** After `rbx_caps_read_namespace = 'None'` (`:206`):

```python
    # Measured velocity of the selected robot, from its NavPose. The dict is
    # replaced whole, never mutated, so a plain read is a consistent read.
    robot_navpose_topic = ''
    robot_subs_if = None
    robot_velocity_dict = None
    robot_velocity_last_time = 0
```

**3d. Public getter.** Directly after `get_connect_namespace` (ends at `:736`):

```python
    def get_robot_velocity_dict(self):
        """Return the selected robot's measured velocity.

        Body frame of nepi_interfaces/GotoVelocity -- x forward, y LEFT, z up --
        in METERS PER SECOND, yaw rate in DEGREES PER SECOND positive to port.
        That is the frame and units of the goto_x_mps / goto_y_mps /
        goto_yaw_degps this app commands, so measured and commanded compare
        directly. Live: call it on every use rather than holding the result.

        Returns:
            dict: BLANK_ROBOT_VELOCITY_DICT form. A value is None unless its
                flag is True. Every flag is False when no NavPose has arrived
                for the selected robot, or the last one is older than
                ROBOT_VELOCITY_STALE_SEC.
        """
        velocity_dict = self.robot_velocity_dict
        last_time = self.robot_velocity_last_time
        if velocity_dict is None:
            return copy.deepcopy(BLANK_ROBOT_VELOCITY_DICT)
        age_s = nepi_utils.get_time() - last_time
        if age_s > ROBOT_VELOCITY_STALE_SEC:
            velocity_dict = copy.deepcopy(BLANK_ROBOT_VELOCITY_DICT)
        else:
            velocity_dict = copy.deepcopy(velocity_dict)
        velocity_dict['age_s'] = age_s
        return velocity_dict
```

**3e. `unregister`.** After `self.unsubscribeSourceTopics()` (`:874`):

```python
        self.unsubscribeRobotTopics()
```

**3f. `updateRobotState`** (`:955`). Replace the whole function with:

```python
    def updateRobotState(self):
        if self.rbx_if is None:
            return
        selected_topic = self.rbx_if.get_selected_topic()
        if selected_topic is None or selected_topic == '':
            selected_topic = 'None'
        selection_changed = (selected_topic != self.rbx_namespace)
        self.rbx_namespace = selected_topic
        self.rbx_connected = self.rbx_if.check_connection()
        self.rbx_ready = self.rbx_connected and (self.rbx_if.check_ready() == True)

        # Capabilities are a service call, so they are read on a selection
        # change and on the first connection of a selection -- not every tick.
        if selection_changed == True:
            self.rbx_has_goto_velocity = False
            self.rbx_caps_read_namespace = 'None'
            # A velocity reading belongs to the robot it came from.
            self.unsubscribeRobotTopics()
        if self.rbx_connected == True and self.rbx_caps_read_namespace != self.rbx_namespace:
            self.updateRbxCapabilities()
            self.rbx_caps_read_namespace = self.rbx_namespace

        # Follow the selected robot's navpose topic. Re-resolved every tick
        # because the device reports it only once its NavPose exists, which can
        # be well after the device itself connects.
        navpose_topic = self.getRobotNavPoseTopic()
        if navpose_topic != self.robot_navpose_topic:
            self.subscribeRobotTopics(navpose_topic)
```

**3g. Private methods.** Directly after `unsubscribeSourceTopics` (ends at
`:1093`):

```python
    def getRobotNavPoseTopic(self):
        # The device reports where its pose is published
        # (DeviceRBXStatus.navpose_topic). Empty until the robot's NavPose
        # exists -- for the WPILib app, until rbx_enabled is on and telemetry
        # has arrived.
        if self.rbx_if is None or self.rbx_connected == False:
            return ''
        try:
            status_dict = self.rbx_if.get_status_dict()
        except Exception:
            return ''
        if not isinstance(status_dict, dict):
            return ''
        navpose_topic = status_dict.get('navpose_topic', '')
        if navpose_topic is None or navpose_topic == 'None':
            return ''
        return navpose_topic

    def subscribeRobotTopics(self, navpose_topic):
        # One subscriber per selected robot, rebuilt whenever the device's
        # reported navpose topic changes -- the same pattern as
        # subscribeSourceTopics for the image companions.
        self.unsubscribeRobotTopics()
        if navpose_topic == '':
            return

        robot_subs_dict = {
            'auto_move_robot_navpose_sub': {
                    'namespace': navpose_topic,
                    'msg': NavPose,
                    'topic': '',
                    'qsize': 1,
                    'callback': self.robotNavPoseCb,
                    'callback_args': ()
            },
        }

        self.msg_if.pub_info('Registering to robot navpose topic: ' + str(navpose_topic), log_name_list = self.log_name_list)
        self.robot_subs_if = NodeSubscribersIF(
                subs_dict = robot_subs_dict,
                log_name_list = self.log_name_list,
                msg_if = self.msg_if)
        self.robot_navpose_topic = navpose_topic

    def unsubscribeRobotTopics(self):
        if self.robot_subs_if is not None:
            try:
                self.robot_subs_if.unregister_subs()
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister robot subs: " + str(e), log_name_list = self.log_name_list)
            self.robot_subs_if = None
        self.robot_navpose_topic = ''
        self.robot_velocity_dict = None
        self.robot_velocity_last_time = 0
```

**3h. Callback.** Directly after `obstaclesCb` (ends at `:1175`), before the
`# Command Callbacks` banner:

```python
    def robotNavPoseCb(self, msg):
        # NavPose velocity is in the navigation (field) frame. Rotated here into
        # the body frame of nepi_interfaces/GotoVelocity -- x forward, y LEFT --
        # so it compares directly with goto_x_mps / goto_y_mps. The rotation
        # needs yaw; without it only the planar speed survives, because speed
        # does not depend on the frame.
        speed_valid = (msg.has_velocity == True)
        yaw_rate_valid = (msg.has_orientation == True)
        body_valid = speed_valid and yaw_rate_valid

        velocity_dict = copy.deepcopy(BLANK_ROBOT_VELOCITY_DICT)
        velocity_dict['speed_valid'] = speed_valid
        velocity_dict['body_valid'] = body_valid
        velocity_dict['yaw_rate_valid'] = yaw_rate_valid
        if speed_valid == True:
            x_nav = float(msg.x_m_per_sec)
            y_nav = float(msg.y_m_per_sec)
            velocity_dict['speed_mps'] = math.sqrt((x_nav * x_nav) + (y_nav * y_nav))
            velocity_dict['z_mps'] = float(msg.z_m_per_sec)
            if body_valid == True:
                yaw_rad = math.radians(float(msg.yaw_deg))
                velocity_dict['x_mps'] = (x_nav * math.cos(yaw_rad)) + (y_nav * math.sin(yaw_rad))
                velocity_dict['y_mps'] = -(x_nav * math.sin(yaw_rad)) + (y_nav * math.cos(yaw_rad))
        if yaw_rate_valid == True:
            velocity_dict['yaw_degps'] = float(msg.yaw_deg_per_sec)

        self.robot_velocity_dict = velocity_dict
        self.robot_velocity_last_time = nepi_utils.get_time()
```

`copy`, `math`, `nepi_utils` and `NodeSubscribersIF` are already imported in
this file.

**Not built, by decision:** no `robot_dict['velocity_dict']`, no
`NepiAppAutoMoveStatus` fields, and no RUI. Planners don't receive the velocity.
A process that needs it calls `self.get_robot_velocity_dict()` on each tick, for
example from `gotoProcessCb` / `runMoving`.

What the getter is for:
- Stall detection while moving: commanded speed is above zero, but measured
  speed stays near zero for N seconds.
- Arrival confirmation in velocity mode: the device is ready and the measured
  speed is near zero.

It is **not** for setting `move_speed_mps` from the measured speed.
`WPILIB_IF_DESIGN.md:304-307` rejected that, because a robot reports zero while
it sits still, and clicking while stopped is the normal case.

---

## Task 4 — Build and deploy

Both apps' `deploy_app.sh` do two things in one run:
- They copy the app into the build workspace, so a later build keeps the change.
- They live-sync `scripts/`, `api/` and `sdk/` straight into the running NEPI
  container. `api/` lands in `nepi_api` and `sdk/` lands in `nepi_sdk`.

These are Python-only changes, so **no build is needed**. The live sync only
works while NEPI is running. If the script prints `Live Updates Failed`, the
change reached only the build workspace, and it needs `codebld` and a restart.

1. **WPILib app.** This change is `scripts/` only. Run
   `nepi_app_wpilib_if/deploy_app.sh`.
2. **Auto Move.** This change is in `api/`. Run
   `nepi_app_auto_move/deploy_app.sh`, then confirm the live copy from inside
   the container (`nepilogin`):
   `grep -c get_robot_velocity_dict /opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_api/auto_move_if.py`.
   It must print a non-zero count.
3. **Restart both apps.** Disable and re-enable each one, and check that each
   pid changed. rsync does not restart a node, and a running node keeps the old
   code it already imported.
4. **RoboRIO.** Give the updated contract (Task 1) to the RoboRIO developer.
5. **Commit** in first_robotics: the contract doc, `wpilib_if_app_node.py` and
   `auto_move_if.py`. Remove the temporary log line from Task 5 before you
   commit.

---

## Task 5 — Verify

**WPILib side.** Run `rostopic echo -n1 <wpilib node ns>/npx/navpose`.
- With the Velocity group live (a real robot, or test mode), `has_velocity` is
  true and `time_velocity` is non-zero.
- On a real robot, stop the Velocity group (stop publishing it, or set `valid`
  false). Within about 1 s, `has_velocity` turns false, and `x_m_per_sec` and
  `heading_m_per_sec` both read `-999`. `has_position` stays true.

**Auto Move side.** The getter has no ROS surface, so add a temporary line at the
top of `updaterCb`:

```python
        self.msg_if.pub_info("TEMP robot velocity: " + str(self.get_robot_velocity_dict()), throttle_s = 5.0)  # TEMP -- remove before commit
```

Then check:

1. **Test mode.** `speed_valid` is true, `speed_mps` is `0.0`, and `age_s` is
   under 1.0. Zero is correct here: test mode never moves.
2. **Frame check (real robot; needs Task 1 implemented on the RoboRIO).** Drive
   forward at about 1 m/s, turn about 90°, and drive forward again. Both times,
   `x_mps` should be about `+1.0` and `y_mps` about `0.0`. If x and y swap or
   flip sign after the turn, the frames don't match: the RoboRIO is publishing
   robot-relative values, or rotating with an angle other than the one it
   publishes as `yaw_rad`.
3. **Signs.** Strafing left gives a positive `y_mps`. Rotating counter-clockwise
   gives a positive `yaw_degps`.
4. **Robot selection.** Change the selected robot. Every flag goes false until
   the new robot's NavPose arrives.
5. **Dead publisher.** Disable the WPILib app. Every flag goes false within about
   1 s.

Remove the TEMP line.

---

## Where it breaks

| Symptom | Look at |
|---|---|
| Auto Move logs no "Registering to robot navpose topic" | `DeviceRBXStatus.navpose_topic` is empty. Plan 1 step A4 isn't deployed, or `rbx_enabled` is off, or Position and Orientation are both dead, so no NavPose exists yet. |
| `has_velocity` is false while the robot is clearly moving | The Velocity group is not live. Check that `valid` is true and that `timestamp` advances on every publish. |
| `has_velocity` is always false and `x_m_per_sec` always `-999`, even in test mode | Task 2 isn't running, so the WPILib app never sets the flag and the False default wins. Check the pid changed after the deploy. |
| `speed_valid` is true but `body_valid` is false | Orientation isn't live. That is expected behavior: speed only. |
| Body velocity swings around as the robot turns | Frame mismatch. See Task 5, check 2. |
| `yaw_degps` is always `None` | Orientation isn't live. `/NEPI/Velocity/angular_velocity_radps` is never used. |
| `yaw_degps` is `0.0` while turning | Someone set a mount transform on the WPILib device's `npx`. Clear it. |
| Nothing arrives after a rebuild; rospy logs an md5sum mismatch | Some node is still running the old NavPose definition. Restart all of NEPI. |
| A stall check fires in test mode | Expected. Test mode reports zero velocity by design. |
