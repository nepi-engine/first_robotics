# Velocity Feedback — Plan 1: engine and producer changes

Repos touched: **nepi_engine_ws** and **nepi_drones**, deployed and built
together. **Do this first.** Plan 2 (`VELOCITY_FEEDBACK_PLAN_2_FIRST_ROBOTICS.md`)
can't start until this plan is built and running.

This plan delivers three things:

1. **NavPose gets a real `has_velocity` flag**, plus `time_velocity`. It defaults
   to **False** and gates `x/y/z_m_per_sec` on its own, the way `has_position`
   gates `x/y/z_m`. Today, velocity is carried whenever position is, so a source
   with no velocity, or a stale one, publishes a confident `0.0 m/s`.
2. **Every producer that really has velocity sets the flag True.** That is
   `rbx_gazebo` in nepi_drivers, plus the five sim drivers and sim_connector in
   nepi_drones. The WPILib app is the one exception; Plan 2 covers it.
3. **An RBX device reports where its pose is.** `DeviceRBXStatus.navpose_topic`
   is empty for every `RBXRobotIF` device today. It will carry the NPX child's
   navpose topic.

## Settled decisions (2026-10-01)

- **The flag defaults to `False`, not "not stated".** A producer that never
  thinks about velocity now publishes "no velocity" instead of a fake value tied
  to position. That means one meaning for every `has_*` flag, no special
  accessor, and a default that can't reintroduce the bug. The cost is this
  coordinated, multi-repo change.
- **The flag gates linear velocity only.** `heading_m_per_sec` and
  `location_m_per_sec` stay under their own flags.
- **The flag is not a capability.** It doesn't go into NPX caps or status, and
  there's no new component topic. The RUI is unchanged.

**What this does to everything else.** Every source that has a position but no
velocity, such as `rbx_ardupilot`, the NPX/PTX/IDX drivers and navpose_mgr's
fixed navposes, will now publish velocity as `-999` instead of `0.0`. That is
correct, and it breaks nothing. I checked the RUI and all Python in
nepi_engine_ws, nepi_drones and first_robotics: **nothing reads navpose
velocity** apart from the producers and the SDK.

**Between this plan and Plan 2,** the WPILib app's velocity reads `-999`. That
is harmless, because nothing consumes it yet.

---

## Part A — nepi_engine_ws

### A1. `src/nepi_interfaces/msg/NavPose.msg`

After line 52 (`float32 z_m_per_sec`), insert:

```
# Linear velocity validity, independent of has_position. When false,
# x/y/z_m_per_sec carry -999. time_velocity is the velocity sample time.
bool has_velocity
float32 time_velocity
```

### A2. `src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_nav.py`

**a. `BLANK_NAVPOSE_DICT`.** After `'z_m_per_sec': 0.0,` (`:249`), add:

```python
    # Linear velocity validity, independent of has_position. A producer with a
    # real velocity sets it True; left False, x/y/z_m_per_sec go out as -999
    # whatever the position is doing.
    'has_velocity': False,
    'time_velocity': 0.0,
```

**b. `clear_navpose_dict_comp`, in the `'position'` branch.** After
`npdata_dict['z_m_per_sec']  = 0.0` (`:368`), add:

```python
          npdata_dict['has_velocity'] = False
          npdata_dict['time_velocity'] = 0.0
```

**c. `update_navpose_dict_from_dict`.** Take the three velocity lines out of the
`has_position` block (`:427-429`), and add a block of their own directly after
it:

```python
        if npdata_dict_new['has_position'] == True:
            npdata_dict_org['has_position'] = True
            npdata_dict_org['time_position'] = npdata_dict_new['time_position']
            npdata_dict_org['x_m'] = npdata_dict_new['x_m']
            npdata_dict_org['y_m'] = npdata_dict_new['y_m']
            npdata_dict_org['z_m'] = npdata_dict_new['z_m']
        # .get, not [] -- navpose_mgr's fixed navposes are reloaded from saved
        # config written before this key existed, and a KeyError here is
        # swallowed by the bare except below, silently dropping the whole merge.
        if npdata_dict_new.get('has_velocity', False) == True:
            npdata_dict_org['has_velocity'] = True
            npdata_dict_org['time_velocity'] = npdata_dict_new.get('time_velocity', 0.0)
            npdata_dict_org['x_m_per_sec'] = npdata_dict_new['x_m_per_sec']
            npdata_dict_org['y_m_per_sec'] = npdata_dict_new['y_m_per_sec']
            npdata_dict_org['z_m_per_sec'] = npdata_dict_new['z_m_per_sec']
```

Use `.get` here and only here. Every other edit reads a dict that has already
been filled from BLANK.

**d. `update_navpose_dict_from_msg`, in the `'position'` /
`nepi_interfaces/NavPosePosition` branch.** After
`navpose_dict['z_m_per_sec'] = msg.z_m_per_sec` (`:692`), add:

```python
          # The component msg carries no velocity flag. Its publisher writes
          # -999 when it has none (data_if.py position_pub), so that is the
          # flag -- otherwise a position-only source's 0.0 would come back
          # through navpose_mgr as a real reading.
          navpose_dict['has_velocity'] = (msg.x_m_per_sec != -999)
          navpose_dict['time_velocity'] = navpose_dict['time_position']
```

Leave the Odometry, Pose and Point branches alone. They don't touch velocity.

**e. `convert_navpose_dict2msg`.** Replace `:1176-1178` with:

```python
      np_msg.has_velocity = (npdata_dict['has_velocity'] == True)
      np_msg.time_velocity = npdata_dict['time_velocity'] if np_msg.has_velocity else 0.0
      np_msg.x_m_per_sec = npdata_dict['x_m_per_sec'] if np_msg.has_velocity else -999
      np_msg.y_m_per_sec = npdata_dict['y_m_per_sec'] if np_msg.has_velocity else -999
      np_msg.z_m_per_sec = npdata_dict['z_m_per_sec'] if np_msg.has_velocity else -999
```

The BLANK-fill loop at `:1142-1144` adds the key before this runs, so direct
indexing is safe here.

**f. `convert_navpose_msg2dict`.** After
`npdata_dict['z_m_per_sec'] = np_msg.z_m_per_sec` (`:1244`), add:

```python
    npdata_dict['has_velocity'] = np_msg.has_velocity
    npdata_dict['time_velocity'] = np_msg.time_velocity
```

### A3. `src/nepi_engine/nepi_api/src/nepi_api/data_if.py`

In `NavPoseIF.publish_navpose`, in the `pub_position` component block, replace
`:732-734` with:

```python
                # The component msg has no velocity flag, so -999 is the flag:
                # nepi_nav.update_navpose_dict_from_msg reads it back that way.
                msg.x_m_per_sec = np_dict['x_m_per_sec'] if np_dict['has_velocity'] else -999
                msg.y_m_per_sec = np_dict['y_m_per_sec'] if np_dict['has_velocity'] else -999
                msg.z_m_per_sec = np_dict['z_m_per_sec'] if np_dict['has_velocity'] else -999
```

`np_dict` is built from `BLANK_NAVPOSE_DICT` (`:652`), so the key always exists,
and a producer's flag is carried through by the copy loop with no extra code.

### A4. `src/nepi_engine/nepi_api/src/nepi_api/device_if_rbx.py`

In `publish_status`, directly after the ready guard (`:2133-2134`,
`if self.ready == False: return`), add:

```python
        # The navpose belongs to the NPX child, not to this class, so mirror its
        # topic or DeviceRBXStatus.navpose_topic stays empty and a consumer
        # holding only the RBX device cannot find the robot's pose. Copied on
        # every publish: NPX creates its NavPoseIF late, on first telemetry.
        if self.npx_if is not None:
            self.status_msg.navpose_topic = self.npx_if.status_msg.navpose_topic
```

### A5. `src/nepi_drivers/rbx_drivers/rbx_gazebo_node.py`

After `self.navpose_dict['z_m_per_sec'] = 0.0` (`:892`), add:

```python
    # A real velocity from the sim, so say so: NavPose.has_velocity defaults
    # False and gates x/y/z_m_per_sec on its own.
    self.navpose_dict['has_velocity'] = True
    self.navpose_dict['time_velocity'] = now
```

### A6. `CLAUDE.md`, DECISION LOG

```
2026-10 — NavPose gained has_velocity / time_velocity, defaulting False and independent of has_position — Velocity was carried only under has_position, so any source with a position but no velocity, or a stale velocity, published a confident 0.0 m/s (first hit: the WPILib app, where Velocity and Position are separate NetworkTables groups). has_velocity now gates x/y/z_m_per_sec on its own, exactly as has_position gates x/y/z_m. Defaults False in BLANK_NAVPOSE_DICT, so a producer that never sets it publishes velocity as -999 — chosen over a "not stated, follow has_position" fallback because the fallback makes every future producer reintroduce the fake-zero bug by default, needs a special accessor and a pass-through in NavPoseIF, and "migrate when touched" never finishes. Every producer with a real velocity sets it True: rbx_gazebo (nepi_drivers); rbx_sim, rbx_gazebo, rbx_webots, rbx_webots_quadcopter, rbx_mujoco and nepi_app_sim_connector (nepi_drones); the WPILib app (first_robotics). A NEW producer that has velocity must set has_velocity True and time_velocity, or its velocity is silently dropped. Position-only sources (rbx_ardupilot, NPX/PTX/IDX drivers, navpose_mgr fixed navposes) now report -999 instead of 0.0; nothing reads navpose velocity, checked across all three repos. The NavPosePosition component msg has no flag, so its publisher writes -999 when has_velocity is False and update_navpose_dict_from_msg infers the flag from that. update_navpose_dict_from_dict reads it with .get(), because navpose_mgr reloads fixed navposes saved before the key existed and its bare except would otherwise swallow the KeyError and drop the merge. Gates linear velocity only; not a capability flag; RUI unchanged. The NavPose md5 changed: nepi_interfaces and first_robotics nepi_app_obstacles (whose msgs embed NavPose) must be regenerated, and nothing may run on the old definition. Also: RBXRobotIF now mirrors its NPX child's navpose_topic into DeviceRBXStatus.navpose_topic on every status publish (it was always empty).
```

---

## Part B — nepi_drones

### B1. The five sim drivers in `src/nepi_drivers/rbx_drivers/`

In each one, directly after the `self.navpose_dict['z_m_per_sec'] = ...` line,
add the same two lines plus a comment. Every one of these functions already has
`now`, the same value it writes to `time_position`.

| File | Insert after line |
|---|---|
| `rbx_gazebo_node.py` | `:892` |
| `rbx_sim_node.py` | `:1504` |
| `rbx_webots_node.py` | `:863` |
| `rbx_webots_quadcopter_node.py` | `:803` |
| `rbx_mujoco_node.py` | `:816` |

```python
    # A real velocity from the sim, so say so: NavPose.has_velocity defaults
    # False and gates x/y/z_m_per_sec on its own.
    self.navpose_dict['has_velocity'] = True
    self.navpose_dict['time_velocity'] = now
```

### B2. `src/nepi_apps/nepi_app_sim_connector/scripts/sim_connector_app_node.py` — `navPoseMirrorCb`

Take the three velocity lines out of the `if msg.has_position:` block
(`:2478-2480`), and add a block of their own directly after it:

```python
    if msg.has_velocity:
      nd['has_velocity'] = True
      nd['time_velocity'] = msg.time_velocity
      nd['x_m_per_sec'] = msg.x_m_per_sec
      nd['y_m_per_sec'] = msg.y_m_per_sec
      nd['z_m_per_sec'] = msg.z_m_per_sec
```

### B3. Same file — `processTelemetryLine`

Take the three velocity lines out of the position block (`:3801-3803`), and add
a block of their own directly after it. It is gated by presence of the keys,
like every other block in this function:

```python
    if any(k in telem for k in ('x_m_per_sec', 'y_m_per_sec', 'z_m_per_sec')):
      nd['has_velocity'] = True
      nd['time_velocity'] = now
      nd['x_m_per_sec'] = float(telem.get('x_m_per_sec', nd['x_m_per_sec']))
      nd['y_m_per_sec'] = float(telem.get('y_m_per_sec', nd['y_m_per_sec']))
      nd['z_m_per_sec'] = float(telem.get('z_m_per_sec', nd['z_m_per_sec']))
```

The three bridges (gazebo, gazebo_quadcopter, webots) already send these keys,
so **they need no edit**.

### B4. `src/nepi_api/data_if.py` — mirror of A3

Make the same replacement as A3, at `:1520-1522`.

### B5. `src/nepi_api/device_if_rbx.py` — mirror of A4

Make the same insertion as A4, directly after the ready guard at `:2159`.

**Why B4 and B5 are needed:** `deploy_nepi_source.sh` rsyncs nepi_drones'
`src/` to `/mnt/nepi_config/system_cfg/src`. If that tree overlays the engine,
these copies replace the engine's `data_if.py` and `device_if_rbx.py`. Mirroring
the edits makes the result the same either way.

### B6. `nepi_sample_auto_scripts/tests/test_navpose_set_fixed_config_script.py`

`_FakeNavPose` uses `__slots__`, so once the SDK sets `has_velocity` it raises
AttributeError. In `_NAVPOSE_FIELDS` (`:189`), change

```python
        "x_m_per_sec", "y_m_per_sec", "z_m_per_sec",
```

to

```python
        "x_m_per_sec", "y_m_per_sec", "z_m_per_sec",
        "has_velocity", "time_velocity",
```

Also add the same two names to the field list in the comment block above it
(`:167`).

---

## Part C — first_robotics

There are no code edits in this plan; the WPILib app's flag is Plan 2. The
first_robotics apps still have to be **synced into the build workspace before
you build** (step 4 below). `nepi_app_obstacles`'s `Obstacles.msg` and
`ObstaclesDepthMap.msg` embed `NavPose` and must be regenerated, and the
`/mnt/.../src/nepi_apps/` copies are untracked and may be stale.

---

## Deploy and build — one pass

1. **Commit.**
   - nepi_engine_ws: commit in `src/nepi_interfaces` (A1), `src/nepi_engine`
     (A2–A4) and `src/nepi_drivers` (A5). Then commit `CLAUDE.md` (A6) and the
     three pointers in the superproject.
   - nepi_drones: commit B1–B6.
   - Push using the PUSH EDITS workflow when you're ready.
2. **Deploy nepi_engine_ws with `nepidpl`** (`deploy_nepi_complete.sh`), **not**
   `repodpl`. `repodpl` only carries `nepi_engine`, `nepi_rui` and
   `nepi_interfaces`, and A5 is in `nepi_drivers`. Confirm it landed:

   ```
   cd /mnt/nepi_storage/nepi_src/nepi_engine_ws/src
   grep -c has_velocity nepi_interfaces/msg/NavPose.msg nepi_engine/nepi_sdk/src/nepi_sdk/nepi_nav.py \
     nepi_engine/nepi_api/src/nepi_api/data_if.py nepi_drivers/rbx_drivers/rbx_gazebo_node.py
   grep -c "npx_if.status_msg.navpose_topic" nepi_engine/nepi_api/src/nepi_api/device_if_rbx.py
   ```

   Every count should be non-zero.
3. **Deploy nepi_drones** with its `deploy_nepi_source.sh`. Confirm with
   `grep -rl has_velocity /mnt/nepi_config/system_cfg/src`, which should list
   the five drivers, sim_connector and `data_if.py`.
4. **Sync the first_robotics apps.** Run each app's `deploy_app.sh` from the
   current first_robotics checkout. At minimum run it for `nepi_app_obstacles`.
5. **Build** with the catkin code build (`codebld` / `build_nepi_code.sh`).
   `nepibld` isn't needed: it rebuilds the Docker filesystem and wipes `etc`.
6. **Restart all of NEPI**, including rosbridge. Nothing may keep running on the
   old NavPose definition: rospy logs one md5sum mismatch, and that subscriber
   gets nothing after that.

## Verify

1. **The message has the fields.** Run
   `rosmsg show nepi_interfaces/NavPose | grep velocity`. It should list
   `has_velocity` and `time_velocity`.
2. **The converter and merge.** Run this on the device inside the NEPI
   environment, with NEPI running:

   ```python
   import copy
   from nepi_sdk import nepi_sdk, nepi_nav
   nepi_sdk.init_node('navpose_velocity_check', disable_signals=True)
   B = nepi_nav.BLANK_NAVPOSE_DICT

   d = copy.deepcopy(B); d['has_position'] = True; d['x_m_per_sec'] = 1.5
   m = nepi_nav.convert_navpose_dict2msg(copy.deepcopy(d))
   assert m.has_position and not m.has_velocity and m.x_m_per_sec == -999   # unset = no velocity

   d['has_velocity'] = True; d['time_velocity'] = 5.0
   m = nepi_nav.convert_navpose_dict2msg(copy.deepcopy(d))
   assert m.has_velocity and m.x_m_per_sec == 1.5 and m.time_velocity == 5.0

   v = copy.deepcopy(B); v['has_velocity'] = True; v['time_velocity'] = 7.0; v['x_m_per_sec'] = 2.0
   m = nepi_nav.convert_navpose_dict2msg(copy.deepcopy(v))
   assert m.has_velocity and m.x_m_per_sec == 2.0 and m.x_m == -999        # velocity without position

   back = nepi_nav.convert_navpose_msg2dict(m)
   assert back['has_velocity'] is True and back['time_velocity'] == 7.0

   old = copy.deepcopy(B); del old['has_velocity']; del old['time_velocity']
   old['has_position'] = True; old['x_m'] = 3.0                             # config saved before the key existed
   merged = nepi_nav.update_navpose_dict_from_dict(copy.deepcopy(B), old)
   assert merged['x_m'] == 3.0 and merged['has_velocity'] is False          # merge still applied
   print('navpose velocity OK')
   ```

3. **Every velocity producer you can run.** For each one, run
   `rostopic echo -n1 <node ns>/npx/navpose`:
   - **`rbx_gazebo`** (both repos), **`rbx_sim`**, **`rbx_webots`**,
     **`rbx_webots_quadcopter`**, **`rbx_mujoco`**: `has_velocity: True`, and
     the velocity values match what they read before the change.
   - **sim_connector:** check both paths if you can, a bridge connection and the
     navpose mirror. Both should show `has_velocity: True`.

   A producer that should be True but reads False is a missed edit. It fails
   silently otherwise, so don't skip this step.
4. **Position-only source.** Pick one, such as `rbx_ardupilot` or any NPX device
   without velocity. It should show `has_velocity: False` and
   `x_m_per_sec: -999`. That is expected.
5. **navpose_mgr.** The fused navpose shows `has_velocity` matching its position
   source. A frame with a saved fixed navpose still applies it, with no
   `NavPose Dict update` warnings in the navpose_mgr log.
6. **RBX status.** Run
   `rostopic echo -n1 <node ns>/rbx/status | grep navpose_topic`. With telemetry
   flowing it should read `<node ns>/npx/navpose`.
7. **The nepi_drones test script** (B6) runs clean.
8. **Hand off.** Tell the first_robotics developer that Plan 2 can start.

---

## Spotted, not in scope

- `nepi_nav.py:689` sets `navpose_dict['z_m'] = msg.y_m`, so z is taken from y.
- `transform_navpose_dict` reads `x_deg`/`y_deg`/`z_deg` (`:1010-1014`), keys
  that don't exist. Any non-blank mount transform therefore throws halfway
  through, after it has already zeroed the angular rates (`:1002`).
