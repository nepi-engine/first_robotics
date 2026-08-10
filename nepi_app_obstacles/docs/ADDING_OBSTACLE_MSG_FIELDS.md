# Adding Fields to the Obstacle Messages

How the obstacle output pipeline is built in this app, and the procedure for adding a field to
either of its two messages. Everything below refers to files as they stand in this package.

## The pipeline as built

An obstacle makes four hops:

| Hop | File | Symbol |
|---|---|---|
| Originates | `scripts/nepi_obstacles.py` | `process_1()` / `process_2()` |
| In flight | `scripts/nepi_obstacles.py` | a dict shaped by `OBSTACLE_DICT` |
| Becomes a message | `scripts/obstacles_app_node.py` | `convertObstaclesMsg()` |
| Published | `scripts/obstacles_app_node.py` | `updateProcess()` |

A process function is called once per cycle at `PROCESS_UPDATE_RATE_HZ` (5.0 Hz,
`obstacles_app_node.py`). It puts a list of `OBSTACLE_DICT`-keyed dicts into
`process_data_dict['obstacles']` and the count into `process_data_dict['obstacle_count']`.
`updateProcess()` then calls `convertObstaclesMsg()`, which turns that list into one
`nepi_app_obstacles/Obstacles` message holding one `nepi_app_obstacles/Obstacle` per dict, and
publishes it through the `obstacles_pub` entry of `PUBS_DICT`.

Published topic: `<base_namespace>/app_obstacles/obstacles`, type `nepi_app_obstacles/Obstacles`,
qsize 1, **not latched** — it is a streaming product, not state.

The process module stays plain dicts on purpose. `obstacles_app_node.py` re-imports it on the
`reload_processes` path, so it must stay importable where the generated message types are not on
the path. All message construction lives in the node.

## Adding a field to Obstacle.msg

One `Obstacle` is one detected obstacle. Numbered steps:

1. **Edit `msg/Obstacle.msg`.** Add the field. A type from another package must be written
   package-qualified (`nepi_interfaces/NavPose`); a type from *this* package is written bare
   (`Obstacle[]`).
2. **Add the matching key to `OBSTACLE_DICT`** in `scripts/nepi_obstacles.py`. Same name as the
   message field. Default it to the unset value for its type (see below).
3. **Fill it in whichever process functions can supply it.** `process_2` builds its obstacle dicts
   from `get_blank_obstacle_dict()`; assign the new key there. If a process cannot supply the
   field, leave it at its unset default — do not substitute.
4. **Assign it in `convertObstaclesMsg()`** in `scripts/obstacles_app_node.py`, in the
   `for obstacle_dict in obstacle_dicts:` loop, one line beside the existing five:
   ```python
   obstacle_msg.my_field = obstacle_dict.get('my_field', nepi_obstacles.FLOAT_FIELD_UNSET)
   ```
5. **Rebuild**, so the message regenerates:
   ```bash
   cd ~/nepi_engine_ws && catkin_make --pkg nepi_app_obstacles
   ```
   Until this runs, no node can import the new field. Build the whole workspace
   (`./build_nepi_code.sh`) if `nepi_interfaces` also changed.
6. **Confirm it on the live topic:**
   ```bash
   rosmsg show nepi_app_obstacles/Obstacle
   rostopic echo -n1 /<prefix>/<device_id>/app_obstacles/obstacles
   ```

Sanity check before shipping: byte-compile both scripts and confirm the three lists agree —
`Obstacle.msg` fields, `OBSTACLE_DICT` keys, and the assignments in `convertObstaclesMsg()`.

## Adding a field to Obstacles.msg

One `Obstacles` is one processing cycle. Same procedure with one difference: **the node fills this
one, not a process function.** There is no dict hop — skip steps 2 and 3, and assign the field
directly in `convertObstaclesMsg()` beside `obstacles_msg.process_name` and its siblings.

**Which message does a new field belong in?**

> Per cycle goes on `Obstacles`. Per obstacle goes on `Obstacle`.

A source topic, a NavPose, a process name, a cycle timestamp — one value for the whole cycle —
goes on `Obstacles`. A name, a confidence, a range, a position — one value per detected thing —
goes on `Obstacle`.

## Rules that silently break the app

These fail quietly rather than loudly. Each has cost someone an afternoon.

- **A `.msg` file is invisible until it is listed in `add_message_files()`** in `CMakeLists.txt`.
  A message sitting in `msg/` that is not listed is simply never generated, and the import fails
  with a plain `ImportError` that says nothing about the real cause. Both `Obstacle.msg` and
  `Obstacles.msg` were in this state before this pass.
- **A field type from another package needs that package in three places**: `find_package(...
  COMPONENTS)`, `generate_messages(DEPENDENCIES)`, and `catkin_package(CATKIN_DEPENDS)` — plus
  `<build_depend>` / `<exec_depend>` in `package.xml`, which is what orders the workspace build.
  `nepi_interfaces` is there now for the `NavPose` field.
- **Package-qualify a type from another package; do not qualify a sibling.** `nepi_interfaces/NavPose`
  is qualified. `Obstacle[]` inside `Obstacles.msg` is not — it is a sibling in this package.
  Writing `nepi_app_obstacles/Obstacles[] obstacles` is what made this message contain an array of
  itself before this pass.
- **A message change requires a rebuild before any node can import it.** Editing the `.msg` and
  restarting the node is not enough; the generated Python is stale until catkin regenerates it.
- **The two unset constants are duplicated.** `INT_FIELD_UNSET` and `FLOAT_FIELD_UNSET` are
  declared in `msg/Obstacle.msg` *and* redeclared in `scripts/nepi_obstacles.py`. **Edit both.**
  They are not imported from the generated message so the process module stays importable without
  it. Nothing checks that they agree.

## The unset field convention

A field with no value this cycle carries a sentinel, never a plausible substitute:

| Type | Unset value |
|---|---|
| `float32` / `float64` | `FLOAT_FIELD_UNSET` (`-999.0`) |
| `int16` / `int32` | `INT_FIELD_UNSET` (`-999`) |
| `string` | `''` (empty string) |
| `nepi_interfaces/NavPose` | default constructed, with `navpose_frame` empty |

Consumers test against these. `Obstacles.navpose_frame` being empty is specifically how a consumer
detects that no NavPose was attached — a real NavPose always names its frame.

**Leave a field unset rather than substituting a plausible value from a different source.** An
obstacle whose range came from a different sensor than the one it claims is worse than an obstacle
with no range, because nothing downstream can tell the difference.

## Current state — the work list

Be specific about what is and is not filled. As of this pass:

**`process_1` emits no obstacles at all.** It consumes only the depth map and NavPose. A depth
return is a geometry — a range, a bearing, a size, a position — and `Obstacle.msg` declares no
geometry field today. There is no honest way to name, identify or score an obstacle from a depth
map alone. **Adding geometry fields to `Obstacle.msg` and filling them here is the first thing to
do.**

**`process_2` fills every field the message currently declares**, from the targets source:

| Field | Source | Status |
|---|---|---|
| `name` | target `name` | filled |
| `uid` | target `uid` | filled |
| `confidence` | target `confidence` | filled |
| `timestamp` | target `timestamp` (source image epoch seconds) | filled |
| `id` | index in the emitted list | filled, see below |

`id` is the obstacle's index in this cycle's list, **not** the source target's id.
`nepi_interfaces/Target` declares an `id` field but no producer assigns it —
`nepi_api/node_if_ai_detector.py` fills timestamp, name, uid, confidence and the pixel fields and
never id — so it arrives `0` on every target. Index is the only definition that distinguishes the
obstacles in one message today. **It is not stable across cycles.** A tracker assigning a
persistent id is open work.

**Controls that are declared but inert:**

- `use_navpose` (both processes) — asks for obstacle *positions* in the NavPose frame, and
  `Obstacle.msg` has no position field to express one. The NavPose still reaches the consumer on
  the `Obstacles` message.
- `min_obstacle_size_ratio` (`process_1`) — no size field, and `process_1` emits nothing.
- `min_range_m` / `max_range_m` in `process_2` are **live**, not inert:
  `nepi_interfaces/Target` carries `range_m`. But the producer leaves it at `FLOAT_FIELD_UNSET`
  when it had no depth map to measure against, so an unset range *skips* the gate rather than
  failing it — otherwise every obstacle from a detector running without depth would be dropped.
  In `process_1` these two are inert until depth-map fusion lands.

**Not implemented:** depth-map / targets fusion. `process_2` reads the targets source only; the
depth map is passed in and not used.

## RUI note

The app's page is `rui/NepiAppObstacles.js`, and it is **unchanged** by this pass.

Three files — `rui/NepiAppObstacles--Controls.js`, `rui/NepiAppObstacles--Data.js`, and
`rui/NepiAppObstacles-Images.js` — look like a split of that page but are not. They are copies of
a *different app's* components (`nepi_app_pan_tilt_auto`): the first two are byte-identical to each
other and both export `NepiAppPTAutoControls` bound to `nepi_app_pan_tilt_auto/PanTiltAutoAppStatus`,
and the third exports `ImageViewersSelector` and imports `./NepiAppPTAuto-ImageViewerSelector`,
which does not exist in `nepi_rui`. **They do not work and must not be mounted.** Delete them or
replace them with real sections before touching the page.

Any obstacle display is a separate change under `rui/`. It goes through this app's
`params/obstacles_app_params.yaml` `RUI_DICT` (add the file to `rui_files`) and
`build_nepi_rui.sh`. **Never hand-edit the generated registration files** `Nepi_IF_Apps.js` or
`NepiApps.js`.

## Verification checklist

Static checks — the messages are not generated until the app is built into a NEPI image:

- [ ] `python3 -m py_compile scripts/nepi_obstacles.py scripts/obstacles_app_node.py`
- [ ] Every `Obstacle.msg` field is assigned in `convertObstaclesMsg()`
- [ ] Every `OBSTACLE_DICT` key matches an `Obstacle.msg` field name
- [ ] Every `Obstacles.msg` field is assigned in `convertObstaclesMsg()`
- [ ] `INT_FIELD_UNSET` / `FLOAT_FIELD_UNSET` agree between `msg/Obstacle.msg` and
      `scripts/nepi_obstacles.py`
- [ ] No `rospy` import or call was introduced — in this workspace `rospy` is called only inside
      `nepi_sdk/nepi_ros.py`; nodes, apps and drivers use the `nepi_sdk` wrappers
- [ ] Every field type resolves to a ROS primitive or to a package listed in
      `generate_messages(DEPENDENCIES)`
- [ ] New `.msg` files are listed in `add_message_files()`
