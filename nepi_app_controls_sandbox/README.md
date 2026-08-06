# nepi_app_controls_sandbox

Controls Sandbox — a demonstration NEPI app that registers exactly one control of
each `nepi_controls` / `ControlsIF` type through a single `ControlsIF` instance and
renders each one in the RUI with the correct widget.

## Purpose

This app is the first consumer of the `nepi_controls` (SDK) / `ControlsStatus` /
`ControlsIF` (API) pipeline. It exercises all ten `CONTROL_TYPES`:

`Menu`, `Selection`, `Selections`, `Trigger`, `Bool`, `String`, `Int`, `Float`,
`FloatSlider`, `FloatsSlider`.

It is also the first consumer of the read-only `nepi_data` (SDK) / `DataStatus` /
`DataIF` (API) pipeline, exercising all eight `DATUM_TYPES`:

`Bool`, `Bools`, `String`, `Strings`, `Int`, `Ints`, `Float`, `Floats`.

The data values are driven from the `DataIF` updater callback at 1 Hz — a
counter, a sine, a toggling bool and a wall-clock string — so the read-only
display can be seen to be live. The RUI never writes a datum; this node is the
only writer of record.

## ROS interface

- Node: `app_controls_sandbox`
- Published: `.../app_controls_sandbox/status` (`ControlsSandboxStatus`, latched)
- Published: `.../app_controls_sandbox/controls/status` (`nepi_interfaces/ControlsStatus`, latched)
  — one `Control` per registered control, published by the `ControlsIF` instance.
- Published: `.../app_controls_sandbox/data/status` (`nepi_interfaces/DataStatus`, latched)
  — one `Datum` per registered datum, published by the `DataIF` instance.
- Subscribed (per-datum value setters + display management): under
  `.../app_controls_sandbox/data/` — `set_<type>_datum_value`,
  `set_datum_hidden`, `set_data_hidden`, `set_datum_order`, `set_datum_up`,
  `set_datum_down`, `set_datum_top`, `set_datum_bottom`. The RUI publishes to
  none of these.
- Subscribed (per-control value setters + display management): under
  `.../app_controls_sandbox/controls/` — `set_<type>_control_value`,
  `set_control_hidden`, `set_control_display_name`, `set_control_description`,
  `set_control_move`, `set_control_order`, `set_control_reset`,
  `set_control_factory_reset`.

## RUI

- `NepiAppControlsSandbox.js` — main panel.
- `NepiAppControlsSandbox-Controls.js` — the Controls box (one widget per
  control). **App-local copy**, forked from the shared
  `nepi_rui/src/rui_webserver/rui-app/src/Nepi_IF_Controls.js`. Shipped in this
  app package; editing it changes this app only.
- `NepiAppControlsSandbox-Data.js` — the read-only Data box (one row per datum).
  Read-only value boxes for floats, ints and strings; `BooleanIndicator` for
  bools; no sliders, no editable inputs, no publish path. **App-local copy**,
  forked from the shared
  `nepi_rui/src/rui_webserver/rui-app/src/Nepi_IF_Data.js`. Shipped in this app
  package; editing it changes this app only.
- `NepiAppControlsSandbox-Settings.js` — the Controls Settings box (display
  management), a Select-dropdown settings panel (Nepi_IF_Settings pattern) shown
  only in `develop` run mode or when admin mode is set.

## Working on this app

This app owns its Controls box and its Data box outright. Controls in, data out
is the whole point of the sandbox, so both interface components were forked out
of `nepi_rui` and into this package. Change them freely. **Do not edit
`Nepi_IF_Controls.js` or `Nepi_IF_Data.js` in `nepi_rui`** — those are shared,
and `Nepi_IF_Controls.js` has four other consumers (`nepi_app_obstacles`,
`nepi_app_auto_move`, `nepi_app_stereo_cam`, `nepi_app_wpilib_if`).

### Adding a control

Controls are built in `scripts/controls_sandbox_app_node.py`, in
`createControlsInitDict()`. Each entry is one control; the dict's insertion
order sets the initial display order. The whole dict is handed to the single
`ControlsIF` instance in `__init__`, which publishes it as a
`nepi_interfaces/ControlsStatus` at `<node>/controls/status` and subscribes the
`set_<type>_control_value` topics that the RUI publishes to.

To add one more control of an existing type, add one entry to that dict. For a
second Int, for example:

```python
'demo_int_two': {
    'type': 'Int', 'default': 3, 'bounds': [0, 20],
    'display_name': 'Demo Int Two', 'description': 'A second integer.', 'hidden': False},
```

That is the only edit. Nothing in the RUI needs to change — the Controls box
renders whatever the status message carries, one widget per control, keyed by
the control's declared `type`. Read the current value from the node side with
`self.controls_if.get_control_value('demo_int_two')`;
`controlsUpdatedCb(control_name)` fires after every applied change.

### Adding a datum

Data are built the same way, in `createDataInitDict()`, and handed to the single
`DataIF` instance. A datum has a value and a timestamp only — no bounds, no
options, no default or factory value. The values are driven at 1 Hz from
`dataUpdaterCb()`, which is this node's only writer of record; the RUI never
writes a datum.

To add one more datum of an existing type, add one entry to the dict:

```python
'demo_int_two_data': {
    'type': 'Int', 'value': 0,
    'display_name': 'Demo Int Two', 'description': 'Another counter.', 'hidden': False},
```

and one write in `dataUpdaterCb()`:

```python
self.data_if.set_datum_value('demo_int_two_data', count * 2)
```

`set_datum_value()` publishes status on each write, which is why
`dataUpdaterCb()` returns `False`.

### Registering a new RUI file

`RUI_DICT.rui_files` in `params/controls_sandbox_app_params.yaml` is the **one
and only** place a RUI file is registered. Add the filename there and nothing
else. `build_nepi_rui.sh` generates the registration from that list.

**`Nepi_IF_Apps.js` and `NepiApps.js` are generated files. Never hand-edit
them.** An edit there is overwritten by the next RUI build.

Every app's `rui/` directory installs **flat** into the same `nepi_rui` src
folder, so RUI basenames share one global namespace across all app packages. A
duplicate basename from any app silently overwrites. Keep every file in this
`rui/` directory prefixed `NepiAppControlsSandbox-`.

### Known rough edges

Confirmed by reading `nepi_sdk/nepi_controls.py` and `nepi_api/system_if.py`.
These are defects in the shared pipeline, not in this app — they will bite in
any app that uses `ControlsIF`.

- **Avoid the `FloatSliders` type.** Broken in at least three places. The
  `create_controls_dict` branch clamps into an undefined `value[0]`/`value[1]`
  (only `values` exists) and reads an undefined `int_bounds`; `set_control_value`
  reads `value0` before assigning it. The exceptions are swallowed, so the
  control is silently dropped rather than erroring. `demo_floats_slider` is in
  this app's dict for completeness of the type survey, not because it works.
- **`Trigger` cannot be fired from the RUI.** The Controls box sends an
  `UpdateString`, `ControlsIF` subscribes the topic as `UpdateTrigger`, and
  `Store.js` has no `sendUpdateTriggerMsg` to send the right type with. A
  trigger needs its own plain app topic and a `Button`, the way
  `nepi_app_stereo_cam` does its Reload Processes.
- **Runtime hiding does not work.** `nepi_controls.set_control_hidden()` does
  `hidden = str(hidden)`, writing `'True'`/`'False'` into a bool field.
  `hidden` is usable only as authored in the init dict. To suppress a control at
  runtime, do not mount the component.
- **Factory reset behaves exactly like reset.** `reset_control_value()`'s guard
  is `if value_key != 'factory' or value_key != 'default'`, which is always
  true, so `value_key` is forced to `'default'` on every call.
  `ControlsIF.factory_reset()` cannot restore factory values.
- **Construct `ControlsIF` with `node_if=None`.** Passing a shared `node_if`
  takes a branch that reads `self.selected_sources` before it is ever assigned,
  and raises. This app passes nothing, so the IF builds and owns its own
  `NodeClassIF`.
- **Some setters do not force an immediate republish.**
  `ControlsIF.set_control_value()`, `set_control_options()` and
  `set_control_bounds()` write bare `self.publish_status` with no parentheses —
  the call never happens, so the RUI waits out the 1 Hz status timer before the
  change appears. Not a data-loss bug, just a visible lag.
- **Keep every `Float` default inside its own bounds.** The `Float`/`FloatSlider`
  upper-bound clamp reads `float(int_bounds[1])` — the *Int* bounds, a leftover
  from the branch above it. An out-of-bounds Float default picks up a stale Int
  bound from a previously processed control, or raises `NameError` and the
  control is silently dropped.
