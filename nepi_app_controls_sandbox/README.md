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
- `Nepi_IF_Controls.js` — the reusable Controls box (one widget per control).
  This component lives in the shared `nepi_rui` source tree
  (`src/rui_webserver/rui-app/src/`) alongside the other controls components, and
  is imported by the main panel; it is not shipped in this app package.
- `Nepi_IF_Data.js` — the reusable read-only Data box (one row per datum).
  Read-only value boxes for floats, ints and strings; `BooleanIndicator` for
  bools; no sliders, no editable inputs, no publish path. Also lives in the
  shared `nepi_rui` source tree and is not shipped in this app package.
- `NepiAppControlsSandbox-Settings.js` — the Controls Settings box (display
  management), a Select-dropdown settings panel (Nepi_IF_Settings pattern) shown
  only in `develop` run mode or when admin mode is set.
