# UI mockups — nepi_app_controls_sandbox

Static, self-contained HTML pages (no build step, no ROS connection). They exist to compare UI
directions for this app's RUI panel. Nothing here is wired into the running app, and no file in
`rui/` was changed to produce them — open any `.html` file directly in a browser.

Every page carries the same functional surface as the real app: the ten `CONTROL_TYPES` from
`createControlsInitDict()` and the eight `DATUM_TYPES` from `createDataInitDict()` in
`scripts/controls_sandbox_app_node.py`. Only the presentation differs between pages.

## baseline_current.html

Reproduces today's actual RUI as closely as static HTML allows: the 75% / 2% / 23% image-viewer
+ sidebar split from `NepiAppControlsSandbox.js`, the bordered `Section` boxes and flex `Label`
rows from `Section.js` / `Label.js`, and the real color tokens from `Styles.js`
(`grey1 #a5abb4` borders, `blue #00a5ed` accents, `green #228b22` confirmed-state, white
background, uppercase bold section titles). This is the reference point every concept below is
compared against, not a redesign itself.

## concept_1_card_dashboard.html — Card dashboard

Regroups the same controls into labeled card groups (Choices, Toggles & Actions, Values,
Sliders, Live Data) in a responsive grid with rounded cards and a light shadow, instead of one
long bordered list. Places to look for ideas: the grouping itself borrows nothing from this
codebase — it's the general IoT/smart-home dashboard convention (tile-per-setting, grouped by
function) — but every card's internal control (`<select>`, text input, `AsyncToggle`-style pill)
keeps the same field names and topics as `NepiAppControlsSandbox-Controls.js`, so the regrouping
is presentation-only.

## concept_2_dense_table.html — Dense table

All 18 controls and data in two compact tables (name / type / value), monospace, dark, one row
per item — trades the current per-item bordered box for scan-ability across everything at once.
Places to look for ideas: `nepi_app_onvif_mgr` (see `src/nepi_apps/CLAUDE.md`) already manages a
list of discovered devices in table-like form and is the closest in-repo precedent for
multi-item tabular display; this concept extends that idea to controls/data instead of devices.

## concept_3_tabbed_groups.html — Tabbed groups

Splits the sidebar into four tabs (Choices, Actions, Values, Data) instead of stacking every
Section vertically. Places to look for ideas: `nepi_app_stereo_cam/rui/` already splits one app's
configuration across separate files/panels (`NepiAppStereoCam-Calibration.js`,
`-CalibrationTarget.js`, `-Advanced.js`) — a real in-repo precedent for dividing one app's
controls into named sub-panels, which this concept turns into literal clickable tabs.

## concept_4_mobile_accordion.html — Mobile accordion

Single-column, phone-width frame; each functional group collapses into an accordion whose
closed header shows a live-value chip (e.g. "Low · Bravo · 2 sel") so an operator can read state
without expanding anything. Places to look for ideas: this directly extends a pattern already in
the codebase — the single global "Show Controls" / "Show Data" `Toggle` in
`NepiAppControlsSandbox-Controls.js` / `-Data.js` — from one page-wide toggle into one toggle per
functional group, and is otherwise the standard mobile-accordion convention for field/handheld
use (this app's README and CLAUDE.md both note NEPI targets field deployment on varied hardware).

## concept_5_dark_telemetry.html — Dark telemetry console

Inverts the current priority: Data Sandbox becomes large, tabular-figure readouts with trend
sparklines across the top, and Controls Sandbox collapses into a `<details>` drawer below, for a
direction where monitoring is the operator's primary task and adjusting controls is occasional.
Places to look for ideas: the tile color rule (blue = pending/unconfirmed, green = confirmed)
is not invented — it's the exact `AsyncToggle` semantic already documented in
`EDITING_GUIDE.md` section 5.6.5 ("the thumb moves the moment you click; the colour only follows
a confirmed value from the node"), carried over into a dashboard-tile idiom instead of a switch.
The overall look (dark background, large numerals, sparklines) follows the general
mission-control/telemetry-dashboard convention, which is not present elsewhere in this codebase.

## Known gaps carried over from the real app

Two things visible in these mockups are faithful to bugs documented in the app's own README
and EDITING_GUIDE.md, not mockup errors: `Demo Floats Slider` (`FloatSliders`) is marked as
broken/dropped by the SDK in every mockup that shows it, and `Demo Trigger` is marked as
non-firing from the RUI in the baseline and concept 2 for the same reason (`ControlsIF`
subscribes the wrong message type). Concept 3's note points at the documented fix (the
plain-app-topic pattern `nepi_app_stereo_cam` already uses for its Reload Processes button).
