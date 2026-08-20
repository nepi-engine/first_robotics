# Controls Sandbox UI Redesign — Style Guide Exploration (Aug 2026)

## What this is

Five distinct visual style guides for `nepi_app_controls_sandbox`'s RUI panel, each paired with a
full concept mockup that applies that guide to the app's real layout. All ten files are static
HTML — no build step, no ROS connection — in
[`UI_mockups/`](../UI_mockups/). This replaces the app's previous `ui_mockups/` folder, which
compared five *layouts* of one visual language; this round instead compares five *visual
languages*, each demonstrated with one representative layout.

Nothing in `rui/` changed. This is a design-exploration artifact, not an implementation — see
[Status and next steps](#status-and-next-steps) for what promoting one of these to real code would
involve.

## Why style guides first, not just more layout concepts

The app's real components (`NepiAppControlsSandbox.js` and its `-Controls`/`-Data`/`-Settings`/
`-Tabs` children) already had five layout explorations behind them, all sharing NEPI's current
color tokens, type scale, and component shapes from `Styles.js`. The ask this time was different:
hold the layout question loosely and instead ask what the app could look like under a genuinely
different design language — different color families, different corner treatments, different
typographic voice, different metaphors for state (a glow vs. a color swap vs. a shadow depth).
Each guide is documented on its own terms — palette, type scale, spacing scale, and every
component's states — before it's ever applied to a page, the same way a design system is usually
authored independent of any one screen.

## The functional surface every concept covers

Every concept mockup renders the same 18 demo fields the real app defines in
`scripts/controls_sandbox_app_node.py`, so the five are comparable on equal footing:

**10 controls** (`createControlsInitDict()`): Demo Menu (Menu), Demo Selection (Selection), Demo
Selections (Selections), Demo Trigger (Trigger), Demo Bool (Bool), Demo String (String), Demo Int
(Int, 0–10), Demo Float (Float, 0–10), Demo Float Slider (FloatSlider, 0–100), Demo Floats Slider
(FloatSliders, dual-handle 0–1).

**8 data fields** (`createDataInitDict()`, read-only, driven at 1 Hz): Demo Bool, Demo Bools, Demo
String (wall-clock timestamp), Demo Strings, Demo Int (counter), Demo Ints, Demo Float (sine
wave), Demo Floats.

**Plus** the image-connect selector and viewer wired through `ConnectImageIF` /
`Nepi_IF_ConnectData`, matching the pattern in `nepi_app_auto_move`.

Two known SDK bugs, documented in the app's own `README.md` and `EDITING_GUIDE.md`, are flagged
in every concept rather than hidden: `Demo Trigger` cannot fire from the RUI (a message-type
mismatch in `ControlsIF`), and `Demo Floats Slider` is silently dropped by the SDK. Showing five
different visual treatments of a broken control would be misleading if any of them looked like it
worked.

## The five guides

### 1. Field Industrial
Daylight-readable and glove-operable. Keeps NEPI's existing hue family (steel greys, amber/green/
red safety colors) but pushes every border to 2–3px, sets numeric and stateful labels in
uppercase monospace, and swaps pure white for a warm paper background to cut glare. This is
framed as an evolution of the tokens already in `Styles.js`, not a replacement of them, and its
concept keeps the closest layout resemblance to the app as it looks today (image column left, tab
group right).

### 2. Soft Neumorphic
A single continuous surface where depth comes only from dual-direction soft shadow — raised for
interactive elements at rest, inset for anything read-only or currently active. Aimed at an indoor
lab-bench console rather than a field device, trading urgency for calm. Its concept uses a
card-grid dashboard, since discrete tiles suit this single-material language better than one long
bordered list.

### 3. Brutalist Terminal
Monospace type exclusively, pure black rules on white, zero color except one red accent reserved
for broken/destructive items only. Built on the idea that an engineer who already lives in a
terminal all day shouldn't have the RUI fight that mental model. Its concept renders all 18 fields
as two dense tables (controls, data) — a literal grid of rules, the most direct expression of the
guide's "no ambiguity" principle.

### 4. Glass Console
Translucent panels floating over a fixed dark radial-gradient background, with state shown as a
glow (a box-shadow bloom in cyan/violet/green/red) rather than a plain color swap, since color
alone reads poorly in the dark. Aimed at night operations or a dim control room. Its concept adds
a strip of glowing telemetry tiles above the controls to show the guide's light-carries-meaning
premise directly.

### 5. Playful Flat
Bright saturated colors, generous rounded cards, and a friendly rounded typeface — the only guide
of the five aimed at a non-engineer visiting the RUI for the first time rather than a technician.
Technical field names (e.g. `demo_float_slider`) are demoted to small print under a plain-language
label. Its concept uses an accordion layout with a live-value summary chip on each collapsed
header, so state is readable without expanding anything.

## Design approach and constraints

- **No build step.** Every file is inline HTML/CSS with no external dependencies, matching the
  previous mockup round's approach, so anyone can open a file directly in a browser without
  installing anything.
- **Same data, five languages.** Holding the 18 fields and the image-connect flow constant across
  all five is what makes the comparison fair — only palette, type, spacing, and component shape
  vary.
- **Guide before layout.** Each `style_guide_*.html` documents palette, type scale, spacing scale,
  and every component's states (select, text input, editable vs. read-only, button, toggle,
  multi-select, boolean indicator, slider, status indicator) in isolation, so the guide can be
  judged on its own vocabulary before seeing it used on a page.
- **Known bugs stay visible.** Both documented SDK issues (`Demo Trigger`, `Demo Floats Slider`)
  are flagged in-place in every concept instead of being quietly omitted or shown as if fixed.

## Status and next steps

These are comparison artifacts, not a decision. Promoting any one guide toward the real app would
mean: picking (or blending) a direction, translating its CSS rules into `Styles.Create()` rule
objects the way `NepiAppControlsSandbox-Tabs.js` already documents doing for the prior round's
tabbed-groups concept, and deciding whether the change is scoped to this one app or proposed as an
update to the shared tokens in `Styles.js` that every NEPI app inherits from. Neither decision was
made as part of this exploration — see [`UI_mockups/README.md`](../UI_mockups/README.md) for the
per-guide file index.
