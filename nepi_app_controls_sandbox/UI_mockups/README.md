# UI mockups — nepi_app_controls_sandbox

Five standalone style guides, each paired with one full-page concept that applies it to this
app's real RUI panel. Nothing here is wired into the running app and no file in `rui/` was
changed to produce them — every `.html` file below is self-contained (inline CSS, no build step,
no ROS connection) and opens directly in a browser.

This replaces the previous `ui_mockups/` folder (lowercase), which explored five different
*layouts* of the same visual language (card dashboard, dense table, tabbed groups, mobile
accordion, dark telemetry). This round instead explores five different *visual languages* (style
guides) first, each with its own color palette, type scale, spacing, and component rules, and
only then shows one representative layout per guide. See `docs/UI_Redesign_2026.md` for the full
write-up of this approach.

Every concept page carries the same functional surface as the real app: the ten `CONTROL_TYPES`
from `createControlsInitDict()` and the eight `DATUM_TYPES` from `createDataInitDict()` in
`scripts/controls_sandbox_app_node.py`, plus the image-connect selector and viewer from
`NepiAppControlsSandbox.js`. Only the presentation differs between them.

## How to use this folder

Open any `style_guide_*.html` file to see that guide's palette, type scale, spacing scale, and
component states in isolation — no layout opinions, just the design tokens and the vocabulary of
UI parts. Then open the matching `concept_*.html` file to see that same guide applied to a full
page: the connections/image-viewer column, the ten controls, and the eight read-only data fields.

| # | Style guide | Concept | One-line character |
|---|---|---|---|
| 1 | [style_guide_1_field_industrial.html](style_guide_1_field_industrial.html) | [concept_1_field_industrial.html](concept_1_field_industrial.html) | Daylight-readable, thick borders, mono labels — an evolution of NEPI's current tokens |
| 2 | [style_guide_2_soft_neumorphic.html](style_guide_2_soft_neumorphic.html) | [concept_2_soft_neumorphic.html](concept_2_soft_neumorphic.html) | Single-surface, soft dual shadows, calm indoor console |
| 3 | [style_guide_3_brutalist_terminal.html](style_guide_3_brutalist_terminal.html) | [concept_3_brutalist_terminal.html](concept_3_brutalist_terminal.html) | Monospace-only, pure black/white, terminal-session energy |
| 4 | [style_guide_4_glass_console.html](style_guide_4_glass_console.html) | [concept_4_glass_console.html](concept_4_glass_console.html) | Frosted glass over a dark gradient, glow-based state, mission-control mood |
| 5 | [style_guide_5_playful_flat.html](style_guide_5_playful_flat.html) | [concept_5_playful_flat.html](concept_5_playful_flat.html) | Bright saturated colors, big rounded cards, consumer-app friendliness |

## 1. Field Industrial

Keeps NEPI's existing hue family (steel greys, amber/green/red) but pushes every edge toward
maximum legibility for outdoor/glove use: 2–3px borders, all-caps monospace labels for anything
numeric or stateful, and a warm paper background instead of pure white to cut glare. The concept
page lays the app out close to the real current structure — image column at left, a tab group
(Choices/Actions/Values/Data) at right — so it reads as a refinement of what exists today rather
than a replacement of it.

## 2. Soft Neumorphic

Every element is the same material as the background, differentiated only by light and shadow:
raised at rest, inset when active or read-only. The concept page uses a card-grid dashboard
(inspired by the previous round's card-dashboard layout, redrawn in the neumorphic material)
because that layout's discrete tiles suit a single-surface language better than one long bordered
list would.

## 3. Brutalist Terminal

Monospace only, pure black rules on white, zero color except one red accent reserved exclusively
for things the app's own README documents as broken (`Demo Trigger`, `Demo Floats Slider`). The
concept page uses a dense two-table layout (controls table, data table) — the previous round's
dense-table concept, redrawn — because a literal grid of rules is the most direct expression of
this guide's "no ambiguity" principle.

## 4. Glass Console

Translucent panels over a fixed dark radial-gradient background; state is communicated with a
glow (box-shadow bloom in cyan/violet/green/red), not just a color swap, so it reads at a glance
in a dark room. The concept page borrows the previous round's dark-telemetry idea — a strip of
glowing data tiles above the controls — because this guide's whole premise is that light itself
carries meaning, which a telemetry-tile treatment shows off directly.

## 5. Playful Flat

Bright saturated colors, generous rounded cards, and a friendly rounded sans typeface aimed at a
non-engineer operator rather than a technician. The concept page uses an accordion layout with
live-value summary chips on each collapsed header (an idea carried over from the previous round's
mobile-accordion concept) because collapsing detail behind a friendly one-line summary is exactly
the kind of simplification this guide's "approachable to a first-time visitor" principle calls
for.

## Known gaps carried over from the real app

Two things visible in these mockups are faithful to bugs documented in the app's own `README.md`
and `EDITING_GUIDE.md`, not mockup errors: `Demo Floats Slider` (`FloatSliders`) is marked in
every concept as dropped/broken by the SDK, and `Demo Trigger` is marked as unable to fire from
the RUI (message-type mismatch in `ControlsIF`). Every concept flags both rather than hiding them,
so none of the five accidentally implies the underlying SDK issue has been fixed.

## Design tokens referenced

Color, spacing, and type values in "Field Industrial" (guide 1) map directly to the real
`Styles.js` tokens (`grey1 #a5abb4`, `blue #00a5ed`, `green #228b22`, `red #a52a2a`, spacing scale
4/8/16/24/38px). Guides 2–5 intentionally diverge from those tokens — the point of drawing five
considerably different languages is to compare options, not to assume the current one is settled.
