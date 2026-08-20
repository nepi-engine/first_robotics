/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */

/*
 * APP-LOCAL Glass Console theme tokens for nepi_app_controls_sandbox.
 *
 * Ports the palette, spacing, and inline-style vocabulary of
 * UI_mockups/style_guide_4_glass_console.html into JS constants every rui/*.js
 * file in this app imports. Scoped to this app only -- it does not touch or
 * replace the shared Styles.js tokens every other NEPI app inherits, matching
 * docs/UI_Redesign_2026.md's note that promoting a guide is a per-app decision
 * unless a shared-token update is separately proposed.
 *
 * Import "./NepiAppControlsSandbox-GlassConsole.css" once (done in
 * NepiAppControlsSandbox.js, the app's single entry point) for the handful of
 * things inline styles cannot reach: react-toggle and rc-slider's own
 * internal markup, both from third-party packages with fixed class names.
 */

const colors = {
  bg0: "#070b16",
  bg1: "#0e1730",
  glass: "rgba(255,255,255,0.06)",
  glassBrd: "rgba(255,255,255,0.14)",
  text: "#e8edf9",
  textDim: "#8a93ab",
  cyan: "#4be3ff",
  violet: "#9d7bff",
  green: "#3ef2a8",
  red: "#ff5d7a",
  dark: "#04101f"
}

const sans = '"Segoe UI", Arial, sans-serif'

const pageBackground = {
  background:
    "radial-gradient(circle at 15% 10%, rgba(75,227,255,0.16), transparent 40%)," +
    "radial-gradient(circle at 85% 30%, rgba(157,123,255,0.14), transparent 45%)," +
    `linear-gradient(160deg, ${colors.bg0}, ${colors.bg1})`,
  color: colors.text,
  fontFamily: sans,
  fontSize: "14px",
  lineHeight: 1.5
}

const glassPanel = {
  background: colors.glass,
  border: `1px solid ${colors.glassBrd}`,
  borderRadius: "16px",
  padding: "16px",
  marginBottom: "18px"
}

const panelCaption = {
  fontSize: "11px",
  textTransform: "uppercase",
  letterSpacing: "1.5px",
  fontWeight: 700,
  color: colors.cyan,
  marginBottom: "12px"
}

const row = {
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
  padding: "9px 0",
  borderBottom: "1px solid rgba(255,255,255,0.06)"
}

const rowName = {
  fontWeight: 600,
  fontSize: "12.5px",
  color: colors.text
}

const rowSub = {
  fontSize: "9px",
  color: colors.textDim
}

const rowFlag = {
  fontSize: "8.5px",
  color: colors.red,
  fontWeight: 700
}

const indField = {
  width: "100%",
  fontFamily: sans,
  fontSize: "13px",
  padding: "8px 10px",
  border: `1px solid ${colors.glassBrd}`,
  borderRadius: "10px",
  background: "rgba(255,255,255,0.04)",
  color: colors.text
}

const btnGlass = {
  border: `1px solid ${colors.glassBrd}`,
  borderRadius: "10px",
  background: "rgba(255,255,255,0.06)",
  color: colors.text,
  padding: "8px 14px",
  fontWeight: 600,
  fontSize: "11px",
  cursor: "pointer",
  width: "100%"
}

const btnGlassBroken = {
  color: colors.red,
  borderColor: "rgba(255,93,122,0.5)"
}

const chip = {
  fontSize: "10px",
  fontWeight: 600,
  padding: "4px 10px",
  borderRadius: "20px",
  border: `1px solid ${colors.glassBrd}`,
  color: colors.textDim,
  cursor: "pointer"
}

const chipOn = {
  color: colors.dark,
  background: colors.violet,
  borderColor: colors.violet
}

const dot = {
  width: "9px",
  height: "9px",
  borderRadius: "50%",
  background: colors.textDim,
  display: "inline-block"
}

const dotOn = {
  background: colors.green,
  boxShadow: `0 0 8px ${colors.green}`
}

// Backdrop frame behind the real image viewer, not a fixed-aspect crop --
// Nepi_IF_ImageViewer (shared, unforkable) is a full light-themed control
// surface with its own buttons and config panels, not a plain <img>, so it
// renders at its natural size on top of this background rather than being
// squeezed into concept_4_glass_console.html's flat 16:9 mockup box.
const viewer = {
  width: "100%",
  minHeight: "80px",
  borderRadius: "14px",
  background:
    "radial-gradient(circle at 30% 30%, rgba(75,227,255,0.08), transparent 60%), #0a1226",
  border: `1px solid ${colors.glassBrd}`,
  padding: "10px",
  marginTop: "12px"
}

const hint = {
  fontSize: "10px",
  color: colors.textDim,
  marginTop: "8px"
}

const tabBar = {
  display: "flex",
  gap: "6px",
  marginBottom: "14px"
}

const tab = {
  flex: 1,
  textAlign: "center",
  fontSize: "11px",
  fontWeight: 700,
  textTransform: "uppercase",
  letterSpacing: "0.5px",
  padding: "8px 4px",
  borderRadius: "10px",
  border: `1px solid ${colors.glassBrd}`,
  color: colors.textDim,
  cursor: "pointer"
}

const tabActive = {
  color: colors.dark,
  background: `linear-gradient(90deg, ${colors.cyan}, ${colors.violet})`,
  border: "none",
  fontWeight: 700
}

const telemetryGrid = {
  display: "grid",
  gridTemplateColumns: "repeat(4, 1fr)",
  gap: "10px"
}

const telemetryTile = {
  ...glassPanel,
  marginBottom: 0,
  padding: "12px",
  textAlign: "center"
}

const telemetryNum = {
  fontSize: "20px",
  fontWeight: 700,
  color: colors.cyan
}

const telemetryLbl = {
  fontSize: "9px",
  color: colors.textDim,
  textTransform: "uppercase",
  marginTop: "4px"
}

const sparkline = {
  height: "2px",
  background: `linear-gradient(90deg, ${colors.cyan}, ${colors.violet})`,
  marginTop: "8px",
  borderRadius: "2px",
  opacity: 0.6
}

export default {
  colors,
  sans,
  pageBackground,
  glassPanel,
  panelCaption,
  row,
  rowName,
  rowSub,
  rowFlag,
  indField,
  btnGlass,
  btnGlassBroken,
  chip,
  chipOn,
  dot,
  dotOn,
  viewer,
  hint,
  tabBar,
  tab,
  tabActive,
  telemetryGrid,
  telemetryTile,
  telemetryNum,
  telemetryLbl,
  sparkline
}
