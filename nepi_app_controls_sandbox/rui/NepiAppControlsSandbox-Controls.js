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
 * APP-LOCAL COPY of the core RUI component Nepi_IF_Controls.js.
 *
 * This file is owned by nepi_app_controls_sandbox. It exists so this app's
 * control rendering can be changed and extended without editing core nepi_rui.
 * Edits here affect this app only; the shared Nepi_IF_Controls.js and its other
 * consumers are untouched.
 *
 * Two rules keep it working:
 *   1. This file must stay listed under RUI_DICT.rui_files in
 *      params/controls_sandbox_app_params.yaml. That listing is the only place
 *      a RUI file is registered; build_nepi_rui.sh generates the rest.
 *   2. Its basename must stay unique across all app packages. Every app's rui/
 *      directory installs flat into the same nepi_rui src folder, so a
 *      duplicate basename from any app silently overwrites this file.
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Toggle from "react-toggle"
import AsyncToggle from "./AsyncToggle"
import Select, { Option } from "./Select"
import Input from "./Input"
import { SliderAdjustment } from "./AdjustmentWidgets"
import RangeAdjustment from "./RangeAdjustment"
import Theme from "./NepiAppControlsSandbox-Theme"

import { setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue } from "./Utilities"

@inject("ros")
@observer

// Component that contains the ControlsIF controls. Renders one widget per
// control from a nepi_interfaces/ControlsStatus message.
class NepiAppControlsSandboxControls extends Component {
  constructor(props) {
    super(props)

    this.state = {
      controlsNamespace: null,
      status_msg: null,

      // name -> in-progress edit string for editable text/number inputs
      editValues: {},

      // name -> { baseline, typed, type } for values we have sent but not yet
      // seen confirmed in an incoming status. Keeps the optimistic override in
      // editValues alive until statusListener() reconciles it (see below).
      pending: {},

      // "Show Controls" toggle state (Nepi_IF_Settings pattern). Defaults shown;
      // can be overridden via the show_controls prop or forced on via
      // allways_show_controls.
      show_controls: (this.props.show_controls !== undefined) ? this.props.show_controls : true,

      statusListener: null,
      needs_update: false
    }

    this.getNamespace = this.getNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.getControlValue = this.getControlValue.bind(this)
    this.renderControl = this.renderControl.bind(this)
    this.onInputChange = this.onInputChange.bind(this)
    this.onInputKey = this.onInputKey.bind(this)
  }

  getNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null) {
      if (this.props.namespace !== undefined) {
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  // Read the current value a control reports in a status message, by name and
  // type. Returns null if the control isn't present or isn't an editable type.
  getControlValue(message, name, type) {
    if (message == null) { return null }
    const names = message.controls_name_list || []
    const i = names.indexOf(name)
    if (i === -1) { return null }
    const msgs = message.controls_msg_list || []
    const m = msgs[i]
    if (m == null) { return null }
    if (type === "String") { return m.set_string }
    if (type === "Int") { return m.set_int }
    if (type === "Float") { return m.set_float }
    return null
  }

  statusListener(message) {
    // Reconcile any in-progress edits against the freshly received status.
    // While a value is being edited we keep showing the user's typed text (an
    // optimistic override in editValues) until this status confirms the change.
    // We drop the override when either the backend value has moved off what it
    // held when we sent (covers the node clamping/rejecting to a *different*
    // value, e.g. Int bounds [0,10]) or it now equals what the user typed.
    // Dropping the override in the same message that carries the new value lets
    // the input hand off from typed-text to backend-value with no stale frame.
    const pendingKeys = Object.keys(this.state.pending)
    if (pendingKeys.length === 0) {
      this.setState({ status_msg: message })
      return
    }
    const editValues = { ...this.state.editValues }
    const pending = { ...this.state.pending }
    let changed = false
    pendingKeys.forEach((name) => {
      const p = pending[name]
      const cur = this.getControlValue(message, name, p.type)
      if (cur == null) { return }
      var moved = false
      var matches = false
      if (p.type === "Int") {
        moved = cur !== p.baseline
        matches = cur === parseInt(p.typed, 10)
      } else if (p.type === "Float") {
        moved = cur !== p.baseline
        matches = cur === parseFloat(p.typed)
      } else { // String
        moved = String(cur) !== String(p.baseline)
        matches = String(cur) === String(p.typed)
      }
      if (moved || matches) {
        delete editValues[name]
        delete pending[name]
        changed = true
      }
    })
    if (changed) {
      this.setState({ status_msg: message, editValues: editValues, pending: pending })
    } else {
      this.setState({ status_msg: message })
    }
  }

  updateStatusListener(namespace) {
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1) {
      const statusNamespace = namespace + '/status'
      var statusListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_interfaces/ControlsStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ controlsNamespace: namespace, needs_update: false, editValues: {}, pending: {} })
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getNamespace()
    if ((namespace != null && namespace !== this.state.controlsNamespace) || this.state.needs_update === true) {
      this.updateStatusListener(namespace)
    }
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null })
    }
  }

  // Editable text/number input helpers (PTX controls pattern)
  onInputChange(name, e) {
    const el = document.getElementById('csbx_' + name)
    if (el) { setElementStyleModified(el) }
    const editValues = { ...this.state.editValues }
    editValues[name] = e.target.value
    this.setState({ editValues: editValues })
  }

  onInputKey(name, type, e) {
    if (e.key !== 'Enter') { return }
    const namespace = this.getNamespace()
    const { sendUpdateStringMsg, sendUpdateIntMsg, sendUpdateFloatMsg } = this.props.ros
    const el = document.getElementById('csbx_' + name)
    if (el) { clearElementStyleModified(el) }
    const raw = e.target.value
    // Value the control reports right now; statusListener() uses this baseline
    // to detect when the backend has acted on our change.
    const baseline = this.getControlValue(this.state.status_msg, name, type)
    var sent = false
    if (type === "String") {
      sendUpdateStringMsg(namespace + "/set_string_control_value", name, raw)
      sent = true
    } else if (type === "Int") {
      const val = parseInt(raw, 10)
      if (!Number.isNaN(val)) { sendUpdateIntMsg(namespace + "/set_int_control_value", name, val); sent = true }
    } else if (type === "Float") {
      const val = parseFloat(raw)
      if (!Number.isNaN(val)) { sendUpdateFloatMsg(namespace + "/set_float_control_value", name, val); sent = true }
    }
    const editValues = { ...this.state.editValues }
    const pending = { ...this.state.pending }
    if (sent) {
      // Keep showing the typed text (optimistic) until a status message
      // confirms the change; statusListener() clears these once reconciled.
      editValues[name] = raw
      pending[name] = { baseline: baseline, typed: raw, type: type }
    } else {
      // Invalid input: fall back to the last reported value (original behavior).
      delete editValues[name]
      delete pending[name]
    }
    this.setState({ editValues: editValues, pending: pending })
  }

  // A control's on-screen row: caption + optional wire-name/bounds subtext +
  // optional broken/dropped flag on the left (concept_4_glass_console.html's
  // .row / .name / .sub / .flag), the widget on the right in a fixed-width
  // .ctl column. Every renderControl() branch below builds its own widget and
  // hands it to this wrapper instead of <Label>, since Label's wrapper divs
  // carry no class or style hook this app can reach (see NepiAppControlsSandbox
  // -Theme.js's header comment).
  renderRow(name, display_name, sub, flag, widget) {
    return (
      <div style={Theme.row} key={name}>
        <div>
          <div style={Theme.rowName}>{display_name}</div>
          {(sub !== undefined && sub !== null && sub !== '') ? <div style={Theme.rowSub}>{sub}</div> : null}
          {(flag !== undefined && flag !== null && flag !== '') ? <div style={Theme.rowFlag}>{flag}</div> : null}
        </div>
        <div style={{ width: "48%" }}>{widget}</div>
      </div>
    )
  }

  // Render a single control given its type and Control message.
  // Each block below maps one nepi_controls control type to its RUI widget and
  // the nepi_controls "set_*_control_value" topic it publishes to on change.
  renderControl(name, type, control_msg, index) {
    const namespace = this.getNamespace()
    const { sendUpdateIntMsg, sendUpdateStringMsg, sendUpdateBoolMsg } = this.props.ros
    const display_name = (control_msg.display_name && control_msg.display_name !== '') ? control_msg.display_name : name

    // Value inputs whose value tracks either the in-progress edit or the message
    const editing = (name in this.state.editValues)

    // MENU -- drop-down of string options; the control's value is the *index*
    // of the selected option. Sends the new index as an Int.
    if (type === "Menu") {
      const options = control_msg.string_options
      const set_index = control_msg.set_index
      const widget = (
        <Select
          id={'csbx_' + name}
          className="csbx-select"
          style={Theme.indField}
          value={set_index}
          onChange={(e) => sendUpdateIntMsg(namespace + "/set_menu_control_value", name, parseInt(e.target.value, 10))}
        >
          {options.map((opt, i) => <Option key={name + '_' + i} value={i}>{opt}</Option>)}
        </Select>
      )
      return this.renderRow(name, display_name, name, null, widget)
    }

    // SELECTION -- drop-down of string options; the control's value is the
    // selected option *text* (not its index). Sends the new text as a String.
    if (type === "Selection") {
      const options = control_msg.string_options
      const set_string = control_msg.set_string
      const widget = (
        <Select
          id={'csbx_' + name}
          className="csbx-select"
          style={Theme.indField}
          value={set_string}
          onChange={(e) => sendUpdateStringMsg(namespace + "/set_selection_control_value", name, e.target.value)}
        >
          {options.map((opt, i) => <Option key={name + '_' + i} value={opt}>{opt}</Option>)}
        </Select>
      )
      return this.renderRow(name, display_name, name, null, widget)
    }

    // SELECTIONS -- a multi-select: each option gets its own toggle, drawn as
    // a Glass Console chip. The value is the full array of currently-selected
    // option strings. On every click we send the complete desired selection
    // (declarative), not a single delta.
    if (type === "Selections") {
      const options = control_msg.string_options
      const set_strings = control_msg.set_strings || []
      const { sendUpdateStringArrayMsg } = this.props.ros
      const widget = (
        <div style={{ display: "flex", gap: "6px", justifyContent: "flex-end", flexWrap: "wrap" }}>
          {options.map((opt, i) => {
            const on = set_strings.indexOf(opt) !== -1
            return (
              <div
                key={name + '_' + i}
                style={on ? { ...Theme.chip, ...Theme.chipOn } : Theme.chip}
                onClick={() => {
                  // Send the complete desired selection (declarative), not a toggle.
                  const next = on
                    ? set_strings.filter((s) => s !== opt)
                    : [...set_strings, opt]
                  sendUpdateStringArrayMsg(namespace + "/set_selections_control_value", name, next)
                }}
              >
                {opt}
              </div>
            )
          })}
        </div>
      )
      return this.renderRow(name, display_name, name, null, widget)
    }

    // TRIGGER -- a momentary action. There is no persistent value; pressing the
    // button fires a one-shot trigger (an empty String payload). Broken today
    // (see README's Known rough edges): the Controls box sends an UpdateString
    // and ControlsIF subscribes the topic as UpdateTrigger, so the type
    // mismatch means the node never receives a click. Flagged rather than
    // hidden, matching every concept_4 mockup convention.
    if (type === "Trigger") {
      const widget = (
        <button
          style={{ ...Theme.btnGlass, ...Theme.btnGlassBroken }}
          onClick={() => sendUpdateStringMsg(namespace + "/set_trigger_control_value", name, "")}
        >
          {"Trigger"}
        </button>
      )
      return this.renderRow(name, display_name, name, "! no fire — msg-type mismatch", widget)
    }

    // BOOL -- a single on/off switch. Sends the *opposite* of the current
    // value as a Bool each time it is clicked.
    if (type === "Bool") {
      const checked = (control_msg.set_bool === true)
      const widget = (
        <div style={{ textAlign: "right" }}>
          <AsyncToggle
            checked={checked}
            onClick={() => sendUpdateBoolMsg(namespace + "/set_bool_control_value", name, !checked)}
          />
        </div>
      )
      return this.renderRow(name, display_name, name, null, widget)
    }

    // STRING / INT / FLOAT -- free-form typed values. These follow the PTX
    // editable-input pattern: the box shows an in-progress edit string while
    // the user types, and the value is sent (parsed to the right type) only on
    // Enter. See onInputChange / onInputKey above.
    if (type === "String" || type === "Int" || type === "Float") {
      var msgValue = ''
      if (type === "String") { msgValue = control_msg.set_string }
      else if (type === "Int") { msgValue = control_msg.set_int }
      else { msgValue = control_msg.set_float }
      const value = editing ? this.state.editValues[name] : msgValue
      const bounds = control_msg.int_bounds || control_msg.float_bounds || []
      const has_bounds = (type !== "String" && bounds.length > 1)
      const sub = has_bounds ? (name + " [" + bounds[0] + "," + bounds[1] + "]") : name
      const widget = (
        <Input
          id={'csbx_' + name}
          className="csbx-input"
          style={Theme.indField}
          value={value}
          onChange={(e) => this.onInputChange(name, e)}
          onKeyDown={(e) => this.onInputKey(name, type, e)}
        />
      )
      return this.renderRow(name, display_name, sub, null, widget)
    }

    // FLOATSLIDER -- a single decimal value dragged between a min and max.
    // float_bounds carries [min, max]; -999 in either slot means "no limit",
    // in which case we fall back to a sensible default (0 / 100).
    if (type === "FloatSlider") {
      const bounds = control_msg.float_bounds || []
      const min = (bounds.length > 0 && bounds[1] !== -999) ? bounds[0] : 0
      const max = (bounds.length > 1 && bounds[0] !== -999) ? bounds[1] : 100
      // Step and display precision follow the control's own round_value
      // (decimals its stored value is rounded to) instead of the previous
      // hardcoded step of 1, which made any sub-unit range unusable.
      const round_value = control_msg.round_value
      const has_round = (typeof round_value === 'number' && round_value >= 0)
      const step = has_round ? 1 / Math.pow(10, round_value) : 1
      const displayDecimals = has_round ? round_value : undefined
      return (
        <div style={Theme.row} key={name}>
          <div>
            <div style={Theme.rowName}>{display_name}</div>
            <div style={Theme.rowSub}>{name}</div>
          </div>
          <div style={{ width: "48%" }}>
            <SliderAdjustment
              title={null}
              noLabel
              comp_name={name}
              topic={namespace + "/set_floatslider_control_value"}
              msgType={"std_msgs/Float32"}
              adjustment={control_msg.set_float}
              min={min}
              max={max}
              step={step}
              displayDecimals={displayDecimals}
              scaled={1}
              tooltip={control_msg.description}
              unit={""}
            />
          </div>
        </div>
      )
    }

    // FLOATSLIDERS -- a min/max *range* dragged between two limits. set_floats
    // holds the current [min, max] handles; float_bounds holds the outer
    // [min_limit, max_limit] the handles may move within. Dropped by the SDK
    // today (see README's Known rough edges) -- flagged rather than hidden.
    if (type === "FloatSliders") {
      const set_floats = control_msg.set_floats || [0, 1]
      const bounds = control_msg.float_bounds || []
      const min_limit = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max_limit = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 1
      return (
        <div style={Theme.row} key={name}>
          <div>
            <div style={Theme.rowName}>{display_name}</div>
            <div style={Theme.rowSub}>{name}</div>
            <div style={Theme.rowFlag}>! dropped by SDK</div>
          </div>
          <div style={{ width: "48%" }}>
            <RangeAdjustment
              title={null}
              noLabel
              comp_name={name}
              topic={namespace + "/set_floatsliders_control_value"}
              min={set_floats[0]}
              max={set_floats[1]}
              min_limit_m={min_limit}
              max_limit_m={max_limit}
              tooltip={control_msg.description}
              unit={""}
            />
          </div>
        </div>
      )
    }

    return null
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const status_msg = this.state.status_msg

    // Show Controls toggle (Nepi_IF_Settings pattern). allways_show_controls
    // forces the controls open and hides the toggle.
    const allways_show_controls = (this.props.allways_show_controls !== undefined) ? this.props.allways_show_controls : false
    const show_controls = (allways_show_controls === true) ? true : this.state.show_controls

    // show_visibility_toggle lets a parent that already mounts several
    // instances of this box (e.g. one per tab) suppress the per-instance
    // "Show Controls" toggle so it does not appear multiple times on one page.
    const show_visibility_toggle = (this.props.show_visibility_toggle !== undefined) ? this.props.show_visibility_toggle : true

    const show_controls_toggle = (allways_show_controls === false && show_visibility_toggle === true) ? (
      <div style={{ ...Theme.row, borderBottom: "none" }}>
        <div style={Theme.rowName}>Show Controls</div>
        {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
        <Toggle
          checked={show_controls === true}
          onClick={() => onChangeSwitchStateValue.bind(this)("show_controls", show_controls)}>
        </Toggle>
      </div>
    ) : null

    // Controls widgets, one per non-hidden control. Only built when the section
    // is expanded and a status has arrived.
    // type_filter optionally restricts this box to only the listed control
    // types (e.g. one tab showing only Menu/Selection/Selections). Undefined
    // means show every type, which is today's behavior.
    const type_filter = this.props.type_filter

    var controls_body = null
    if (show_controls === true && status_msg != null) {
      const names = status_msg.controls_name_list || []
      const types = status_msg.controls_type_list || []
      const msgs = status_msg.controls_msg_list || []
      controls_body = (
        <div>
          {names.map((name, i) => {
            const control_msg = msgs[i]
            if (control_msg == null) { return null }
            // Hidden controls are not shown in the Controls box (they remain
            // manageable from the Controls Settings box).
            if (control_msg.hidden === true) { return null }
            if (type_filter !== undefined && type_filter.indexOf(types[i]) === -1) { return null }
            return this.renderControl(name, types[i], control_msg, i)
          })}
        </div>
      )
    }

    const body = (
      <React.Fragment>
        {show_controls_toggle}
        {controls_body}
      </React.Fragment>
    )

    if (make_section === false) {
      return body
    }
    return (
      <div style={Theme.glassPanel} className="csbx-glass-panel">
        <div style={Theme.panelCaption}>{(this.props.title !== undefined) ? this.props.title : "CONTROLS"}</div>
        {body}
      </div>
    )
  }
}

export default NepiAppControlsSandboxControls
