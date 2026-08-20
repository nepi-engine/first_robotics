/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Toggle from "react-toggle"
import Select from "./Select"
import Input from "./Input"
import Theme from "./NepiAppControlsSandbox-Theme"

import { createMenuListFromStrList, setElementStyleModified, clearElementStyleModified } from "./Utilities"


@inject("ros")
@observer

// Controls Settings box: per-control display management (order, display name,
// description, hidden, reset, factory-reset). A Select dropdown picks one
// control at a time (Nepi_IF_Settings pattern); the chosen control's edit
// fields are rendered below. Rendered only in develop run mode or when admin
// mode is set (gated by the parent and re-checked here).
class NepiAppControlsSandboxSettings extends Component {
  constructor(props) {
    super(props)

    this.state = {
      controlsNamespace: null,
      status_msg: null,

      selectedControlName: "",

      // (name + '_dn' | name + '_desc') -> in-progress edit string
      editValues: {},

      statusListener: null,
      needs_update: false
    }

    this.getNamespace = this.getNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.getControlMsg = this.getControlMsg.bind(this)
    this.updateSelectedControl = this.updateSelectedControl.bind(this)
    this.onFieldChange = this.onFieldChange.bind(this)
    this.onFieldKey = this.onFieldKey.bind(this)
    this.renderSelectedControl = this.renderSelectedControl.bind(this)
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

  statusListener(message) {
    this.setState({ status_msg: message })
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
    this.setState({ controlsNamespace: namespace, needs_update: false, editValues: {} })
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

  // Return the Control message for a control by name, or null.
  getControlMsg(name) {
    const status_msg = this.state.status_msg
    if (status_msg == null) { return null }
    const names = status_msg.controls_name_list || []
    const i = names.indexOf(name)
    if (i === -1) { return null }
    const msgs = status_msg.controls_msg_list || []
    return (msgs[i] != null) ? msgs[i] : null
  }

  // Dropdown onChange: pick the control whose name matches the selected option.
  updateSelectedControl(event) {
    const ind = event.nativeEvent.target.selectedIndex
    const name = event.nativeEvent.target[ind].text
    this.setState({ selectedControlName: name, editValues: {} })
  }

  onFieldChange(editKey, e) {
    const el = document.getElementById('csbxset_' + editKey)
    if (el) { setElementStyleModified(el) }
    const editValues = { ...this.state.editValues }
    editValues[editKey] = e.target.value
    this.setState({ editValues: editValues })
  }

  onFieldKey(topicSuffix, name, editKey, e) {
    if (e.key !== 'Enter') { return }
    const namespace = this.getNamespace()
    const { sendUpdateStringMsg } = this.props.ros
    const el = document.getElementById('csbxset_' + editKey)
    if (el) { clearElementStyleModified(el) }
    sendUpdateStringMsg(namespace + topicSuffix, name, e.target.value)
    const editValues = { ...this.state.editValues }
    delete editValues[editKey]
    this.setState({ editValues: editValues })
  }

  renderSelectedControl() {
    const name = this.state.selectedControlName
    const control_msg = this.getControlMsg(name)
    if (name === "" || name === "Select" || control_msg == null) {
      return null
    }

    const namespace = this.getNamespace()
    const { sendUpdateBoolMsg, sendUpdateStringMsg, sendUpdateOrderMsg } = this.props.ros

    const display_name = (control_msg.display_name && control_msg.display_name !== '') ? control_msg.display_name : name
    const description = control_msg.description || ''
    const hidden = (control_msg.hidden === true)

    const dnKey = name + '_dn'
    const descKey = name + '_desc'
    const dnValue = (dnKey in this.state.editValues) ? this.state.editValues[dnKey] : display_name
    const descValue = (descKey in this.state.editValues) ? this.state.editValues[descKey] : description

    return (
      <div>

        <div style={{ borderTop: `1px solid ${Theme.colors.glassBrd}`, marginTop: "24px", marginBottom: "4px" }} />

        <div style={Theme.row}>
          <div style={Theme.rowName}>Control</div>
          <div style={{ fontSize: "12.5px", color: Theme.colors.textDim }}>{name + "  (" + control_msg.type + ")"}</div>
        </div>

        <div style={Theme.row}>
          <div style={Theme.rowName}>Display Name</div>
          <div style={{ width: "48%" }}>
            <Input
              id={'csbxset_' + dnKey}
              className="csbx-input"
              style={Theme.indField}
              value={dnValue}
              onChange={(e) => this.onFieldChange(dnKey, e)}
              onKeyDown={(e) => this.onFieldKey("/set_control_display_name", name, dnKey, e)}
            />
          </div>
        </div>

        <div style={Theme.row}>
          <div style={Theme.rowName}>Description</div>
          <div style={{ width: "48%" }}>
            <Input
              id={'csbxset_' + descKey}
              className="csbx-input"
              style={Theme.indField}
              value={descValue}
              onChange={(e) => this.onFieldChange(descKey, e)}
              onKeyDown={(e) => this.onFieldKey("/set_control_description", name, descKey, e)}
            />
          </div>
        </div>

        <div style={Theme.row}>
          <div style={Theme.rowName}>Hidden</div>
          <Toggle
            checked={hidden}
            onClick={() => sendUpdateBoolMsg(namespace + "/set_control_hidden", name, !hidden)}
          />
        </div>

        <div style={Theme.row}>
          <div style={Theme.rowName}>Display Order</div>
          <div style={{ display: "flex", gap: "10px" }}>
            <button style={Theme.btnGlass} onClick={() => sendUpdateOrderMsg(namespace + "/set_control_move", name, "top")}>{"Top"}</button>
            <button style={Theme.btnGlass} onClick={() => sendUpdateOrderMsg(namespace + "/set_control_move", name, "up")}>{"Up"}</button>
            <button style={Theme.btnGlass} onClick={() => sendUpdateOrderMsg(namespace + "/set_control_move", name, "down")}>{"Down"}</button>
            <button style={Theme.btnGlass} onClick={() => sendUpdateOrderMsg(namespace + "/set_control_move", name, "bottom")}>{"Bottom"}</button>
          </div>
        </div>

        <div style={Theme.row}>
          <div style={Theme.rowName}>Reset</div>
          <div style={{ display: "flex", gap: "10px" }}>
            <button style={Theme.btnGlass} onClick={() => sendUpdateStringMsg(namespace + "/set_control_reset", name, "")}>{"Reset"}</button>
            <button style={{ ...Theme.btnGlass, ...Theme.btnGlassBroken }} onClick={() => sendUpdateStringMsg(namespace + "/set_control_factory_reset", name, "")}>{"Factory Reset"}</button>
          </div>
        </div>

      </div>
    )
  }

  render() {
    // Re-check the mode gate here as well (defense in depth); the parent panel
    // also gates on this same condition before mounting this component.
    const { systemRunMode, systemAdminModeSet } = this.props.ros
    const show_settings = (systemRunMode === "develop" || systemAdminModeSet === true)
    if (show_settings === false) {
      return null
    }

    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const status_msg = this.state.status_msg

    if (status_msg == null) {
      return null
    }

    const names = status_msg.controls_name_list || []

    const body = (
      <React.Fragment>

        <div style={Theme.row}>
          <div style={Theme.rowName}>Select Control</div>
          <div style={{ width: "48%" }}>
            <Select
              id="selectedControlName"
              className="csbx-select"
              style={Theme.indField}
              onChange={this.updateSelectedControl}
              value={this.state.selectedControlName}
            >
              {createMenuListFromStrList(names, false, [], ['Select'], [])}
            </Select>
          </div>
        </div>

        {this.renderSelectedControl()}

      </React.Fragment>
    )

    if (make_section === false) {
      return body
    }
    return (
      <div style={Theme.glassPanel} className="csbx-glass-panel">
        <div style={Theme.panelCaption}>{(this.props.title !== undefined) ? this.props.title : "CONTROLS SETTINGS"}</div>
        {body}
      </div>
    )
  }
}

export default NepiAppControlsSandboxSettings
