/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_apps) repo
# (see https://github.com/nepi-engine/nepi_apps)
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
import React, { Component } from "react"
import { observer, inject } from "mobx-react"
import Toggle from "react-toggle"

import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"

import { onChangeSwitchStateValue } from "./Utilities"

// Stereo process selector + per-process settings editor.
//
// Mirrors the process block of NepiAppPTAuto-Controls.js:
//   * a Select bound to the status message's available_processes /
//     selected_process (published to set_selected_process),
//   * a "Reload Processes" trigger (reload_processes),
//   * a "Show Process Settings" toggle that reveals one editable box per
//     tunable variable of the selected process. The node flattens the selected
//     process's settings into status_msg.process_control_names /
//     process_control_values (index-aligned); an edit sends an UpdateFloat to
//     set_process_control_value.
//
// The parent (NepiAppStereoCam.js) owns the status subscription and passes
// the current status_msg + app namespace down as props.
@inject("ros")
@observer
class NepiAppStereoCamControls extends Component {
  constructor(props) {
    super(props)

    this.state = {
      show_settings: false,
      // Local copy of the editable control values, so typing is not clobbered by
      // the next status tick. Synced from props whenever the node publishes a
      // genuinely different set of values (JSON compare, like PT Auto).
      control_names: [],
      control_values: [],
    }

    this.onSelectProcess = this.onSelectProcess.bind(this)
    this.onReloadProcesses = this.onReloadProcesses.bind(this)
    this.onUpdateControlValue = this.onUpdateControlValue.bind(this)
    this.onKeySaveControlValue = this.onKeySaveControlValue.bind(this)
    this.renderControlValue = this.renderControlValue.bind(this)
    this.renderControls = this.renderControls.bind(this)
  }

  // Sync the local editable values from the incoming status message, but only
  // when they actually changed (so an in-progress edit is not overwritten).
  syncControlsFromStatus() {
    const status_msg = this.props.status_msg
    if (status_msg == null) {
      return
    }
    const names = status_msg.process_control_names || []
    const values = status_msg.process_control_values || []
    const changed =
      JSON.stringify(names) !== JSON.stringify(this.state.control_names) ||
      JSON.stringify(values) !== JSON.stringify(this.state.control_values)
    if (changed) {
      this.setState({ control_names: names, control_values: values })
    }
  }

  componentDidMount() {
    this.syncControlsFromStatus()
  }

  componentDidUpdate(prevProps) {
    if (prevProps.status_msg !== this.props.status_msg) {
      this.syncControlsFromStatus()
    }
  }

  onSelectProcess(event) {
    const { sendStringMsg } = this.props.ros
    const namespace = this.props.appNamespace
    if (namespace != null) {
      sendStringMsg(namespace + "/set_selected_process", event.target.value)
    }
  }

  onReloadProcesses() {
    const { sendTriggerMsg } = this.props.ros
    const namespace = this.props.appNamespace
    if (namespace != null) {
      sendTriggerMsg(namespace + "/reload_processes")
    }
  }

  // Track typing locally and flag the box as modified (red) until saved.
  onUpdateControlValue(event, name, index) {
    const control_values = this.state.control_values.slice()
    control_values[index] = event.target.value
    this.setState({ control_values: control_values })
    const el = document.getElementById(name)
    if (el) {
      el.style.color = Styles.vars.colors.red
    }
  }

  // On Enter, push the edited value to the node as an UpdateFloat(name, value).
  onKeySaveControlValue(event, name, index) {
    if (event.key !== "Enter") {
      return
    }
    const namespace = this.props.appNamespace
    const parsed = parseFloat(event.target.value)
    if (namespace != null && !Number.isNaN(parsed)) {
      this.props.ros.sendUpdateFloatMsg(namespace + "/set_process_control_value", name, parsed)
    } else {
      // Bad input -- revert to the last published value.
      this.setState({ control_values: (this.props.status_msg.process_control_values || []) })
    }
    const el = document.getElementById(name)
    if (el) {
      el.style.color = Styles.vars.colors.black
    }
  }

  renderControlValue(name, value, index) {
    return (
      <Label key={name} title={name}>
        <Input
          id={name}
          style={{ width: "45%", float: "left" }}
          value={value}
          onChange={(event) => this.onUpdateControlValue(event, name, index)}
          onKeyDown={(event) => this.onKeySaveControlValue(event, name, index)}
        />
      </Label>
    )
  }

  renderControls() {
    const status_msg = this.props.status_msg
    if (status_msg == null) {
      return (<Columns><Column></Column></Columns>)
    }

    // available_processes / selected_process come straight from the app status,
    // so the RUI menu always matches the node.
    const available_processes = (status_msg.available_processes && status_msg.available_processes.length > 0)
      ? status_msg.available_processes : ["None"]
    const selected_process = status_msg.selected_process
    const process_ready = status_msg.process_ready

    const show_settings = this.state.show_settings
    const control_names = this.state.control_names
    const control_values = this.state.control_values

    return (
      <React.Fragment>

        {/* Process dropdown + reload -- always visible in the control section. */}
        <Label title={"Stereo Process"}>
          <Select
            id="set_selected_process"
            onChange={this.onSelectProcess}
            value={selected_process}
            disabled={process_ready === false}
          >
            {available_processes.map((opt) => (
              <Option key={opt} value={opt}>{opt}</Option>
            ))}
          </Select>
        </Label>

        <ButtonMenu>
          <Button onClick={this.onReloadProcesses}>{"Reload Processes"}</Button>
        </ButtonMenu>

        <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        {/* Toggle reveals one editable box per tunable setting of the process. */}
        <Label title={"Show Process Settings"}>
          <Toggle
            checked={show_settings}
            onClick={() => onChangeSwitchStateValue.bind(this)("show_settings", show_settings)}
          />
        </Label>

        <div hidden={show_settings === false}>
          {control_names.map((name, index) => (
            this.renderControlValue(name, control_values[index], index)
          ))}
        </div>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

      </React.Fragment>
    )
  }

  render() {
    return this.renderControls()
  }
}

export default NepiAppStereoCamControls
