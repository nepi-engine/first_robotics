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
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"

import NepiIFControls from "./Nepi_IF_Controls"

import { onChangeSwitchStateValue } from "./Utilities"

// Stereo process selector + the active process's controls.
//
//   * a Select bound to the status message's available_processes /
//     selected_process (published to set_selected_process),
//   * a "Reload Processes" trigger (reload_processes),
//   * a "Show Process Settings" toggle that reveals ONE Nepi_IF_Controls, bound to
//     the controls namespace the node reports as active.
//
// Pre-migration this block rendered one plain <Input> per entry of the node's
// flattened status_msg.process_control_names / process_control_values arrays and
// sent every edit back as an UpdateFloat to set_process_control_value -- a text box
// for a bool, a text box for a value with three legal choices, and a float on the
// wire regardless. All of that is gone. Each process now owns a ControlsIF whose
// nepi_controls control set carries type, bounds, options and display name, so
// Nepi_IF_Controls renders the right widget per setting and publishes to the typed
// set_<type>_control_value topic. Same arrangement as
// NepiAppObstacles.renderProcessControls().
//
// The node owns one controls namespace PER PROCESS and only the active one is
// mounted here; the inactive process's component is never mounted at all, which is
// what actually suppresses it. Per-control 'hidden' cannot do that job --
// nepi_controls.set_control_hidden() stringifies the bool, and Nepi_IF_Controls
// reads each control's own hidden flag rather than ControlsStatus.hidden.
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
    }

    this.onSelectProcess = this.onSelectProcess.bind(this)
    this.onReloadProcesses = this.onReloadProcesses.bind(this)
    this.isTopic = this.isTopic.bind(this)
    this.getActiveControlsNamespace = this.getActiveControlsNamespace.bind(this)
    this.renderProcessControls = this.renderProcessControls.bind(this)
    this.renderControls = this.renderControls.bind(this)
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

  // True when a string off the status message names a real namespace. The node
  // reports 'None' when there is no valid selection, and an un-set ROS string
  // field arrives as ''.
  isTopic(topic) {
    return (topic != null && topic !== '' && topic !== 'None' && topic !== 'None Available')
  }

  // Controls namespace of the ACTIVE stereo process, or null when there is none
  // to mount.
  //
  // Falls back to <app>/<selected_process> if the namespace field is unset but the
  // process name is known -- the node builds it exactly that way. Null when
  // neither is usable, and renderProcessControls() then mounts nothing rather than
  // a dead subscription.
  getActiveControlsNamespace() {
    const status_msg = this.props.status_msg
    if (status_msg == null) {
      return null
    }
    if (this.isTopic(status_msg.active_controls_namespace)) {
      return status_msg.active_controls_namespace
    }
    const appNamespace = this.props.appNamespace
    if (appNamespace != null && this.isTopic(status_msg.selected_process)) {
      return appNamespace + "/" + status_msg.selected_process
    }
    return null
  }

  // One Nepi_IF_Controls on the namespace the node reports as active.
  //
  // key={namespace} makes a process switch REMOUNT the component rather than
  // repoint it. Nepi_IF_Controls does resubscribe when its namespace prop changes,
  // but it carries per-control edit state (editValues / pending) that belongs to
  // the set it was showing; remounting drops that state with the component instead
  // of letting one process's in-progress edit reconcile against the other's status.
  //
  // allways_show_controls suppresses the component's own "Show Controls" toggle:
  // the "Show Process Settings" toggle above it is already this block's collapse
  // control, and two nested toggles for the same thing is noise.
  renderProcessControls() {
    const namespace = this.getActiveControlsNamespace()
    if (namespace == null) {
      return null
    }
    return (
      <NepiIFControls
        key={namespace}
        namespace={namespace}
        make_section={false}
        allways_show_controls={true}
      />
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

        {/* Toggle reveals the active process's controls. */}
        <Label title={"Show Process Settings"}>
          <Toggle
            checked={show_settings}
            onClick={() => onChangeSwitchStateValue.bind(this)("show_settings", show_settings)}
          />
        </Label>

        <div hidden={show_settings === false}>
          {this.renderProcessControls()}
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
