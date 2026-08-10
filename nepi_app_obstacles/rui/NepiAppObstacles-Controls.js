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

import Section from "./Section"
import Label from "./Label"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"

import NepiIFControls from "./Nepi_IF_Controls"

@inject("ros")
@observer

// This app's own controls: the obstacle process selector, the ACTIVE process's
// control set, and the example control set.
//
// Holds no subscription. NepiAppObstacles.js is the sole owner of every ROS
// subscription on the page and passes the app namespace and the current app status
// message down as props -- the same contract NepiAppStereoCam-Controls.js takes
// from NepiAppStereoCam.js. See the state-ownership comment at the top of
// NepiAppObstacles.js. This component does publish: the two process-selector
// handlers below go straight out on this app's own topics.
//
// Nothing here is derived from the selected depth map, so nothing here is subject
// to the "clear before re-deriving" guard that governs the other two children --
// which process runs is app state that outlives any depth map selection.
class NepiAppObstaclesControls extends Component {

  constructor(props) {
    super(props)

    this.isTopic = this.isTopic.bind(this)
    this.getActiveControlsNamespace = this.getActiveControlsNamespace.bind(this)
    this.getExampleControlsNamespace = this.getExampleControlsNamespace.bind(this)
    this.onSelectProcess = this.onSelectProcess.bind(this)
    this.onReloadProcesses = this.onReloadProcesses.bind(this)
    this.renderProcessSelector = this.renderProcessSelector.bind(this)
    this.renderProcessControls = this.renderProcessControls.bind(this)
    this.renderExampleControls = this.renderExampleControls.bind(this)
  }

  // True when a string off the status message names a real topic or namespace. The
  // node reports 'None' when there is no valid selection, and an un-set ROS string
  // field arrives as ''. Same predicate the parent applies -- a pure string test
  // with no state.
  isTopic(topic) {
    return (topic != null && topic !== '' && topic !== 'None' && topic !== 'None Available')
  }

  // Process selector handlers. Both publish to this app's own topics, the pair
  // nepi_app_stereo_cam uses: a String carrying the process NAME, and an Empty
  // trigger to re-import the module. The node is the authority on both -- neither
  // handler touches local state, so the dropdown only moves once the node has
  // accepted the change and republished its status.
  onSelectProcess(event) {
    const { sendStringMsg } = this.props.ros
    const appNamespace = this.props.appNamespace
    if (appNamespace != null) {
      sendStringMsg(appNamespace + "/set_selected_process", event.target.value)
    }
  }

  onReloadProcesses() {
    const { sendTriggerMsg } = this.props.ros
    const appNamespace = this.props.appNamespace
    if (appNamespace != null) {
      sendTriggerMsg(appNamespace + "/reload_processes")
    }
  }

  // Controls namespace of the ACTIVE obstacle process, or null when there is none
  // to mount.
  //
  // This is the whole of the one-set-at-a-time rule on the RUI side: the node owns
  // one ControlsIF per process in nepi_obstacles.PROCESSES_DICT, names the active
  // one here, and this component mounts a Nepi_IF_Controls on that namespace only.
  // The inactive process's component is never mounted, which is what actually hides
  // it -- the node's set_controls_hidden() call is intent, not suppression, because
  // Nepi_IF_Controls reads each control's own hidden flag and never
  // ControlsStatus.hidden.
  //
  // Falls back to <app>/<selected_process> if the namespace field is unset but the
  // process name is known, for the same reason as above; null when neither is
  // usable, and renderProcessControls() then mounts nothing rather than a dead
  // subscription.
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

  // Namespace of this app's example ControlsIF -- the control set that belongs to
  // no process. Distinct from getActiveControlsNamespace() above, which names the
  // ACTIVE process's set and changes as the operator switches processes; this one
  // is fixed for the life of the node. Taken from the app status, which publishes
  // it fully qualified, with the conventional <app>/example_controls path as the
  // fallback for the window before the first status message arrives. Mirrors
  // NepiAppControlsSandbox.js getControlsNamespace().
  getExampleControlsNamespace() {
    const status_msg = this.props.status_msg
    if (status_msg != null && status_msg.example_controls_namespace) {
      return status_msg.example_controls_namespace
    }
    const appNamespace = this.props.appNamespace
    return (appNamespace != null) ? appNamespace + "/example_controls" : null
  }

  // Obstacle process selector: a native dropdown plus a Reload Processes button,
  // the same block NepiAppStereoCam-Controls.js renders and the one exception to
  // this page owning no controls of its own.
  //
  // It is deliberately NOT a Nepi_IF_Controls Selection control. Which process runs
  // is app state that outlives either control set, a reload trigger is unreachable
  // through Nepi_IF_Controls (it sends UpdateString to a topic ControlsIF subscribes
  // as UpdateTrigger, and Store.js has no sendUpdateTriggerMsg), and a selector on a
  // plain topic keeps working when a ControlsIF does not.
  //
  // Both lists come off the app status message, so the menu cannot drift from the
  // processes nepi_obstacles actually registers. The dropdown is disabled while the
  // node reports process_ready false, i.e. mid-reload.
  //
  // Carries no bold title Label of its own: render() puts this block and
  // renderProcessControls() inside one "Obstacle Process" Section, and that
  // Section's title is the heading. Only the per-widget "Process" Label remains.
  renderProcessSelector() {
    const status_msg = this.props.status_msg
    const available_processes = (status_msg != null && status_msg.available_processes != null &&
                                 status_msg.available_processes.length > 0)
      ? status_msg.available_processes : ["None"]
    const selected_process = (status_msg != null && this.isTopic(status_msg.selected_process))
      ? status_msg.selected_process : "None"
    const process_ready = (status_msg != null) ? status_msg.process_ready : false

    return (
      <React.Fragment>

        <Label title={"Process"}>
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

      </React.Fragment>
    )
  }

  // Controls of the ACTIVE obstacle process, or nothing.
  //
  // One Nepi_IF_Controls on the namespace the node reports as active. The node
  // owns one controls namespace per process and only this one is ever mounted, so
  // the operator is shown exactly the active process's controls and never the
  // other's -- see getActiveControlsNamespace().
  //
  // key={namespace} makes a process switch REMOUNT the component rather than
  // repoint it, the same guard renderImageViewer() in NepiAppObstacles-Images.js
  // uses on image topics. Nepi_IF_Controls does resubscribe when its namespace prop
  // changes, but it carries per-control edit state (editValues / pending) that
  // belongs to the set it was showing; remounting drops that state with the
  // component instead of letting one process's in-progress edit reconcile against
  // the other's status.
  //
  // allways_show_controls suppresses the component's own "Show Controls" toggle: a
  // collapse toggle inside an already-labelled panel is noise, and the enclosing
  // "Obstacle Process" Section is the grouping affordance. make_section={false}
  // keeps Nepi_IF_Controls from drawing a bordered box of its own inside that one.
  //
  // The bold Label below stays even though the Section is titled: it carries the
  // LIVE process name, which the Section title does not.
  renderProcessControls(divider) {
    const namespace = this.getActiveControlsNamespace()
    if (namespace == null) {
      return null
    }
    const status_msg = this.props.status_msg
    const title = (status_msg != null && this.isTopic(status_msg.selected_process))
      ? ("Controls: " + status_msg.selected_process)
      : "Process Controls"

    return (
      <React.Fragment>

        {divider}

        <Label title={title} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFControls
          key={namespace}
          namespace={namespace}
          make_section={false}
          allways_show_controls={true}
        />

      </React.Fragment>
    )
  }

  // A copy of the controls sandbox app's Controls box, bound to this app's example
  // ControlsIF. Same component, mounted the same way the sandbox page mounts it --
  // make_section={false} inside a Section of its own -- so it looks and behaves
  // identically; the "Show Controls" toggle at the top of the box is
  // Nepi_IF_Controls' own, not something this page adds. Only the Section title
  // differs.
  //
  // This is the page's SECOND Nepi_IF_Controls, and the one that is always mounted.
  // renderProcessControls() mounts the other on the active process's namespace and
  // only while there is one; this box belongs to no process, so it never unmounts.
  //
  // The sandbox page also mounts NepiAppControlsSandbox-Settings below the box in
  // develop/admin mode. That component is deliberately NOT copied: it lives in the
  // sandbox app's rui/ directory, so importing it would make this app's RUI build
  // depend on nepi_app_controls_sandbox being installed. Every app's rui js files
  // install FLAT into the shared nepi_rui src directory, so such an import
  // resolves only when that other app happens to be installed on the same device.
  renderExampleControls() {
    return (
      <Section title={"Example Controls"}>

        <NepiIFControls
          namespace={this.getExampleControlsNamespace()}
          make_section={false}
        />

      </Section>
    )
  }

  // The process selector and the active process's controls sit OUTSIDE the
  // Connections box in NepiAppObstacles-Data.js -- they are this app's own
  // controls, not source connections -- in a bordered "Obstacle Process" Section
  // of their own, and the Example Controls box sits at the bottom of the column,
  // below everything else the page puts there.
  //
  // ONE Section around both, not two: which process is running and what that
  // process's controls are is a single operator concern. The divider still
  // separates them, now inside that box. Example Controls keeps its own separate
  // Section -- it belongs to no process.
  //
  // The Section renders whether or not renderProcessControls() returns null, so
  // with no active controls namespace the box still shows the selector and the
  // Reload Processes button. That is the point: the reload button is how the
  // operator recovers from exactly that state.
  render() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    return (
      <React.Fragment>

        <Section title={"Obstacle Process"}>

          {this.renderProcessSelector()}

          {this.renderProcessControls(divider)}

        </Section>

        {this.renderExampleControls()}

      </React.Fragment>
    )
  }
}

export default NepiAppObstaclesControls
