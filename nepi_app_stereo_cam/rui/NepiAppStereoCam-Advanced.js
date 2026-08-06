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
import Styles from "./Styles"

import NepiIFControls from "./Nepi_IF_Controls"

import { onChangeSwitchStateValue } from "./Utilities"

// Advanced Controls panel.
//
// A "Show Advanced Controls" toggle over a collapsed block, which is what a TAB is
// in this RUI: there is no tab-bar component anywhere in nepi_rui, and every app
// page (including this one's Stereo Calibration block, and Nepi_IF_Controls itself)
// separates a secondary group of settings exactly this way. Same effect -- the
// group is out of the operator's way until they ask for it -- without inventing a
// widget the rest of the RUI does not have.
//
// What is inside, and why it is split across two mechanisms:
//
//   * Max Framerate -- the depth rate cap. This is the app node's own param and
//     Float32 topic (set_max_framerate) and is reported in the app status message
//     this panel already receives, so it is rendered here as a plain input. The
//     node REJECTS an out-of-range value rather than clamping it, and the box
//     re-syncs from status after every send, so a refused edit visibly snaps back
//     to what the node kept.
//   * Everything else -- one Nepi_IF_Controls bound to the node's
//     advanced_controls ControlsIF, which owns the type/bounds validation, the
//     persistence and the widget choice per control. Adding an advanced setting is
//     a stereo_cam_app_node.py createAdvancedControlsInitDict() entry and no JS.
//
// Read-only rows above the box (effective rate, last L/R pair gap) are there to be
// set AGAINST: Frame Sync Tolerance is meaningless without knowing what the gap
// actually measures, and a Max Framerate above the loop rate silently does nothing.
//
// The parent (NepiAppStereoCam.js) owns the status subscription and passes the
// current status_msg + app namespace down as props.
@inject("ros")
@observer
class NepiAppStereoCamAdvanced extends Component {
  constructor(props) {
    super(props)

    this.state = {
      show_advanced: false,
      // Local copy so typing is not clobbered by the next status tick. null means
      // "not being edited", i.e. show the node's value.
      max_framerate: null,
    }

    this.getAdvancedControlsNamespace = this.getAdvancedControlsNamespace.bind(this)
    this.formatLimit = this.formatLimit.bind(this)
    this.onUpdateMaxFramerate = this.onUpdateMaxFramerate.bind(this)
    this.onKeySaveMaxFramerate = this.onKeySaveMaxFramerate.bind(this)
    this.renderFramerate = this.renderFramerate.bind(this)
    this.renderAdvanced = this.renderAdvanced.bind(this)
  }

  // Same adopt-but-never-overwrite-an-edit rule as the calibration panel.
  syncFromStatus() {
    const status_msg = this.props.status_msg
    if (status_msg == null) {
      return
    }
    if (this.state.max_framerate === null) {
      this.setState({ max_framerate: status_msg.max_framerate })
    }
  }

  componentDidMount() {
    this.syncFromStatus()
  }

  componentDidUpdate(prevProps) {
    if (prevProps.status_msg !== this.props.status_msg) {
      this.syncFromStatus()
    }
  }

  // Namespace of the node's advanced ControlsIF. Taken from the app status, which
  // publishes it fully qualified, with the conventional <app>/advanced_controls path
  // as the fallback -- which also covers the window before the first status message
  // arrives, and a device still running a status message built before this field
  // was added. Mirrors NepiAppStereoCam.js getExampleControlsNamespace().
  getAdvancedControlsNamespace() {
    const status_msg = this.props.status_msg
    if (status_msg != null && status_msg.advanced_controls_namespace &&
        status_msg.advanced_controls_namespace !== 'None') {
      return status_msg.advanced_controls_namespace
    }
    const appNamespace = this.props.appNamespace
    return (appNamespace != null) ? appNamespace + "/advanced_controls" : null
  }

  // One bound, printed for the range label.
  //
  // The limits arrive as float32 status fields, so a value with no exact float32
  // form comes across the bridge as its full double expansion -- 0.1 reads as
  // 0.10000000149011612 and would go into the label character for character. One
  // decimal is more precision than a rate limit needs, and the trailing '.0' is
  // dropped so a whole number reads as '1' rather than '1.0'.
  formatLimit(value) {
    return parseFloat(Number(value).toFixed(1)).toString()
  }

  // Track typing locally and flag the box as modified (red) until saved.
  onUpdateMaxFramerate(event) {
    this.setState({ max_framerate: event.target.value })
    const el = document.getElementById("max_framerate")
    if (el) {
      el.style.color = Styles.vars.colors.red
    }
  }

  // On Enter, publish the value, then drop the local copy so the next status tick
  // shows what the node ACTUALLY took -- a value outside the node's accept range is
  // refused, and the box should say so rather than keep displaying the request.
  onKeySaveMaxFramerate(event) {
    if (event.key !== "Enter") {
      return
    }
    const { sendFloatMsg } = this.props.ros
    const namespace = this.props.appNamespace
    if (namespace != null) {
      sendFloatMsg(namespace + "/set_max_framerate", event.target.value)
    }
    this.setState({ max_framerate: null })
    const el = document.getElementById("max_framerate")
    if (el) {
      el.style.color = Styles.vars.colors.black
    }
  }

  renderFramerate() {
    const status_msg = this.props.status_msg

    // Fall back to the node's own limits when the status message predates these
    // fields, so the label still states a range rather than "undefined".
    const fr_min = (status_msg != null && status_msg.max_framerate_min > 0)
      ? status_msg.max_framerate_min : 1.0
    const fr_max = (status_msg != null && status_msg.max_framerate_max > 0)
      ? status_msg.max_framerate_max : 60.0
    const effective = (status_msg != null) ? status_msg.effective_framerate : undefined
    const loop_rate = (status_msg != null) ? status_msg.depth_loop_rate_hz : undefined
    const pair_dt = (status_msg != null) ? status_msg.last_pair_dt_ms : undefined

    // The cap is not the output rate when the loop ticks slower than it. Flag that
    // rather than let the operator read a cap they are not getting.
    const capped_by_loop = (effective !== undefined && loop_rate !== undefined &&
                            loop_rate < status_msg.max_framerate)

    return (
      <React.Fragment>

        <Label title={"Max Framerate (Hz) [" + this.formatLimit(fr_min) + "-" + this.formatLimit(fr_max) + "]"}>
          <Input
            id={"max_framerate"}
            style={{ width: "45%", float: "left" }}
            value={this.state.max_framerate !== null ? this.state.max_framerate : ""}
            onChange={this.onUpdateMaxFramerate}
            onKeyDown={this.onKeySaveMaxFramerate}
          />
        </Label>

        <Label title={"Effective Rate (Hz)"}>
          <div style={{ color: capped_by_loop ? Styles.vars.colors.red : Styles.vars.colors.black }}>
            {(effective !== undefined) ? effective.toFixed(1) : "---"}
          </div>
        </Label>

        {capped_by_loop ? (
          <div style={{ fontStyle: "italic" }}>
            {"Depth Loop Rate (" + loop_rate.toFixed(1) + " Hz) is below Max Framerate, so it is what limits the output rate."}
          </div>
        ) : null}

        {/* What Frame Sync Tolerance below has to be set against: the gap the last
            pair was actually measured at. A tolerance under this number means no
            pair qualifies and no depth comes out. */}
        <Label title={"Last L/R Pair Gap (ms)"}>
          <div>{(pair_dt !== undefined) ? pair_dt.toFixed(1) : "---"}</div>
        </Label>

      </React.Fragment>
    )
  }

  renderAdvanced() {
    const show_advanced = this.state.show_advanced
    const namespace = this.getAdvancedControlsNamespace()

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Show Advanced Controls"}>
          <Toggle
            checked={show_advanced}
            onClick={() => onChangeSwitchStateValue.bind(this)("show_advanced", show_advanced)}
          />
        </Label>

        <div hidden={show_advanced === false}>

          {this.renderFramerate()}

          <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.xs, marginBottom: Styles.vars.spacing.xs }} />

          {/* allways_show_controls suppresses the component's own "Show Controls"
              toggle: the toggle above is already this block's collapse control, and
              two nested toggles for the same thing is noise. Same call
              NepiAppStereoCam-Controls makes for the process controls. */}
          {(namespace != null) ? (
            <NepiIFControls
              key={namespace}
              namespace={namespace}
              make_section={false}
              allways_show_controls={true}
            />
          ) : null}

        </div>

      </React.Fragment>
    )
  }

  render() {
    // Unlike the calibration panel, this one renders before the first status
    // message: the controls box binds to a namespace that can be derived from the
    // app namespace alone, and the framerate rows show "---" until status arrives.
    if (this.props.appNamespace == null) {
      return (<Columns><Column></Column></Columns>)
    }
    return this.renderAdvanced()
  }
}

export default NepiAppStereoCamAdvanced
