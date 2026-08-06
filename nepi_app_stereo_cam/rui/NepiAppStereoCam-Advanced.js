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
// Read-only rows above the box are there to be set AGAINST, or to explain an output
// the viewers cannot: Frame Sync Tolerance is meaningless without knowing what the
// L/R gap actually measures, a Max Framerate that does not divide into the depth
// loop tick quietly runs slower than it says, and a depth image with 0% valid
// pixels looks exactly like a depth image of a blank scene (see renderDepthOutput).
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
    this.renderDepthOutput = this.renderDepthOutput.bind(this)
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
      ? status_msg.max_framerate_max : 30.0
    const effective = (status_msg != null) ? status_msg.effective_framerate : undefined
    const loop_rate = (status_msg != null) ? status_msg.depth_loop_rate_hz : undefined
    const pair_dt = (status_msg != null) ? status_msg.last_pair_dt_ms : undefined

    // The cap is not the output rate when it falls between two wake-ups of the
    // depth loop: the throttle is only tested at a tick, so the rate drops to the
    // nearest tick division below the cap (20 Hz on the 30 Hz loop runs at 15).
    // Flagged rather than left for the operator to notice the two numbers
    // disagree. The loop tick is fixed and the cap cannot exceed it, so this is the
    // only way the two can differ. The 0.05 Hz margin keeps float32 round-trip
    // noise from flagging a rate that does match.
    const rate_shortfall = (effective !== undefined && loop_rate !== undefined &&
                            effective < (status_msg.max_framerate - 0.05))

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
          <div style={{ color: rate_shortfall ? Styles.vars.colors.red : Styles.vars.colors.black }}>
            {(effective !== undefined) ? effective.toFixed(1) : "---"}
          </div>
        </Label>

        {rate_shortfall ? (
          <div style={{ fontStyle: "italic" }}>
            {"Max Framerate does not divide evenly into the " + loop_rate.toFixed(0) +
             " Hz depth loop, so the rate drops to the nearest division below it. " +
             "Whole divisions of " + loop_rate.toFixed(0) + " hit exactly."}
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

  // What the depth viewer alone cannot tell you: whether the loop is running at all,
  // and whether the last pass MEASURED anything.
  //
  // Depth State is the row that matters, and it is the node's own account rather
  // than anything worked out here. The three numbers below it cannot stand in for
  // it: they come from the node's stereo_data_dict, which is only replaced by a pass
  // that got as far as running the stereo process. Every earlier way the loop can
  // stop -- a camera not publishing, no L/R pair inside the sync tolerance, no
  // calibration loaded -- leaves the blank dict's 0.0 in place, which reads on
  // screen as "ran and matched nothing" when nothing ran. Both states also LOOK the
  // same in the viewer, since a pass that matches no pixel still publishes a
  // colorized image and that image is a single flat color. So the state line is
  // shown first, the percentage second, and the guidance comes from the node, which
  // is the only thing that knows which case this is.
  renderDepthOutput() {
    const status_msg = this.props.status_msg
    const depth_message = (status_msg != null) ? status_msg.depth_message : undefined
    const valid_ratio = (status_msg != null) ? status_msg.valid_ratio : undefined
    const min_mm = (status_msg != null) ? status_msg.result_min_depth_mm : undefined
    const max_mm = (status_msg != null) ? status_msg.result_max_depth_mm : undefined
    const measured = (valid_ratio !== undefined && valid_ratio > 0.0)

    return (
      <React.Fragment>

        {/* Empty on a device still running a status message built before this
            field existed, which is the one case with nothing to report. */}
        {(depth_message !== undefined && depth_message !== "") ? (
          <React.Fragment>
            <Label title={"Depth State"} />
            <div style={{ wordWrap: "break-word",
                          color: measured ? Styles.vars.colors.black : Styles.vars.colors.red }}>
              {depth_message}
            </div>
          </React.Fragment>
        ) : null}

        <Label title={"Valid Depth Pixels (%)"}>
          <div style={{ color: measured ? Styles.vars.colors.black : Styles.vars.colors.red }}>
            {measured ? (valid_ratio * 100.0).toFixed(1) : "---"}
          </div>
        </Label>

        <Label title={"Measured Depth Range (mm)"}>
          <div>
            {measured ? (min_mm.toFixed(0) + " - " + max_mm.toFixed(0)) : "---"}
          </div>
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

          {this.renderDepthOutput()}

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
