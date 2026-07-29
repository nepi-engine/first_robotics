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
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"

import { onChangeSwitchStateValue } from "./Utilities"

// Stereo calibration panel.
//
// Operator flow: hold a chessboard so BOTH cameras see it, press Capture,
// repeat ~10-20 times at varied distances / angles / image corners, then press
// Solve. The node (custom_stereo_app_node.py) does the work through
// calibrate.StereoCalibrator and reports every result back in
// status_msg.calib_message, which is shown verbatim below the buttons.
//
// Topics (all under the app namespace):
//   set_calib_board_value  UpdateFloat  board_cols / board_rows / square_mm
//   set_calib_file         String       where the .npz is written / read
//   capture_calib_frame    Empty
//   solve_calib            Empty
//   clear_calib            Empty
//   load_calib             Empty
//
// The parent (NepiAppCustomStereo.js) owns the status subscription and passes
// the current status_msg + app namespace down as props.
@inject("ros")
@observer
class NepiAppCustomStereoCalibration extends Component {
  constructor(props) {
    super(props)

    this.state = {
      show_calibration: false,
      // Local copies so typing is not clobbered by the next status tick.
      board_cols: null,
      board_rows: null,
      square_mm: null,
      calib_file: null,
    }

    this.onUpdateBoardValue = this.onUpdateBoardValue.bind(this)
    this.onKeySaveBoardValue = this.onKeySaveBoardValue.bind(this)
    this.onUpdateCalibFile = this.onUpdateCalibFile.bind(this)
    this.onKeySaveCalibFile = this.onKeySaveCalibFile.bind(this)
    this.onCapture = this.onCapture.bind(this)
    this.onSolve = this.onSolve.bind(this)
    this.onClear = this.onClear.bind(this)
    this.onLoad = this.onLoad.bind(this)
    this.renderBoardInputs = this.renderBoardInputs.bind(this)
    this.renderCalibration = this.renderCalibration.bind(this)
  }

  // Adopt values published by the node, but never overwrite a box the operator
  // is in the middle of editing (local value !== null means "being edited" only
  // until Enter/blur resyncs it, so we compare against the last synced value).
  syncFromStatus() {
    const status_msg = this.props.status_msg
    if (status_msg == null) {
      return
    }
    const updates = {}
    if (this.state.board_cols === null) {
      updates.board_cols = status_msg.calib_board_cols
    }
    if (this.state.board_rows === null) {
      updates.board_rows = status_msg.calib_board_rows
    }
    if (this.state.square_mm === null) {
      updates.square_mm = status_msg.calib_square_mm
    }
    if (this.state.calib_file === null) {
      updates.calib_file = status_msg.calib_file
    }
    if (Object.keys(updates).length > 0) {
      this.setState(updates)
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

  sendTrigger(topic) {
    const { sendTriggerMsg } = this.props.ros
    const namespace = this.props.appNamespace
    if (namespace != null) {
      sendTriggerMsg(namespace + "/" + topic)
    }
  }

  // Track typing locally and flag the box as modified (red) until saved.
  onUpdateBoardValue(event, key, id) {
    const updates = {}
    updates[key] = event.target.value
    this.setState(updates)
    const el = document.getElementById(id)
    if (el) {
      el.style.color = Styles.vars.colors.red
    }
  }

  // On Enter, push the edited value as an UpdateFloat(name, value), then drop
  // the local copy so the next status tick shows what the node ACTUALLY took --
  // the node clamps/rejects bad board descriptions, and the box should say so.
  onKeySaveBoardValue(event, name, id, key) {
    if (event.key !== "Enter") {
      return
    }
    const namespace = this.props.appNamespace
    const parsed = parseFloat(event.target.value)
    if (namespace != null && !Number.isNaN(parsed)) {
      this.props.ros.sendUpdateFloatMsg(namespace + "/set_calib_board_value", name, parsed)
    }
    const updates = {}
    updates[key] = null
    this.setState(updates)
    const el = document.getElementById(id)
    if (el) {
      el.style.color = Styles.vars.colors.black
    }
  }

  onUpdateCalibFile(event) {
    this.setState({ calib_file: event.target.value })
    const el = document.getElementById("calib_file")
    if (el) {
      el.style.color = Styles.vars.colors.red
    }
  }

  onKeySaveCalibFile(event) {
    if (event.key !== "Enter") {
      return
    }
    const { sendStringMsg } = this.props.ros
    const namespace = this.props.appNamespace
    if (namespace != null) {
      sendStringMsg(namespace + "/set_calib_file", event.target.value)
    }
    // Re-sync from status: the node appends a missing .npz extension.
    this.setState({ calib_file: null })
    const el = document.getElementById("calib_file")
    if (el) {
      el.style.color = Styles.vars.colors.black
    }
  }

  onCapture() {
    this.sendTrigger("capture_calib_frame")
  }

  onSolve() {
    this.sendTrigger("solve_calib")
  }

  onClear() {
    this.sendTrigger("clear_calib")
  }

  onLoad() {
    this.sendTrigger("load_calib")
  }

  renderBoardInputs() {
    // cols/rows are INNER corner counts: a 10x7-square board is 9x6 corners.
    const board = [
      { id: "calib_board_cols", name: "board_cols", key: "board_cols", title: "Board Corner Columns" },
      { id: "calib_board_rows", name: "board_rows", key: "board_rows", title: "Board Corner Rows" },
      { id: "calib_square_mm", name: "square_mm", key: "square_mm", title: "Square Size (mm)" },
    ]
    return (
      <React.Fragment>
        {board.map((entry) => (
          <Label key={entry.id} title={entry.title}>
            <Input
              id={entry.id}
              style={{ width: "45%", float: "left" }}
              value={this.state[entry.key] !== null ? this.state[entry.key] : ""}
              onChange={(event) => this.onUpdateBoardValue(event, entry.key, entry.id)}
              onKeyDown={(event) => this.onKeySaveBoardValue(event, entry.name, entry.id, entry.key)}
            />
          </Label>
        ))}
      </React.Fragment>
    )
  }

  renderCalibration() {
    const status_msg = this.props.status_msg
    if (status_msg == null) {
      return (<Columns><Column></Column></Columns>)
    }

    const show_calibration = this.state.show_calibration
    const calib_loaded = status_msg.calib_loaded
    const capture_count = status_msg.calib_capture_count
    const min_captures = status_msg.calib_min_captures
    const epipolar_rms = status_msg.calib_epipolar_rms_px
    const can_solve = capture_count >= min_captures

    // Red until a calibration is loaded and rectifying, so the operator can
    // tell at a glance whether depth is metric or guessed. (Only .red/.black
    // are used anywhere else in this app's RUI, so stick to those.)
    const loaded_color = calib_loaded ? Styles.vars.colors.black : Styles.vars.colors.red

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Show Stereo Calibration"}>
          <Toggle
            checked={show_calibration}
            onClick={() => onChangeSwitchStateValue.bind(this)("show_calibration", show_calibration)}
          />
        </Label>

        <div hidden={show_calibration === false}>

          <Label title={"Calibration Loaded"}>
            <div style={{ color: loaded_color, fontWeight: "bold" }}>
              {calib_loaded ? "YES" : "NO"}
            </div>
          </Label>

          <Label title={"Focal Length (px)"}>
            <div>{calib_loaded ? status_msg.calib_focal_length_px.toFixed(1) : "---"}</div>
          </Label>

          <Label title={"Baseline (mm)"}>
            <div>{calib_loaded ? status_msg.calib_baseline_mm.toFixed(1) : "---"}</div>
          </Label>

          {/* Rectified row alignment from the last solve. Under 1 px is good;
              above that, block matching will not find reliable matches, so
              flag it red. */}
          <Label title={"Epipolar RMS (px)"}>
            <div style={{ color: (epipolar_rms >= 1.0)
              ? Styles.vars.colors.red : Styles.vars.colors.black }}>
              {epipolar_rms > 0.0 ? epipolar_rms.toFixed(3) : "---"}
            </div>
          </Label>

          <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.xs, marginBottom: Styles.vars.spacing.xs }} />

          {this.renderBoardInputs()}

          <Label title={"Calibration File"}>
            <Input
              id={"calib_file"}
              style={{ width: "95%" }}
              value={this.state.calib_file !== null ? this.state.calib_file : ""}
              onChange={this.onUpdateCalibFile}
              onKeyDown={this.onKeySaveCalibFile}
            />
          </Label>

          <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.xs, marginBottom: Styles.vars.spacing.xs }} />

          <Label title={"Captured Views"}>
            <div>{capture_count + " / " + min_captures + " minimum"}</div>
          </Label>

          {/* Hold the board so BOTH cameras see it, then Capture. Vary
              distance, angle and where the board sits in the frame. */}
          <ButtonMenu>
            <Button onClick={this.onCapture}>{"Capture View"}</Button>
            <Button onClick={this.onSolve} disabled={can_solve === false}>{"Solve + Save"}</Button>
          </ButtonMenu>

          <ButtonMenu>
            <Button onClick={this.onClear}>{"Clear Views"}</Button>
            <Button onClick={this.onLoad}>{"Load Saved"}</Button>
          </ButtonMenu>

          <Label title={"Last Result"} />
          <div style={{ wordWrap: "break-word" }}>
            {status_msg.calib_message}
          </div>

        </div>

      </React.Fragment>
    )
  }

  render() {
    return this.renderCalibration()
  }
}

export default NepiAppCustomStereoCalibration
