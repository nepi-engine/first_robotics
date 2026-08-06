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
import { Columns, Column } from "./Columns"
import Label from "./Label"

import NepiIFConnectIDX from "./Nepi_IF_ConnectIDX"
import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

import NepiAppStereoCamControls from "./NepiAppStereoCam-Controls"
import NepiAppStereoCamCalibration from "./NepiAppStereoCam-Calibration"
import NepiAppStereoCamAdvanced from "./NepiAppStereoCam-Advanced"

@inject("ros")
@observer

// Stereo Cam Depth application page (OUTLINE / SKELETON).
//
// Layout, following the referenced patterns:
//   * Two <NepiIFConnectIDX> selectors side by side (left camera, right camera),
//     each bound to its own connect namespace (<app>/left_cam_connect and
//     <app>/right_cam_connect), matching the two ConnectIDXDeviceIF instances in
//     the node (pattern from nepi_app_idx_connect).
//   * A process-select dropdown + Reload button, and the active process's
//     controls as a Nepi_IF_Controls bound to the controls namespace the node
//     reports as active (NepiAppStereoCam-Controls).
//   * Two small image viewers side by side wired to the left/right selected
//     camera image topics (status_msg.left_image_topic / right_image_topic).
//   * One large image viewer wired to the colorized depth image topic
//     (status_msg.depth_map_image_topic, published by the node's DepthMapIF).
//     (pattern from the NepiAppPTAuto-ImageViewer* family.)
class NepiAppStereoCam extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_stereo_cam",
      appNamespace: null,

      status_msg: null,
      connected: false,

      statusListener: null,
      needs_update: true,
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getConnectNamespace = this.getConnectNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.renderSelectors = this.renderSelectors.bind(this)
    this.renderImageViewers = this.renderImageViewers.bind(this)
    this.getExampleControlsNamespace = this.getExampleControlsNamespace.bind(this)
    this.renderExampleControls = this.renderExampleControls.bind(this)
  }

  getBaseNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    if (namespacePrefix !== null && deviceId !== null) {
      return "/" + namespacePrefix + "/" + deviceId
    }
    return null
  }

  getAppNamespace() {
    const base = this.getBaseNamespace()
    if (base !== null) {
      return base + "/" + this.state.appName
    }
    return null
  }

  // Connect namespace a Nepi_IF_ConnectIDX component subscribes to, i.e.
  // <app>/<connect_name>, matching the connect_name passed to each
  // ConnectIDXDeviceIF in the node.
  getConnectNamespace(connectName) {
    const appNamespace = this.getAppNamespace()
    if (appNamespace !== null) {
      return appNamespace + "/" + connectName
    }
    return null
  }

  // Namespace of this app's example ControlsIF. Taken from the app status, which
  // publishes it fully qualified, with the conventional <app>/example_controls path
  // as the fallback for the window before the first status message arrives. Mirrors
  // NepiAppControlsSandbox.js getControlsNamespace().
  getExampleControlsNamespace() {
    const status_msg = this.state.status_msg
    if (status_msg != null && status_msg.example_controls_namespace) {
      return status_msg.example_controls_namespace
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/example_controls" : null
  }

  statusListener(message) {
    this.setState({
      status_msg: message,
      connected: true,
    })
  }

  updateStatusListener(namespace) {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace.indexOf('null') === -1) {
      const statusNamespace = namespace + '/status'
      var statusListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_app_stereo_cam/NepiAppStereoCamStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ appNamespace: namespace, needs_update: false })
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true) {
      this.updateStatusListener(namespace)
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }

  // Two IDX connect selectors side by side (left camera / right camera), inside
  // ONE bordered Section -- the two rows are a single panel of source selections,
  // so the box goes around both rather than around each, and make_section={false}
  // keeps each component from drawing a box of its own inside that panel.
  //
  // show_connect_header={true} titles each column: the component renders its title
  // prop and its green "Connected" BooleanIndicator on one line ABOVE the Select,
  // both driven by that connect namespace's ConnectIFStatus.
  renderSelectors() {
    const leftConnectNamespace = this.getConnectNamespace("left_cam_connect")
    const rightConnectNamespace = this.getConnectNamespace("right_cam_connect")

    return (
      <Section title={"Connections"}>
        <Columns>
          <Column>
            <NepiIFConnectIDX
              namespace={leftConnectNamespace}
              title={"Left Camera"}
              show_selector={true}
              show_data={false}
              show_controls={false}
              show_connect_header={true}
              make_section={false}
            />
          </Column>
          <Column>
            <NepiIFConnectIDX
              namespace={rightConnectNamespace}
              title={"Right Camera"}
              show_selector={true}
              show_data={false}
              show_controls={false}
              show_connect_header={true}
              make_section={false}
            />
          </Column>
        </Columns>
      </Section>
    )
  }

  // Two small viewers side by side (left/right selected camera) + one large
  // viewer (depth map). Topics come from the app status message.
  renderImageViewers() {
    const status_msg = this.state.status_msg
    const left_image_topic = (status_msg != null) ? status_msg.left_image_topic : "None"
    const right_image_topic = (status_msg != null) ? status_msg.right_image_topic : "None"
    // The viewer renders the COLORIZED depth image, not the raw 32FC1 depth
    // array on status_msg.depth_map_topic (which is millimeter floats, not a
    // displayable image). Same split NepiDeviceIDX makes for the depth_map
    // product.
    const depth_map_topic = (status_msg != null) ? status_msg.depth_map_image_topic : "None"

    return (
      <React.Fragment>

        {/* The two input-camera previews just show the image -- the per-image
            save/config control rows are hidden so they fit in the narrow
            side-by-side columns. The depth output viewer (full width) keeps
            its controls. */}
        <Columns>
          <Column>
            <Label title={"Left Camera"} />
            <NepiIFImageViewer
              namespace={left_image_topic}
              image_topic={left_image_topic}
              title={"Left Camera"}
              show_save_controls={false}
              show_image_controls={false}
            />
          </Column>
          <Column>
            <Label title={"Right Camera"} />
            <NepiIFImageViewer
              namespace={right_image_topic}
              image_topic={right_image_topic}
              title={"Right Camera"}
              show_save_controls={false}
              show_image_controls={false}
            />
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"Depth Map"} />
            <NepiIFImageViewer
              namespace={depth_map_topic}
              image_topic={depth_map_topic}
              title={"Depth Map"}
            />
          </Column>
        </Columns>

      </React.Fragment>
    )
  }

  // A copy of the controls sandbox app's Controls box, bound to this app's own
  // example ControlsIF rather than the sandbox app's. Same component, mounted the
  // same way the sandbox page mounts it -- make_section={false} inside a Section of
  // its own -- so it looks and behaves identically; the "Show Controls" toggle at
  // the top of the box is Nepi_IF_Controls' own, not something this page adds. Only
  // the Section title differs.
  //
  // This is the page's SECOND Nepi_IF_Controls, and the one that is always mounted.
  // NepiAppStereoCam-Controls mounts the other on the ACTIVE stereo process's
  // controls namespace and only while there is one; this box belongs to no process,
  // so it never unmounts.
  //
  // The sandbox page also mounts NepiAppControlsSandbox-Settings below the box in
  // develop/admin mode. That component is deliberately NOT copied: it lives in the
  // sandbox app's rui/ directory, so importing it would make this app's RUI build
  // depend on nepi_app_controls_sandbox being installed.
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

  render() {
    const appNamespace = this.getAppNamespace()
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    const body = (
      <React.Fragment>

        {/* Two IDX camera selectors across the top (left cam / right cam). */}
        {this.renderSelectors()}

        {/* Images on the left, control section on the right (pattern from
            NepiAppPTAuto.js). The process dropdown + process settings live in
            the right-hand control column. */}
        <div style={{ display: "flex" }}>

          <div style={{ width: "73%" }}>
            {this.renderImageViewers()}
          </div>

          <div style={{ width: "2%" }} />

          <div style={{ width: "25%" }}>
            <NepiAppStereoCamControls
              appNamespace={appNamespace}
              status_msg={this.state.status_msg}
            />
            {/* Stereo calibration (chessboard capture -> solve -> rectify).
                Lives under the process controls in the same column: it is
                setup you do once, not a per-frame control. */}
            <NepiAppStereoCamCalibration
              appNamespace={appNamespace}
              status_msg={this.state.status_msg}
            />
            {/* Depth framerate cap + the pipeline tunables that used to be
                module constants in the node. Under calibration because it is
                the block an operator reaches for least: the defaults are
                right until the specific cameras on the robot say otherwise. */}
            <NepiAppStereoCamAdvanced
              appNamespace={appNamespace}
              status_msg={this.state.status_msg}
            />
            {/* Bottom of the right-hand column, under everything this page
                already puts there. */}
            {this.renderExampleControls()}
          </div>

        </div>

        <NepiIFConfig
          namespace={appNamespace}
          title={"Nepi_IF_Config"}
        />

      </React.Fragment>
    )

    if (make_section === false) {
      return body
    }
    return (
      <Section title={(this.props.title !== undefined) ? this.props.title : "Stereo Cam Depth"}>
        {body}
      </Section>
    )
  }
}

export default NepiAppStereoCam
