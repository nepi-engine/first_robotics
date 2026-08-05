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
import Styles from "./Styles"

import NepiIFConnectMotor from "./Nepi_IF_ConnectMotor"
import NepiIFConnectNavPose from "./Nepi_IF_ConnectNavPose"
import NepiIFConnectData from "./Nepi_IF_ConnectData"

import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

@inject("ros")
@observer

// AutoMove Application page
//
// Layout follows NepiAppWpilibIF.js: the selectors and config panel sit in the
// right column, the image viewer fills the left column, and the page owns no
// bespoke controls -- it selects sources, it does not drive them. Every connect
// namespace is <app>/<connect_name>, owned by the matching connect IF in
// auto_move_app_node.py:
//   Motor 1 -> <app>/motor_1_connect  (ConnectMotorsDeviceIF)
//   Motor 2 -> <app>/motor_2_connect  (ConnectMotorsDeviceIF)
//   Motor 3 -> <app>/motor_3_connect  (ConnectMotorsDeviceIF)
//   Motor 4 -> <app>/motor_4_connect  (ConnectMotorsDeviceIF)
//   NavPose -> <app>/navpose_connect  (ConnectNavPoseIF)
//   Image   -> <app>/image_connect    (ConnectImageIF)
// Each connect IF is constructed with an explicit connect_name in the node so the
// binding is greppable from both sides; the names held in state below must match
// those node-side names character for character.
//
// The image row is the one place two components share a namespace. The selector
// instance in the right column runs with show_data={false}; a SECOND
// Nepi_IF_ConnectData on the SAME image_connect namespace runs in the left column
// with show_selector={false} show_data={true}, and that second instance is what
// renders Nepi_IF_ImageViewer -- Nepi_IF_ConnectData.renderData() reads the image
// topic off ConnectIFStatus.selected_topic. The viewer topic therefore comes from
// the connect status, not from this app's status message, so
// NepiAppAutoMoveStatus.msg carries no image topic field.
class NepiAppAutoMove extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_auto_move",
      appNamespace: null,

      // Connect names, one per connect IF the node instantiates. Held here
      // rather than inlined at each call site so the six strings that must match
      // the node read as one list.
      motor1ConnectName: "motor_1_connect",
      motor2ConnectName: "motor_2_connect",
      motor3ConnectName: "motor_3_connect",
      motor4ConnectName: "motor_4_connect",
      navposeConnectName: "navpose_connect",
      imageConnectName: "image_connect",

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
    this.renderControls = this.renderControls.bind(this)
    this.renderImageViewer = this.renderImageViewer.bind(this)
    this.getExampleControlsNamespace = this.getExampleControlsNamespace.bind(this)
    this.renderExampleControls = this.renderExampleControls.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
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

  // Connect namespace a Nepi_IF_Connect* component subscribes to, i.e.
  // <app>/<connect_name>, matching the connect_name each connect IF in the node
  // passes to ConnectNodeIF.
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
        "nepi_app_auto_move/NepiAppAutoMoveStatus",
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

  // One selector per connect IF, in node order: the four motors, NavPose, then
  // the image. All of them live inside ONE bordered Section -- the connect rows
  // are a single panel of source selections, not six unrelated blocks, so the
  // box goes around the whole set and each row is separated from the next by the
  // standard RUI divider. make_section={false} keeps each component from drawing
  // a bordered box of its own inside that panel.
  //
  // show_connect_header={true} is what titles each row: the component renders its
  // title prop and its green "Connected" BooleanIndicator on one line ABOVE the
  // Select, both driven by that connect namespace's ConnectIFStatus, so the page
  // no longer renders a bold Label of its own. Nepi_IF_ConnectData reads no
  // show_controls prop, so only show_selector and show_data are passed to the
  // image row; Nepi_IF_ConnectMotor and Nepi_IF_ConnectNavPose read all three.
  renderControls() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    return (
      <Section title={"Connections"}>

        <NepiIFConnectMotor
          namespace={this.getConnectNamespace(this.state.motor1ConnectName)}
          title={"Motor 1"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectMotor
          namespace={this.getConnectNamespace(this.state.motor2ConnectName)}
          title={"Motor 2"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectMotor
          namespace={this.getConnectNamespace(this.state.motor3ConnectName)}
          title={"Motor 3"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectMotor
          namespace={this.getConnectNamespace(this.state.motor4ConnectName)}
          title={"Motor 4"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectNavPose
          namespace={this.getConnectNamespace(this.state.navposeConnectName)}
          title={"NavPose"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectData
          namespace={this.getConnectNamespace(this.state.imageConnectName)}
          title={"Image"}
          show_selector={true}
          show_data={false}
          show_connect_header={true}
          make_section={false}
        />
        <Label title={"Its stream feeds the viewer"}/>

      </Section>
    )
  }

  // The image viewer for the left column: a second Nepi_IF_ConnectData on the
  // SAME image_connect namespace as the selector row, this one carrying only the
  // data panel (show_selector={false} show_data={true}). Its renderData() reads
  // the image topic off ConnectIFStatus.selected_topic and mounts
  // Nepi_IF_ImageViewer on it, and it already handles the unselected case --
  // renderData() returns an empty Columns/Column when selected_topic is null or
  // 'None', so no viewer is mounted on a topic that does not exist. Sharing the
  // namespace rather than adding a second connect IF keeps one selection driving
  // both halves of the row: whatever the operator picks on the right is what
  // streams on the left.
  renderImageViewer() {
    return (
      <Columns>
        <Column>
          <NepiIFConnectData
            namespace={this.getConnectNamespace(this.state.imageConnectName)}
            title={"Image"}
            show_selector={false}
            show_data={true}
            make_section={false}
          />
        </Column>
      </Columns>
    )
  }

  // A copy of the controls sandbox app's Controls box, bound to this app's own
  // example ControlsIF rather than the sandbox app's. Same component, mounted the
  // same way the sandbox page mounts it -- make_section={false} inside a Section of
  // its own -- so it looks and behaves identically; the "Show Controls" toggle at
  // the top of the box is Nepi_IF_Controls' own, not something this page adds. Only
  // the Section title differs.
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

  renderConfig() {
    const appNamespace = this.getAppNamespace()
    return (
      <React.Fragment>
        <NepiIFConfig
          namespace={appNamespace}
          title={"Nepi_IF_Config"}
        />
      </React.Fragment>
    )
  }

  // Standard NEPI device-panel split: the image viewer in the left ~75%, a small
  // gutter, and the selectors/config in the right ~23% column. Mirrors
  // NepiDeviceIDX.js render().
  renderBody() {
    return (
      <div style={{ display: 'flex' }}>

        <div style={{ width: "75%" }}>
          {this.renderImageViewer()}
        </div>

        <div style={{ width: '2%' }}>
          {}
        </div>

        <div style={{ width: "23%" }}>
          {this.renderControls()}
          {this.renderConfig()}
          {this.renderExampleControls()}
        </div>

      </div>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    if (make_section === false) {
      return (
        <Columns>
          <Column>
            {this.renderBody()}
          </Column>
        </Columns>
      )
    } else {
      return (
        <Section>
          {this.renderBody()}
        </Section>
      )
    }
  }
}

export default NepiAppAutoMove
