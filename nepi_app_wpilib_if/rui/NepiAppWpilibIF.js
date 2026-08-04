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

import NepiIFConnectDetections from "./Nepi_IF_ConnectDetections"
import NepiIFConnectTargets from "./Nepi_IF_ConnectTargets"
import NepiIFConnectRBX from "./Nepi_IF_ConnectRBX"
import NepiIFConnectMotor from "./Nepi_IF_ConnectMotor"
import NepiIFConnectNavPose from "./Nepi_IF_ConnectNavPose"

import NepiIFConfig from "./Nepi_IF_Config"

@inject("ros")
@observer

// Wpilib Application page
//
// The controls section is one selector per connect IF the node instantiates,
// in node order: Detections, Targets, RBX, Motors, NavPose. Each selector is
// the reusable Nepi_IF_Connect* component for that IF, bound to the connect
// namespace <app>/<connect_name> that the matching ConnectNodeIF subclass owns
// (pattern from NepiAppStereoCam.js). Each component's selector row carries the
// green "Connected" BooleanIndicator driven by that IF's ConnectIFStatus
// connected flag. Data and controls panels are hidden -- this page selects
// sources, it does not drive them.
class NepiAppWpilibIF extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_wpilib_if",
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
    this.renderControls = this.renderControls.bind(this)
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
        "nepi_app_wpilib_if/NepiAppWpilibIFStatus",
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

  // One selector per connect IF, in node order. Each Nepi_IF_Connect* component
  // renders its own selector row: the source/device Select in the left column
  // and the green "Connected" BooleanIndicator in the right column, both driven
  // by that connect namespace's ConnectIFStatus. make_section={false} keeps each
  // component from drawing a bordered box of its own -- the rows share this one
  // panel, separated by the standard RUI divider, with the bold Label carrying
  // the name the per-component Section title used to.
  renderControls() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    return (
      <React.Fragment>

        <Label title={"Detections"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectDetections
          namespace={this.getConnectNamespace("detections_connect")}
          title={"Detections"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          make_section={false}
        />

        {divider}

        <Label title={"Targets"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectTargets
          namespace={this.getConnectNamespace("targets_connect")}
          title={"Targets"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          make_section={false}
        />

        {divider}

        <Label title={"RBX"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectRBX
          namespace={this.getConnectNamespace("rbx_connect")}
          title={"RBX"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          make_section={false}
        />

        {divider}

        <Label title={"Motors"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectMotor
          namespace={this.getConnectNamespace("motor_connect")}
          title={"Motors"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          make_section={false}
        />

        {divider}

        <Label title={"NavPose"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectNavPose
          namespace={this.getConnectNamespace("navpose_connect")}
          title={"NavPose"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          make_section={false}
        />

      </React.Fragment>
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

  // Standard NEPI device-panel split: a ~75% blank spacer on the left where an
  // image viewer would go, a small gutter, and the selectors/config in the
  // right ~23% column. Mirrors NepiDeviceIDX.js render().
  renderBody() {
    return (
      <div style={{ display: 'flex' }}>

        <div style={{ width: "75%" }}>
          {}
        </div>

        <div style={{ width: '2%' }}>
          {}
        </div>

        <div style={{ width: "23%" }}>
          {this.renderControls()}
          {this.renderConfig()}
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

export default NepiAppWpilibIF
