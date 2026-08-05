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

import Section from "./Section"
import { Columns, Column } from "./Columns"

import Nepi_IF_Controls from "./Nepi_IF_Controls"
import Nepi_IF_Data from "./Nepi_IF_Data"
import NepiAppControlsSandboxSettings from "./NepiAppControlsSandbox-Settings"


@inject("ros")
@observer

// Controls Sandbox main panel: a Controls box (one widget per control), a
// read-only Data box (one row per datum) and, in develop run mode or when admin
// mode is set, a Controls Settings box below them.
class NepiAppControlsSandbox extends Component {
  constructor(props) {
    super(props)

    this.state = {
      appName: 'app_controls_sandbox',
      appNamespace: null,
      controlsNamespace: null,
      dataNamespace: null,
      status_msg: null,

      statusListener: null,
      needs_update: false
    }

    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.getDataNamespace = this.getDataNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
  }

  getAppNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null) {
      if (this.props.namespace !== undefined) {
        namespace = this.props.namespace
      } else {
        namespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
      }
    }
    return namespace
  }

  getControlsNamespace() {
    // Prefer the namespace advertised by the app status; fall back to the
    // conventional <app>/controls path.
    if (this.state.status_msg != null && this.state.status_msg.controls_namespace) {
      return this.state.status_msg.controls_namespace
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/controls" : null
  }

  getDataNamespace() {
    // Prefer the namespace advertised by the app status; fall back to the
    // conventional <app>/data path.
    if (this.state.status_msg != null && this.state.status_msg.data_namespace) {
      return this.state.status_msg.data_namespace
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/data" : null
  }

  statusListener(message) {
    this.setState({
      status_msg: message,
      controlsNamespace: message.controls_namespace,
      dataNamespace: message.data_namespace
    })
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
        "nepi_app_controls_sandbox/ControlsSandboxStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ appNamespace: namespace, needs_update: false })
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true) {
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

  render() {
    const controlsNamespace = this.getControlsNamespace()
    const dataNamespace = this.getDataNamespace()

    // Settings box is shown only in develop run mode or when admin mode is set.
    const { systemRunMode, systemAdminModeSet } = this.props.ros
    const show_settings = (systemRunMode === "develop" || systemAdminModeSet === true)

    return (
      <React.Fragment>
        <Columns>
          <Column>

            <div style={{ display: 'flex' }}>

              <div style={{ width: "75%" }}>
                {}
              </div>

              <div style={{ width: '2%' }}>
                {}
              </div>

              <div style={{ width: "23%" }}>

                <Section title={"CONTROLS SANDBOX"}>

                  <Nepi_IF_Controls
                    key={controlsNamespace}
                    namespace={controlsNamespace}
                    make_section={false}
                  />

                </Section>

                <Section title={"DATA SANDBOX"}>

                  <Nepi_IF_Data
                    key={dataNamespace}
                    namespace={dataNamespace}
                    make_section={false}
                  />

                </Section>

                { (show_settings === true) ?
                  <NepiAppControlsSandboxSettings
                    namespace={controlsNamespace}
                    make_section={true}
                  />
                  : null }

              </div>

            </div>

          </Column>
        </Columns>
      </React.Fragment>
    )
  }
}

export default NepiAppControlsSandbox
