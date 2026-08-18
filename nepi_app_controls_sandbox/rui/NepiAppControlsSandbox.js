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

import { Columns, Column } from "./Columns"
import Styles from "./Styles"

import NepiIFConnectData from "./Nepi_IF_ConnectData"

import NepiAppControlsSandboxControls from "./NepiAppControlsSandbox-Controls"
import NepiAppControlsSandboxData from "./NepiAppControlsSandbox-Data"
import NepiAppControlsSandboxSettings from "./NepiAppControlsSandbox-Settings"
import NepiAppControlsSandboxTabs from "./NepiAppControlsSandbox-Tabs"


@inject("ros")
@observer

// Controls Sandbox main panel: the left column stacks an Image connect
// selector strip, the image viewer, and a hint caption in one bordered group
// (concept_3_tabbed_groups.html's .imgcol); the right column holds the
// Choices/Actions/Values/Data tab group and, in develop run mode or when
// admin mode is set, a Controls Settings box below it.
//
// The image column follows nepi_app_auto_move for the connect wiring. Its
// connect namespace is <app>/image_connect, owned by the ConnectImageIF in
// controls_sandbox_app_node.py, and the connect name held in state below must
// match that node-side IMAGE_CONNECT_NAME character for character. Two
// Nepi_IF_ConnectData instances share that one namespace: the selector strip
// runs with show_data={false}, and the viewer below it runs with
// show_selector={false} show_data={true}, so whichever topic the operator
// picks in the strip is what streams in the viewer. The viewer topic
// therefore comes from the connect status, not from this app's status
// message, so ControlsSandboxStatus.msg carries no image topic field.
class NepiAppControlsSandbox extends Component {
  constructor(props) {
    super(props)

    this.state = {
      appName: 'app_controls_sandbox',
      appNamespace: null,
      controlsNamespace: null,
      dataNamespace: null,

      // Connect name of the image connect IF this page binds to. Must match
      // IMAGE_CONNECT_NAME in controls_sandbox_app_node.py.
      imageConnectName: 'image_connect',

      status_msg: null,

      statusListener: null,
      needs_update: false
    }

    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.getDataNamespace = this.getDataNamespace.bind(this)
    this.getConnectNamespace = this.getConnectNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.renderImageColumn = this.renderImageColumn.bind(this)
    this.renderControlsDataTabs = this.renderControlsDataTabs.bind(this)
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

  // Connect namespace a Nepi_IF_Connect* component subscribes to, i.e.
  // <app>/<connect_name>, matching the connect_name the connect IF in the node
  // passes to ConnectNodeIF.
  getConnectNamespace(connectName) {
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/" + connectName : null
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

  // Left column: connect selector strip, image viewer, and a hint caption, all
  // sharing one border -- concept_3_tabbed_groups.html's .imgcol, where the
  // connect row sits directly above the viewer instead of in a separate
  // "CONNECTIONS" Section next to the tabs. Two Nepi_IF_ConnectData instances
  // still share the one image_connect namespace exactly as before: this method
  // mounts the selector (show_selector={true} show_data={false}) above the
  // viewer (show_selector={false} show_data={true}), so whichever topic the
  // operator picks in the top strip is what streams below it. Both run
  // make_section={false} since the bordering is drawn here, not by Section.
  renderImageColumn() {
    const connectNamespace = this.getConnectNamespace(this.state.imageConnectName)
    const borderStyle = `1px solid ${Styles.vars.colors.grey1}`

    return (
      <React.Fragment>

        <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", border: borderStyle, borderBottom: "none", padding: `${Styles.vars.spacing.small.raw}px ${Styles.vars.spacing.small.raw + 2}px` }}>
          <Columns>
            <Column>
              <NepiIFConnectData
                namespace={connectNamespace}
                title={"Image"}
                show_selector={true}
                show_data={false}
                make_section={false}
              />
            </Column>
          </Columns>
        </div>

        <div style={{ border: borderStyle, borderTop: "none", borderBottom: "none" }}>
          <Columns>
            <Column>
              <NepiIFConnectData
                namespace={connectNamespace}
                title={"Image"}
                show_selector={false}
                show_data={true}
                make_section={false}
              />
            </Column>
          </Columns>
        </div>

        <div style={{ border: borderStyle, borderTop: "none", padding: `${Styles.vars.spacing.xs.raw}px ${Styles.vars.spacing.small.raw + 2}px`, fontSize: Styles.vars.fontSize.small, color: Styles.vars.colors.grey2 }}>
          Its stream feeds the viewer above.
        </div>

      </React.Fragment>
    )
  }

  // The CONTROLS SANDBOX and DATA SANDBOX boxes, presented as four tabs
  // (Choices, Actions, Values, Data) instead of two stacked Sections. Purely a
  // RUI-side regrouping: createControlsInitDict() / createDataInitDict() in
  // controls_sandbox_app_node.py are unchanged, and every control keeps
  // exactly its current wiring -- each tab mounts the same
  // NepiAppControlsSandboxControls / -Data components used before, just with a
  // type_filter prop so only that tab's control types render. Choices holds
  // the pick-from-a-set controls (Menu, Selection, Selections); Actions holds
  // the toggle and fire controls (Bool, Trigger); Values holds the free-form
  // and numeric value controls (String, Int, Float, FloatSlider,
  // FloatSliders); Data holds the entire read-only Data Sandbox box
  // unfiltered. show_visibility_toggle={false} on every instance suppresses
  // the per-box "Show Controls"/"Show Data" toggle so it does not repeat once
  // per tab -- concept_3_tabbed_groups.html has no such toggle at all.
  renderControlsDataTabs() {
    const controlsNamespace = this.getControlsNamespace()
    const dataNamespace = this.getDataNamespace()

    const tabs = [
      {
        title: "Choices",
        content: (
          <NepiAppControlsSandboxControls
            key={controlsNamespace + "_choices"}
            namespace={controlsNamespace}
            make_section={false}
            show_visibility_toggle={false}
            type_filter={["Menu", "Selection", "Selections"]}
          />
        )
      },
      {
        title: "Actions",
        content: (
          <NepiAppControlsSandboxControls
            key={controlsNamespace + "_actions"}
            namespace={controlsNamespace}
            make_section={false}
            show_visibility_toggle={false}
            type_filter={["Bool", "Trigger"]}
          />
        )
      },
      {
        title: "Values",
        content: (
          <NepiAppControlsSandboxControls
            key={controlsNamespace + "_values"}
            namespace={controlsNamespace}
            make_section={false}
            show_visibility_toggle={false}
            type_filter={["String", "Int", "Float", "FloatSlider", "FloatSliders"]}
          />
        )
      },
      {
        title: "Data",
        content: (
          <NepiAppControlsSandboxData
            key={dataNamespace + "_data"}
            namespace={dataNamespace}
            make_section={false}
            show_visibility_toggle={false}
          />
        )
      }
    ]

    // No Section wrapper here: concept_3_tabbed_groups.html's tab bar sits at
    // the top of its own bordered panel with no titled header above it, unlike
    // the stacked "CONTROLS SANDBOX" / "DATA SANDBOX" Sections it replaces.
    return (
      <NepiAppControlsSandboxTabs tabs={tabs} />
    )
  }

  render() {
    const controlsNamespace = this.getControlsNamespace()

    // Settings box is shown only in develop run mode or when admin mode is set.
    const { systemRunMode, systemAdminModeSet } = this.props.ros
    const show_settings = (systemRunMode === "develop" || systemAdminModeSet === true)

    return (
      <React.Fragment>
        <Columns>
          <Column>

            <div style={{ display: 'flex' }}>

              <div style={{ width: "58%" }}>
                {this.renderImageColumn()}
              </div>

              <div style={{ width: '2%' }}>
                {}
              </div>

              <div style={{ width: "40%" }}>

                {this.renderControlsDataTabs()}

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
