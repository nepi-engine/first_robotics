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
import Styles from "./Styles"
import Input from "./Input"
import Label from "./Label"

import NepiIFConnectDetections from "./Nepi_IF_ConnectDetections"
import NepiIFConnectTargets from "./Nepi_IF_ConnectTargets"
import NepiIFConnectRBX from "./Nepi_IF_ConnectRBX"
import NepiIFConnectMotor from "./Nepi_IF_ConnectMotor"
import NepiIFConnectNavPose from "./Nepi_IF_ConnectNavPose"

import NepiIFControls from "./Nepi_IF_Controls"
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

      // Robot Network group. vehicle_subnet is the free-typed top box;
      // subnet_initialized is the one-shot guard that seeds it from the live
      // system config. The settings* keys cache the system-config Settings
      // status so the disabled boxes can show live values.
      vehicle_subnet: "",
      subnet_initialized: false,

      settingsNamespace: 'None',
      settingsListener: null,
      settingsNamesList: [],
      settingsValuesList: [],
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getConnectNamespace = this.getConnectNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getValidSubnetPrefix = this.getValidSubnetPrefix.bind(this)
    this.getDerivedIp = this.getDerivedIp.bind(this)
    this.onChangeVehicleSubnet = this.onChangeVehicleSubnet.bind(this)
    this.onKeyVehicleSubnet = this.onKeyVehicleSubnet.bind(this)
    this.settingsListener = this.settingsListener.bind(this)
    this.updateSettingsListener = this.updateSettingsListener.bind(this)
    this.getSettingValue = this.getSettingValue.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.getExampleControlsNamespace = this.getExampleControlsNamespace.bind(this)
    this.renderExampleControls = this.renderExampleControls.bind(this)
    this.renderSettingLabel = this.renderSettingLabel.bind(this)
    this.renderRobotNetwork = this.renderRobotNetwork.bind(this)
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
        "nepi_app_wpilib_if/NepiAppWpilibIFStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ appNamespace: namespace, needs_update: false })
  }


  ///////////////////
  // Robot Network
  //
  // A copy of the vehicle subnet control group from the ocean aero system setup
  // page, relabeled for an FRC robot network. The operator edits one box -- the
  // 10.TE.AM subnet the robot radio hands out -- and Enter commits that subnet
  // plus every address derived from it in a single batched updateSettings call
  // to <base_namespace>/settings, the system config Settings IF that system_mgr
  // owns. The derived boxes are disabled and read their values back from that
  // IF's status message, so what they show is what the device config holds, not
  // what this page last sent.

  // Returns the "a.b.c" /24 network prefix for a valid subnet entry, or null if
  // the entry is not a valid subnet. Accepts entries like "10.90.23",
  // "10.90.23.0", or "10.90.23.0/24".
  getValidSubnetPrefix(subnet) {
    if (subnet === null || subnet === undefined) {
      return null
    }
    var cleaned = subnet.trim().split("/")[0]
    if (cleaned === "") {
      return null
    }
    var octets = cleaned.split(".")
    if (octets.length < 3) {
      return null
    }
    var prefix_octets = octets.slice(0, 3)
    for (var i = 0; i < prefix_octets.length; i++) {
      var oct = prefix_octets[i]
      if (!/^\d{1,3}$/.test(oct)) {
        return null
      }
      var num = parseInt(oct, 10)
      if (num < 0 || num > 255) {
        return null
      }
    }
    return prefix_octets.join(".")
  }

  // Re-prefix an IP onto a new "a.b.c" network prefix. The host octet and any
  // /mask suffix are preserved from currentValue (e.g. "10.10.10.103/24" with a
  // new prefix "10.90.23" becomes "10.90.23.103/24"). When currentValue is blank
  // or not a parseable IP, defaultHostSuffix is used instead (e.g. "103/24",
  // "1", "2").
  getDerivedIp(currentValue, newPrefix, defaultHostSuffix) {
    var hostSuffix = defaultHostSuffix
    if (currentValue !== null && currentValue !== undefined && currentValue !== "") {
      var parts = currentValue.trim().split("/")
      var octets = parts[0].split(".")
      if (octets.length === 4 && /^\d{1,3}$/.test(octets[3])) {
        hostSuffix = octets[3] + (parts.length > 1 ? "/" + parts[1] : "")
      }
    }
    return newPrefix + "." + hostSuffix
  }

  onChangeVehicleSubnet(event) {
    const el = document.getElementById("WpilibVehicleSubnet")
    el.style.color = "purple"
    el.style.fontWeight = "bold"
    this.setState({ vehicle_subnet: event.target.value })
  }

  onKeyVehicleSubnet(event) {
    if (event.key === 'Enter') {
      const prefix = this.getValidSubnetPrefix(this.state.vehicle_subnet)
      // Only accept the entry if it is a valid subnet. An invalid entry is
      // rejected: it is not committed and the box stays marked (purple/bold).
      if (prefix !== null) {
        const el = document.getElementById("WpilibVehicleSubnet")
        el.style.color = Styles.vars.colors.black
        el.style.fontWeight = "normal"
        // Push the subnet plus the two addresses derived from it in a single
        // batch update. Each address keeps its current host octet and netmask
        // and only swaps the network prefix. The disabled boxes below read the
        // new values back from the live config.
        //
        // Default host suffixes follow the FRC convention for a 10.TE.AM.0/24
        // robot network: .2 is the roboRIO, and NEPI keeps the platform default
        // .103 for its own address. The gateway (NEPI_GATEWAY_IP) is
        // deliberately not driven here -- it stays on the System Setup page.
        const base_namespace = this.getBaseNamespace()
        const settingsList = [
          { nameStr: 'NEPI_VEHICLE_SUBNET', typeStr: 'String', valueStr: prefix },
          { nameStr: 'NEPI_ALIAS_IP_1', typeStr: 'String',
            valueStr: this.getDerivedIp(this.getSettingValue('NEPI_ALIAS_IP_1'), prefix, '103/24') },
          { nameStr: 'NEPI_NTP_IP', typeStr: 'String',
            valueStr: this.getDerivedIp(this.getSettingValue('NEPI_NTP_IP'), prefix, '2') }
        ]
        this.props.ros.updateSettings(base_namespace + '/settings', settingsList)
      }
    }
  }

  // Callback for the system config Settings status message. Caches the setting
  // name/value pairs so the disabled boxes can show live values.
  settingsListener(message) {
    if (message.settings_topic === this.state.settingsNamespace) {
      const settings = message.settings_list
      var namesList = []
      var valuesList = []
      for (let ind = 0; ind < settings.length; ind++) {
        namesList.push(settings[ind].name_str)
        valuesList.push(settings[ind].value_str)
      }
      var newState = {
        settingsNamesList: namesList,
        settingsValuesList: valuesList
      }
      // Initialize the subnet box from the config once, when the box is still
      // empty and the config holds a real (non-NONE) subnet.
      if (this.state.subnet_initialized === false && this.state.vehicle_subnet === "") {
        const subnetInd = namesList.indexOf('NEPI_VEHICLE_SUBNET')
        if (subnetInd !== -1) {
          const subnetVal = valuesList[subnetInd]
          if (subnetVal !== "" && subnetVal !== "NONE") {
            newState.vehicle_subnet = subnetVal
            newState.subnet_initialized = true
          }
        }
      }
      this.setState(newState)
    }
  }

  // Subscribe to the system config Settings status once the base namespace is
  // known (and resubscribe if it changes). Separate from the app status listener
  // above: that one carries this app's own status, this one carries the device
  // system config.
  updateSettingsListener() {
    const base_namespace = this.getBaseNamespace()
    if (base_namespace === null) {
      return
    }
    const settingsNamespace = base_namespace + '/settings'
    if (this.state.settingsNamespace !== settingsNamespace) {
      if (this.state.settingsListener != null) {
        this.state.settingsListener.unsubscribe()
      }
      const listener = this.props.ros.setupSettingsStatusListener(
        settingsNamespace + '/status',
        this.settingsListener
      )
      this.setState({ settingsNamespace: settingsNamespace, settingsListener: listener })
    }
  }

  getSettingValue(name) {
    const namesList = this.state.settingsNamesList
    const valuesList = this.state.settingsValuesList
    const ind = namesList.indexOf(name)
    if (ind !== -1) {
      const value = valuesList[ind]
      // Treat an unset (NONE) config value as blank.
      if (value === "NONE") {
        return ""
      }
      return value
    }
    return ""
  }


  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true) {
      this.updateStatusListener(namespace)
    }
    // Self-guarding: only subscribes when <base_namespace>/settings changes, so
    // calling it on every update settles after the first subscribe rather than
    // looping on its own setState.
    this.updateSettingsListener()
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.settingsListener != null) {
      this.state.settingsListener.unsubscribe()
    }
  }

  // One selector per connect IF, in node order, all inside ONE bordered Section --
  // the connect rows are a single panel of source selections, not five unrelated
  // blocks, so the box goes around the whole set and each row is separated from
  // the next by the standard RUI divider. make_section={false} keeps each
  // component from drawing a bordered box of its own inside that panel.
  //
  // show_connect_header={true} is what titles each row: the component renders its
  // title prop and its green "Connected" BooleanIndicator on one line ABOVE the
  // Select, both driven by that connect namespace's ConnectIFStatus, so the page
  // no longer renders a bold Label of its own.
  renderControls() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    return (
      <Section title={"Connections"}>

        <NepiIFConnectDetections
          namespace={this.getConnectNamespace("detections_connect")}
          title={"Detections"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectTargets
          namespace={this.getConnectNamespace("targets_connect")}
          title={"Targets"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectRBX
          namespace={this.getConnectNamespace("rbx_connect")}
          title={"RBX"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectMotor
          namespace={this.getConnectNamespace("motor_connect")}
          title={"Motors"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectNavPose
          namespace={this.getConnectNamespace("navpose_connect")}
          title={"NavPose"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

      </Section>
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

  // Two-line Label title: the operator-facing name on top, the underlying
  // device config setting name beneath it in small grey text, so what the box
  // writes is visible on the page. Label renders its title inside a <label>,
  // so this returns phrasing content (spans, not divs).
  renderSettingLabel(title, settingName) {
    return (
      <span style={{ display: 'block' }}>
        <span style={{ display: 'block' }}>{title}</span>
        <span style={{
          display: 'block',
          fontSize: Styles.vars.fontSize.small,
          color: Styles.vars.colors.grey1,
          fontWeight: 'normal'
        }}>
          {settingName}
        </span>
      </span>
    )
  }

  // The Robot Network box. One editable box for the robot subnet, three
  // disabled boxes showing the addresses derived from it as the device config
  // currently holds them. Commit is on Enter in the top box -- there is no
  // separate apply button, matching the ocean aero group this is copied from.
  renderRobotNetwork() {
    const vehicle_subnet = this.state.vehicle_subnet
    // Live values read from the device system config.
    const nepi_ip = this.getSettingValue('NEPI_ALIAS_IP_1')
    const ntp_ip = this.getSettingValue('NEPI_NTP_IP')

    return (
      <Section title={"Robot Network"}>

        <Label title={this.renderSettingLabel("Robot Subnet", "NEPI_VEHICLE_SUBNET")}>
          <Input
            id={"WpilibVehicleSubnet"}
            value={vehicle_subnet}
            onChange={this.onChangeVehicleSubnet}
            onKeyDown={this.onKeyVehicleSubnet}
            placeholder={"e.g. 10.90.23"}
          />
        </Label>

        <Label title={this.renderSettingLabel("NEPI Address", "NEPI_ALIAS_IP_1")}>
          <Input disabled value={nepi_ip} />
        </Label>

        <Label title={this.renderSettingLabel("Time Source", "NEPI_NTP_IP")}>
          <Input disabled value={ntp_ip} />
        </Label>

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
          {this.renderRobotNetwork()}
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

export default NepiAppWpilibIF
