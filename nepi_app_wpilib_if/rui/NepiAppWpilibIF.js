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

      // Robot Network group. team_number is the free-typed top box, held as a
      // string so a partially typed entry survives; team_initialized is the
      // one-shot guard that seeds it by reverse-deriving the team number from
      // the subnet the live system config holds. The settings* keys cache the
      // system-config Settings status so the disabled boxes can show live values.
      team_number: "",
      team_initialized: false,

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
    this.getValidTeamNumber = this.getValidTeamNumber.bind(this)
    this.getSubnetFromTeamNumber = this.getSubnetFromTeamNumber.bind(this)
    this.getTeamNumberFromSubnet = this.getTeamNumberFromSubnet.bind(this)
    this.onChangeTeamNumber = this.onChangeTeamNumber.bind(this)
    this.onKeyTeamNumber = this.onKeyTeamNumber.bind(this)
    this.settingsListener = this.settingsListener.bind(this)
    this.updateSettingsListener = this.updateSettingsListener.bind(this)
    this.getSettingValue = this.getSettingValue.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.getExampleControlsNamespace = this.getExampleControlsNamespace.bind(this)
    this.renderExampleControls = this.renderExampleControls.bind(this)
    this.renderSubLabel = this.renderSubLabel.bind(this)
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
  // Adapted from the vehicle subnet control group on the ocean aero system setup
  // page. The operator edits one box -- the FRC team number -- and Enter derives
  // the 10.TE.AM robot subnet from it, then commits that subnet plus every
  // address on it in a single batched updateSettings call to
  // <base_namespace>/settings, the system config Settings IF that system_mgr
  // owns. Every other box, the subnet included, is disabled and reads its value
  // back from that IF's status message, so what they show is what the device
  // config holds, not what this page last sent.

  // Returns the team number as an integer for a valid entry, or null. A valid
  // entry is 1 to 5 digits, non-zero, and lands on a legal 10.TE.AM prefix.
  // The TE octet is the team number over 100, so the prefix stops being a legal
  // IP above team 25599 (which would need octet 256). Entries above that are
  // rejected rather than silently written as an invalid address that
  // system_mgr's IP validator would then refuse.
  getValidTeamNumber(text) {
    if (text === null || text === undefined) {
      return null
    }
    var cleaned = text.trim()
    if (!/^\d{1,5}$/.test(cleaned)) {
      return null
    }
    var team = parseInt(cleaned, 10)
    if (team < 1) {
      return null
    }
    if (Math.floor(team / 100) > 255) {
      return null
    }
    return team
  }

  // Build the "10.TE.AM" /24 network prefix from a team number, per the FRC
  // addressing scheme: the last two digits are the AM octet, everything above
  // them is the TE octet, with leading zeros dropped. Team 1 -> 10.0.1,
  // team 122 -> 10.1.22, team 3456 -> 10.34.56, team 12345 -> 10.123.45.
  getSubnetFromTeamNumber(team) {
    return "10." + Math.floor(team / 100) + "." + (team % 100)
  }

  // Inverse of getSubnetFromTeamNumber, used once to seed the box from the
  // subnet already in the config. Returns the team number as a string, or "" if
  // the subnet is not a 10.TE.AM prefix this scheme could have produced.
  getTeamNumberFromSubnet(subnet) {
    if (subnet === null || subnet === undefined || subnet === "") {
      return ""
    }
    var octets = subnet.trim().split("/")[0].split(".")
    if (octets.length < 3 || octets[0] !== "10") {
      return ""
    }
    if (!/^\d{1,3}$/.test(octets[1]) || !/^\d{1,3}$/.test(octets[2])) {
      return ""
    }
    var team = parseInt(octets[1], 10) * 100 + parseInt(octets[2], 10)
    if (team < 1) {
      return ""
    }
    return String(team)
  }

  onChangeTeamNumber(event) {
    const el = document.getElementById("WpilibTeamNumber")
    el.style.color = "purple"
    el.style.fontWeight = "bold"
    this.setState({ team_number: event.target.value })
  }

  onKeyTeamNumber(event) {
    if (event.key === 'Enter') {
      const team = this.getValidTeamNumber(this.state.team_number)
      const prefix = (team !== null) ? this.getSubnetFromTeamNumber(team) : null
      // Only accept the entry if it is a valid team number. An invalid entry is
      // rejected: it is not committed and the box stays marked (purple/bold).
      if (prefix !== null) {
        const el = document.getElementById("WpilibTeamNumber")
        el.style.color = Styles.vars.colors.black
        el.style.fontWeight = "normal"
        // Push the subnet plus every address on it in a single batch update. The
        // disabled boxes below read the new values back from the live config.
        //
        // Both host octets are pinned rather than carried over from whatever the
        // config held, because on an FRC robot network neither is the operator's
        // to choose. .2 is the roboRIO, fixed by the addressing convention, and
        // it is NEPI's time source. .13 puts NEPI in the .6 to .19 band the FRC
        // static addressing rules leave free for other devices -- .1, .3, .4 and
        // .5 are the radio, the field network and the driver station, and .20 and
        // up may be handed out by DHCP. The gateway (NEPI_GATEWAY_IP) is not
        // driven here at all -- it stays on the System Setup page.
        const base_namespace = this.getBaseNamespace()
        const settingsList = [
          { nameStr: 'NEPI_VEHICLE_SUBNET', typeStr: 'String', valueStr: prefix },
          { nameStr: 'NEPI_ALIAS_IP_1', typeStr: 'String',
            valueStr: prefix + '.13/24' },
          { nameStr: 'NEPI_NTP_IP', typeStr: 'String',
            valueStr: prefix + '.2' }
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
      // Initialize the team number box once, when the box is still empty and the
      // config holds a real (non-NONE) subnet, by reverse-deriving the team
      // number from that subnet. A subnet that is not a 10.TE.AM prefix (a
      // factory 192.168.x, say) yields no team number and leaves the box empty
      // for the operator to fill.
      if (this.state.team_initialized === false && this.state.team_number === "") {
        const subnetInd = namesList.indexOf('NEPI_VEHICLE_SUBNET')
        if (subnetInd !== -1) {
          const subnetVal = valuesList[subnetInd]
          if (subnetVal !== "" && subnetVal !== "NONE") {
            const teamStr = this.getTeamNumberFromSubnet(subnetVal)
            if (teamStr !== "") {
              newState.team_number = teamStr
              newState.team_initialized = true
            }
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

  // Two-line Label title: the operator-facing name on top, a note beneath it in
  // small grey text. Label renders its title inside a <label>, so this returns
  // phrasing content (spans, not divs).
  renderSubLabel(title, note) {
    return (
      <span style={{ display: 'block' }}>
        <span style={{ display: 'block' }}>{title}</span>
        <span style={{
          display: 'block',
          fontSize: Styles.vars.fontSize.small,
          color: Styles.vars.colors.grey1,
          fontWeight: 'normal'
        }}>
          {note}
        </span>
      </span>
    )
  }

  // The Robot Network box. One editable box for the team number, three disabled
  // boxes showing the subnet derived from it and the addresses on that subnet,
  // as the device config currently holds them. Commit is on Enter in the top box
  // -- there is no separate apply button, matching the ocean aero group this is
  // adapted from.
  renderRobotNetwork() {
    const team_number = this.state.team_number
    // Live values read from the device system config.
    const vehicle_subnet = this.getSettingValue('NEPI_VEHICLE_SUBNET')
    const nepi_ip = this.getSettingValue('NEPI_ALIAS_IP_1')
    const ntp_ip = this.getSettingValue('NEPI_NTP_IP')

    return (
      <Section title={"Robot Network"}>

        <Label title={this.renderSubLabel("Team Number", "sets the three below")}>
          <Input
            id={"WpilibTeamNumber"}
            value={team_number}
            onChange={this.onChangeTeamNumber}
            onKeyDown={this.onKeyTeamNumber}
            placeholder={"e.g. 9023"}
          />
        </Label>

        <Label title={"Robot Subnet"}>
          <Input disabled value={vehicle_subnet} />
        </Label>

        <Label title={"NEPI Address"}>
          <Input disabled value={nepi_ip} />
        </Label>

        <Label title={"Time Source"}>
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
