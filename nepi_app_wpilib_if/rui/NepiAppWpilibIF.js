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
import Select, { Option } from "./Select"
import BooleanIndicator from "./BooleanIndicator"
import AsyncToggle from "./AsyncToggle"

import NepiIFConnectDetections from "./Nepi_IF_ConnectDetections"
import NepiIFConnectNavPose from "./Nepi_IF_ConnectNavPose"

import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

// Message type every obstacles app publishes on <app>/obstacles/status. The
// Obstacles row's option list is every topic of this type currently on the
// system, with the trailing /obstacles/status stripped back off to give the app
// namespace ConnectObstaclesIF takes.
const OBSTACLES_STATUS_TYPE = "nepi_app_obstacles/ObstaclesStatus"
const OBSTACLES_STATUS_SUFFIX = "/obstacles/status"

// Unselected state of the Obstacles row, matching NONE_NAMESPACE in the node.
const OBSTACLES_NONE = "None"

// How stale the newest app status message may be before the Robot Network
// indicator reads disconnected. The node publishes status at
// STATUS_PUBLISH_RATE_HZ = 1.0, so this is five status periods -- long enough
// that an ordinary scheduling hiccup or a dropped message does not flicker the
// indicator, short enough that a dead node shows within a few seconds.
const STATUS_TIMEOUT_MS = 5000

// How often the indicator re-evaluates its staleness test. One status period,
// so the timeout above is honoured to within one tick.
const CONN_CHECK_INTERVAL_MS = 1000

// The standard NEPI multi-motor contract the app publishes. Subscribed directly
// (rather than through Nepi_IF_ConnectMotor, which binds to a connect IF this
// app does not host) so the feedback readout can show the fields that DO have a
// MotorStatus home -- measured output, position and speed -- beside the ones
// that do not, which ride on the app's own status message.
const MOTORS_STATUS_TYPE = "nepi_interfaces/MotorsStatus"
const MOTOR_STATUS_SUFFIX = "/motor_status"

// A slot holding this motor_id is unmapped, matching UNMAPPED_MOTOR_ID in the
// node. Setting a slot to this, or clearing its box, is how a slot is turned off.
const UNMAPPED_MOTOR_ID = -1

@inject("ros")
@observer

// Wpilib Application page
//
// The Connections section is one selector per connect path the node
// instantiates, in node order: Detections, Obstacles, NavPose. Detections and
// NavPose are the reusable Nepi_IF_Connect* components, bound to the connect
// namespace <app>/<connect_name> that the matching ConnectNodeIF subclass owns
// (pattern from NepiAppStereoCam.js). Obstacles has no ConnectNodeIF and so no
// connect namespace: its row is built here from the same Label/Select
// primitives, listing the obstacles apps discovered over rosbridge and
// publishing the operator's pick to this app's own set_obstacles_namespace
// topic. Data and controls panels are hidden -- this page selects sources, it
// does not drive them.
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

      // Operator's obstacles app selection, held locally so the row shows the
      // pick straight away in the window before the node's next status message
      // reports it back. Once status arrives, status_msg is authoritative.
      obstacles_namespace: OBSTACLES_NONE,

      // Robot Network connection indicator inputs. last_status_time is stamped
      // in statusListener; conn_tick is bumped by a timer so the indicator
      // re-evaluates when status messages STOP arriving, not only when one does.
      last_status_time: null,
      conn_tick: 0,
      connCheckTimer: null,

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

      // Motor slot editor. Per-slot edits are held as strings keyed by slot so a
      // partially typed entry survives a re-render, and the node's status stays
      // authoritative for every slot the operator is not currently editing.
      motor_slot_count_edit: null,
      motor_id_edits: {},
      motor_name_edits: {},

      // Live nepi_interfaces/MotorsStatus from the app's own motors interface.
      motorsStatusNamespace: null,
      motorsStatusListener: null,
      motors_status_msg: null,
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getConnectNamespace = this.getConnectNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getObstaclesNamespaces = this.getObstaclesNamespaces.bind(this)
    this.getSelectedObstaclesNamespace = this.getSelectedObstaclesNamespace.bind(this)
    this.onObstaclesSelected = this.onObstaclesSelected.bind(this)
    this.getRobotNetworkConnected = this.getRobotNetworkConnected.bind(this)
    this.getValidTeamNumber = this.getValidTeamNumber.bind(this)
    this.getSubnetFromTeamNumber = this.getSubnetFromTeamNumber.bind(this)
    this.getTeamNumberFromSubnet = this.getTeamNumberFromSubnet.bind(this)
    this.onChangeTeamNumber = this.onChangeTeamNumber.bind(this)
    this.onKeyTeamNumber = this.onKeyTeamNumber.bind(this)
    this.settingsListener = this.settingsListener.bind(this)
    this.updateSettingsListener = this.updateSettingsListener.bind(this)
    this.getSettingValue = this.getSettingValue.bind(this)
    this.getMotorIds = this.getMotorIds.bind(this)
    this.getMotorNames = this.getMotorNames.bind(this)
    this.getMotorFeedback = this.getMotorFeedback.bind(this)
    this.getSeenMotorIds = this.getSeenMotorIds.bind(this)
    this.getMotorSlotCountValue = this.getMotorSlotCountValue.bind(this)
    this.onChangeMotorSlotCount = this.onChangeMotorSlotCount.bind(this)
    this.onKeyMotorSlotCount = this.onKeyMotorSlotCount.bind(this)
    this.getMotorIdValue = this.getMotorIdValue.bind(this)
    this.onChangeMotorId = this.onChangeMotorId.bind(this)
    this.onKeyMotorId = this.onKeyMotorId.bind(this)
    this.getMotorNameValue = this.getMotorNameValue.bind(this)
    this.onChangeMotorName = this.onChangeMotorName.bind(this)
    this.onKeyMotorName = this.onKeyMotorName.bind(this)
    this.motorsStatusListener = this.motorsStatusListener.bind(this)
    this.updateMotorsStatusListener = this.updateMotorsStatusListener.bind(this)
    this.getMotorStatus = this.getMotorStatus.bind(this)
    this.onToggleRbxEnabled = this.onToggleRbxEnabled.bind(this)
    this.renderMotors = this.renderMotors.bind(this)
    this.renderMotorSlot = this.renderMotorSlot.bind(this)
    this.renderRobotControl = this.renderRobotControl.bind(this)
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

  // Live list of obstacles app namespaces, read off the topic/type lists the
  // ros store already keeps for the whole system (the same lists Nepi_IF_Messages
  // and NepiMgrScripts read). No new discovery mechanism: an obstacles app is
  // any node publishing <app>/obstacles/status as an ObstaclesStatus.
  getObstaclesNamespaces() {
    const { topicNames, topicTypes } = this.props.ros
    var namespaces = []
    if (topicNames == null || topicTypes == null) {
      return namespaces
    }
    for (var i = 0; i < topicNames.length; i++) {
      if (topicTypes[i] === OBSTACLES_STATUS_TYPE &&
          topicNames[i].endsWith(OBSTACLES_STATUS_SUFFIX)) {
        namespaces.push(topicNames[i].slice(0, -OBSTACLES_STATUS_SUFFIX.length))
      }
    }
    namespaces.sort()
    return namespaces
  }

  // What the Obstacles row shows. The node's status is authoritative once it
  // arrives -- the row reports the namespace the node is actually connected to,
  // not what this page last sent -- with the local pick covering the window
  // before the first status message.
  getSelectedObstaclesNamespace() {
    const status_msg = this.state.status_msg
    if (status_msg != null && status_msg.selected_obstacles_namespace) {
      return status_msg.selected_obstacles_namespace
    }
    return this.state.obstacles_namespace
  }

  onObstaclesSelected(event) {
    const appNamespace = this.getAppNamespace()
    const value = event.target.value
    this.setState({ obstacles_namespace: value })
    if (appNamespace != null) {
      this.props.ros.sendStringMsg(appNamespace + '/set_obstacles_namespace', value)
    }
  }

  // Robot Network connection state. Two inputs, both required: the newest app
  // status message must be recent, AND its connected field must be true. The
  // node currently drives that field from a placeholder that is always set True,
  // so the staleness half is what makes a dead node read red today. When the
  // placeholder is replaced by real NetworkTables connection detection, the
  // field going false turns this red on its own with no change here.
  getRobotNetworkConnected() {
    const status_msg = this.state.status_msg
    const last_status_time = this.state.last_status_time
    if (status_msg == null || last_status_time == null) {
      return false
    }
    if ((Date.now() - last_status_time) > STATUS_TIMEOUT_MS) {
      return false
    }
    return status_msg.connected === true
  }

  statusListener(message) {
    this.setState({
      status_msg: message,
      connected: true,
      last_status_time: Date.now(),
    })
  }

  updateStatusListener(namespace) {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null, last_status_time: null })
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

        // The same entry also sets the app's own team number, which is what its
        // NetworkTables client derives the RoboRIO address from. One operator
        // control, so the device network config and the NT client can never be
        // pointed at two different teams.
        const appNamespace = this.getAppNamespace()
        if (appNamespace != null) {
          this.props.ros.sendIntMsg(appNamespace + '/set_team_number', String(team))
        }
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


  ///////////////////
  // Motor Slots
  //
  // The operator-facing motor model is an ordered list of slots, each holding
  // one RoboRIO motor_id. Slot order IS the NEPI motor index: it selects the
  // channel for MotorControl.motor_ind and it names motor_0, motor_1, ... in
  // MotorsStatus.
  //
  // The motor_id box is FREE ENTRY, not a dropdown of the ids discovered on
  // NetworkTables, and the discovered ids are shown read-only beside it instead.
  // A dropdown whose only options come from live data cannot express the two
  // states that matter most in the pit: mapping slots before the robot code is
  // running at all, and reserving a slot for a motor that is not on the bus yet.
  // See docs/WPILIB_IF_DESIGN.md, Decision 2.

  getMotorIds() {
    const status_msg = this.state.status_msg
    if (status_msg == null || status_msg.motor_ids == null) {
      return []
    }
    return Array.from(status_msg.motor_ids)
  }

  getMotorNames() {
    const status_msg = this.state.status_msg
    if (status_msg == null || status_msg.motor_names == null) {
      return []
    }
    return Array.from(status_msg.motor_names)
  }

  getMotorFeedback() {
    const status_msg = this.state.status_msg
    if (status_msg == null || status_msg.motor_feedback == null) {
      return []
    }
    return Array.from(status_msg.motor_feedback)
  }

  getSeenMotorIds() {
    const status_msg = this.state.status_msg
    if (status_msg == null || status_msg.nt_motor_ids == null) {
      return []
    }
    return Array.from(status_msg.nt_motor_ids)
  }

  getMotorSlotCountValue() {
    if (this.state.motor_slot_count_edit !== null) {
      return this.state.motor_slot_count_edit
    }
    const status_msg = this.state.status_msg
    return (status_msg != null) ? String(status_msg.motor_slot_count) : ""
  }

  onChangeMotorSlotCount(event) {
    const el = document.getElementById("WpilibMotorSlotCount")
    el.style.color = "purple"
    el.style.fontWeight = "bold"
    this.setState({ motor_slot_count_edit: event.target.value })
  }

  onKeyMotorSlotCount(event) {
    if (event.key !== 'Enter') {
      return
    }
    const appNamespace = this.getAppNamespace()
    const text = String(this.state.motor_slot_count_edit).trim()
    // Rejected rather than committed if it is not a plain non-negative integer:
    // the box stays marked so the operator can see the entry was not accepted.
    if (!/^\d{1,3}$/.test(text) || appNamespace == null) {
      return
    }
    const el = document.getElementById("WpilibMotorSlotCount")
    el.style.color = Styles.vars.colors.black
    el.style.fontWeight = "normal"
    this.props.ros.sendIntMsg(appNamespace + '/set_motor_slot_count', text)
    // Cleared so the box follows the node's status again, including the resize
    // the node applies to both per-slot lists.
    this.setState({ motor_slot_count_edit: null, motor_id_edits: {}, motor_name_edits: {} })
  }

  getMotorIdValue(slot) {
    const edits = this.state.motor_id_edits
    if (edits[slot] !== undefined) {
      return edits[slot]
    }
    const motor_ids = this.getMotorIds()
    if (slot >= motor_ids.length) {
      return ""
    }
    // An unmapped slot shows an empty box rather than a bare -1, which reads as
    // a value the operator typed.
    return (motor_ids[slot] === UNMAPPED_MOTOR_ID) ? "" : String(motor_ids[slot])
  }

  onChangeMotorId(slot, event) {
    const el = document.getElementById("WpilibMotorId" + slot)
    el.style.color = "purple"
    el.style.fontWeight = "bold"
    var edits = Object.assign({}, this.state.motor_id_edits)
    edits[slot] = event.target.value
    this.setState({ motor_id_edits: edits })
  }

  onKeyMotorId(slot, event) {
    if (event.key !== 'Enter') {
      return
    }
    const appNamespace = this.getAppNamespace()
    if (appNamespace == null) {
      return
    }
    const text = String(this.getMotorIdValue(slot)).trim()
    // A blank box unmaps the slot. Anything else must be a plain integer.
    var value = UNMAPPED_MOTOR_ID
    if (text !== "") {
      if (!/^-?\d{1,5}$/.test(text)) {
        return
      }
      value = parseInt(text, 10)
    }

    // The whole list is sent in one message, built from the node's current
    // mapping with this slot replaced, so slot order can never be left
    // half-updated by editing one box.
    var motor_ids = this.getMotorIds().map((motor_id) => parseInt(motor_id, 10))
    while (motor_ids.length <= slot) {
      motor_ids.push(UNMAPPED_MOTOR_ID)
    }
    motor_ids[slot] = value

    const el = document.getElementById("WpilibMotorId" + slot)
    el.style.color = Styles.vars.colors.black
    el.style.fontWeight = "normal"
    this.props.ros.sendStringMsg(appNamespace + '/set_motor_ids', motor_ids.join(','))

    var edits = Object.assign({}, this.state.motor_id_edits)
    delete edits[slot]
    this.setState({ motor_id_edits: edits })
  }

  getMotorNameValue(slot) {
    const edits = this.state.motor_name_edits
    if (edits[slot] !== undefined) {
      return edits[slot]
    }
    const motor_names = this.getMotorNames()
    return (slot < motor_names.length) ? String(motor_names[slot]) : ""
  }

  onChangeMotorName(slot, event) {
    const el = document.getElementById("WpilibMotorName" + slot)
    el.style.color = "purple"
    el.style.fontWeight = "bold"
    var edits = Object.assign({}, this.state.motor_name_edits)
    // Commas are stripped on entry: the ordered name list travels as one
    // comma-separated String, so a comma inside a name would split it into two
    // slots. Stripping here keeps that impossible instead of surprising.
    edits[slot] = String(event.target.value).replace(/,/g, '')
    this.setState({ motor_name_edits: edits })
  }

  onKeyMotorName(slot, event) {
    if (event.key !== 'Enter') {
      return
    }
    const appNamespace = this.getAppNamespace()
    if (appNamespace == null) {
      return
    }
    var motor_names = this.getMotorNames().map((name) => String(name))
    while (motor_names.length <= slot) {
      motor_names.push("")
    }
    motor_names[slot] = String(this.getMotorNameValue(slot))

    const el = document.getElementById("WpilibMotorName" + slot)
    el.style.color = Styles.vars.colors.black
    el.style.fontWeight = "normal"
    this.props.ros.sendStringMsg(appNamespace + '/set_motor_names', motor_names.join(','))

    var edits = Object.assign({}, this.state.motor_name_edits)
    delete edits[slot]
    this.setState({ motor_name_edits: edits })
  }

  // The app's own motors interface, publishing the standard MotorsStatus. Its
  // namespace comes off the app status rather than being assumed, so if the
  // interface fails to start the readout simply shows no measured values instead
  // of subscribing to a topic that does not exist.
  motorsStatusListener(message) {
    this.setState({ motors_status_msg: message })
  }

  updateMotorsStatusListener() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return
    }
    const motors_namespace = status_msg.motors_namespace
    if (motors_namespace == null || motors_namespace === "" || motors_namespace === "None") {
      return
    }
    const namespace = motors_namespace + MOTOR_STATUS_SUFFIX
    if (this.state.motorsStatusNamespace === namespace) {
      return
    }
    if (this.state.motorsStatusListener != null) {
      this.state.motorsStatusListener.unsubscribe()
    }
    const listener = this.props.ros.setupStatusListener(
      namespace,
      MOTORS_STATUS_TYPE,
      this.motorsStatusListener
    )
    this.setState({ motorsStatusNamespace: namespace, motorsStatusListener: listener })
  }

  // One MotorStatus by NEPI motor name, or null.
  getMotorStatus(nepi_motor_name) {
    const motors_status_msg = this.state.motors_status_msg
    if (motors_status_msg == null || motors_status_msg.motors == null) {
      return null
    }
    for (var i = 0; i < motors_status_msg.motors.length; i++) {
      if (motors_status_msg.motors[i].motor_name === nepi_motor_name) {
        return motors_status_msg.motors[i]
      }
    }
    return null
  }

  onToggleRbxEnabled() {
    const appNamespace = this.getAppNamespace()
    const status_msg = this.state.status_msg
    if (appNamespace == null || status_msg == null) {
      return
    }
    this.props.ros.sendBoolMsg(appNamespace + '/set_rbx_enabled',
                               status_msg.rbx_enabled !== true)
  }


  componentDidMount() {
    // Re-render on a fixed tick so the Robot Network indicator notices status
    // messages that stopped arriving. Nothing reads conn_tick; the render it
    // forces is the point.
    const connCheckTimer = setInterval(
      () => this.setState((prevState) => ({ conn_tick: prevState.conn_tick + 1 })),
      CONN_CHECK_INTERVAL_MS
    )
    this.setState({ needs_update: true, connCheckTimer: connCheckTimer })
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
    // Same shape: subscribes once the app status names the motors namespace, and
    // no-ops on every update after that.
    this.updateMotorsStatusListener()
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.settingsListener != null) {
      this.state.settingsListener.unsubscribe()
    }
    if (this.state.motorsStatusListener != null) {
      this.state.motorsStatusListener.unsubscribe()
    }
    if (this.state.connCheckTimer != null) {
      clearInterval(this.state.connCheckTimer)
    }
  }

  // One selector per connect path, in node order, all inside ONE bordered
  // Section -- the connect rows are a single panel of source selections, not
  // three unrelated blocks, so the box goes around the whole set.
  // make_section={false} keeps each component from drawing a bordered box of its
  // own inside that panel.
  //
  // Each row is ONE line: the connect name on the left, its dropdown on the
  // right, nothing else. That is the whole of what shortened={true} means to the
  // shared components -- no header line, no "Connected" BooleanIndicator, and
  // the title prop used as the row's only label in place of their hardcoded
  // second word ("Detector", "NavPose Source"). The prop defaults to false, so
  // this page asking for a compact row changes nothing for any other consumer.
  // No dividers -- three single lines read as one list without them.
  //
  // The Obstacles row is built here rather than by a shared component: there is
  // no Nepi_IF_ConnectObstacles.js, because ConnectObstaclesIF is not a
  // ConnectNodeIF and publishes no ConnectIFStatus for one to bind to. It uses
  // the same Label/Select primitives so the three rows are indistinguishable.
  renderControls() {
    const obstacles_namespaces = this.getObstaclesNamespaces()
    const obstacles_selected = this.getSelectedObstaclesNamespace()

    var obstacles_items = []
    obstacles_items.push(<Option value={OBSTACLES_NONE}>{OBSTACLES_NONE}</Option>)
    for (var i = 0; i < obstacles_namespaces.length; i++) {
      obstacles_items.push(
        <Option value={obstacles_namespaces[i]}>{obstacles_namespaces[i]}</Option>
      )
    }

    return (
      <Section title={"Connections"}>

        <NepiIFConnectDetections
          namespace={this.getConnectNamespace("detections_connect")}
          title={"Dete"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          shortened={true}
          make_section={false}
        />

        <Columns>
          <Column>

            <Label title={"Obstacles"}>
              <Select
                onChange={this.onObstaclesSelected}
                value={obstacles_selected}
              >
                {obstacles_items}
              </Select>
            </Label>

          </Column>
        </Columns>

        <NepiIFConnectNavPose
          namespace={this.getConnectNamespace("navpose_connect")}
          title={"NavPose"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          shortened={true}
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

  // The Robot Network box. A connection indicator on top, then one editable box
  // for the team number, then three disabled boxes showing the subnet derived
  // from it and the addresses on that subnet, as the device config currently
  // holds them. Commit is on Enter in the team number box -- there is no
  // separate apply button, matching the ocean aero group this is adapted from.
  //
  // The connection indicator sits on the title line rather than in a row of its
  // own. Section renders its title prop as a child node ({title}), not as a
  // string, so an element passed there lays out beside the heading text with no
  // change to the shared Section.js. The heading div supplies the uppercase bold
  // styling; the flex wrapper only pins the indicator to the same baseline.
  renderRobotNetwork() {
    const team_number = this.state.team_number
    const network_connected = this.getRobotNetworkConnected()
    // Live values read from the device system config.
    const vehicle_subnet = this.getSettingValue('NEPI_VEHICLE_SUBNET')
    const nepi_ip = this.getSettingValue('NEPI_ALIAS_IP_1')
    const ntp_ip = this.getSettingValue('NEPI_NTP_IP')

    const section_title = (
      <div style={{ display: 'flex', alignItems: 'center' }}>

        <div>{"Robot Network"}</div>

        <div style={{ marginLeft: Styles.vars.spacing.regular }}>
          <BooleanIndicator value={network_connected} />
        </div>

      </div>
    )

    return (
      <Section title={section_title}>

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

  // The Motors box. Two box-level rows on top -- how many slots there are, and
  // the motor_ids actually seen on NetworkTables -- then one self-contained
  // group per slot, rendered by renderMotorSlot.
  //
  // Everything belonging to a slot lives inside that slot's group. The earlier
  // layout listed every id and name box first and every feedback readout
  // second, which put a slot's own boxes eight rows away from its own indicator
  // and left neighbouring rows looking paired when they were not. Slot-level
  // titles are also kept to one short line each: a title that wraps to two lines
  // while its box stays top-aligned is what made adjacent rows read as grouped.
  renderMotors() {
    const motor_feedback = this.getMotorFeedback()
    const seen_motor_ids = this.getSeenMotorIds()
    const slot_count_value = this.getMotorSlotCountValue()

    var slot_groups = []
    for (var slot = 0; slot < motor_feedback.length; slot++) {
      slot_groups.push(this.renderMotorSlot(motor_feedback[slot], slot))
    }

    return (
      <Section title={"Motors"}>

        <Label title={"Motor Slots"}>
          <Input
            id={"WpilibMotorSlotCount"}
            value={slot_count_value}
            onChange={this.onChangeMotorSlotCount}
            onKeyDown={this.onKeyMotorSlotCount}
            placeholder={"e.g. 4"}
          />
        </Label>

        <Label title={"Ids On Robot"}>
          <Input disabled value={seen_motor_ids.join(', ')} />
        </Label>

        <div style={{
          marginTop: Styles.vars.spacing.small,
          fontSize: Styles.vars.fontSize.small,
          color: Styles.vars.colors.grey1
        }}>
          {"Slot order is the NEPI motor index: slot 0 is motor_0. Ids On Robot " +
           "are the motor_ids currently seen on NetworkTables."}
        </div>

        {slot_groups}

      </Section>
    )
  }

  // One motor slot as a single group: the NEPI motor name and live indicator on
  // the heading line, then that slot's two editable boxes indented under it,
  // then that slot's readout. A rule above each group separates it from the one
  // before.
  //
  // The readout joins both halves of the motor contract: measured output,
  // position and speed come from the standard nepi_interfaces/MotorsStatus the
  // app publishes, and control mode, commanded output and current come from the
  // app's own status message because MotorStatus has no field for them.
  renderMotorSlot(feedback, slot_ind) {
    const motor_status = this.getMotorStatus(feedback.nepi_motor_name)
    const measured = (motor_status != null) ? motor_status.motor_speed_ratio : null
    const position = (motor_status != null) ? motor_status.motor_position : null
    const speed = (motor_status != null) ? motor_status.motor_speed : null

    var detail = "unmapped"
    if (feedback.motor_id >= 0 && feedback.seen !== true) {
      detail = "id " + feedback.motor_id + " not seen"
    } else if (feedback.motor_id >= 0 && feedback.fresh !== true) {
      detail = "id " + feedback.motor_id + " stale"
    } else if (feedback.motor_id >= 0) {
      detail = "id " + feedback.motor_id + "  " + feedback.control_mode +
               "  out " + Number(feedback.commanded_output).toFixed(2) +
               "/" + ((measured != null) ? Number(measured).toFixed(2) : "-") +
               "  " + Number(feedback.current_amps).toFixed(1) + "A"
      if (position != null) {
        detail = detail + "  pos " + Number(position).toFixed(2) +
                 "  vel " + Number(speed).toFixed(2)
      }
    }

    const heading = feedback.nepi_motor_name +
      ((feedback.display_name !== "") ? "  (" + feedback.display_name + ")" : "")

    return (
      <div key={"motor_slot_" + slot_ind}>

        <div style={{
          borderTop: "1px solid #ffffff",
          marginTop: Styles.vars.spacing.medium,
          marginBottom: Styles.vars.spacing.xs
        }}/>

        <Label title={<span style={{ fontWeight: 'bold' }}>{heading}</span>}>
          <BooleanIndicator value={feedback.seen === true && feedback.fresh === true} />
        </Label>

        <Label title={"RoboRIO Id"} marginLeft={Styles.vars.spacing.regular}>
          <Input
            id={"WpilibMotorId" + slot_ind}
            value={this.getMotorIdValue(slot_ind)}
            onChange={(event) => this.onChangeMotorId(slot_ind, event)}
            onKeyDown={(event) => this.onKeyMotorId(slot_ind, event)}
            placeholder={"unmapped"}
          />
        </Label>

        <Label title={"Display Name"} marginLeft={Styles.vars.spacing.regular}>
          <Input
            id={"WpilibMotorName" + slot_ind}
            value={this.getMotorNameValue(slot_ind)}
            onChange={(event) => this.onChangeMotorName(slot_ind, event)}
            onKeyDown={(event) => this.onKeyMotorName(slot_ind, event)}
            placeholder={"optional"}
          />
        </Label>

        <div style={{
          marginTop: Styles.vars.spacing.xs,
          marginLeft: Styles.vars.spacing.regular,
          fontSize: Styles.vars.fontSize.small,
          color: Styles.vars.colors.grey1
        }}>
          {detail}
        </div>

      </div>
    )
  }

  // The Robot Control box: the RBX device toggle, the namespace the device
  // appears at once it is on, and the RBX Feedback readout.
  //
  // The namespace reads "None" until the RoboRIO has reported its supported
  // capabilities, because RBXRobotIF derives and caches its capability flags at
  // construction -- so the device is deliberately not built until there is a
  // real capability list to build it from.
  renderRobotControl() {
    const status_msg = this.state.status_msg
    const rbx_enabled = (status_msg != null) ? status_msg.rbx_enabled === true : false
    const rbx_namespace = (status_msg != null) ? status_msg.rbx_namespace : ""
    const rbx_ready = (status_msg != null) ? status_msg.rbx_ready === true : false
    const navpose_topic = (status_msg != null) ? status_msg.navpose_topic : ""
    const navpose_valid = (status_msg != null) ? status_msg.navpose_valid === true : false
    const capabilities = (status_msg != null && status_msg.supported_capabilities != null)
      ? Array.from(status_msg.supported_capabilities) : []
    const request_status = (status_msg != null) ? status_msg.request_status : ""
    const request_id = (status_msg != null) ? status_msg.active_request_id : ""
    const request_type = (status_msg != null) ? status_msg.active_request_type : ""
    const status_message = (status_msg != null) ? status_msg.status_message : ""

    return (
      <Section title={"Robot Control"}>

        <Label title={this.renderSubLabel("Robot Device", "present this robot to NEPI as RBX")}>
          <AsyncToggle
            checked={rbx_enabled}
            onClick={this.onToggleRbxEnabled}>
          </AsyncToggle>
        </Label>

        <div hidden={rbx_enabled === false}>

          <Label title={"RBX Namespace"}>
            <Input disabled value={rbx_namespace} />
          </Label>

          <Label title={"RBX Ready"}>
            <BooleanIndicator value={rbx_ready} />
          </Label>

          <Label title={this.renderSubLabel("NavPose", navpose_topic)}>
            <BooleanIndicator value={navpose_valid} />
          </Label>

          <Label title={"Robot Capabilities"}>
            <Input disabled value={capabilities.join(', ')} />
          </Label>

          <Label title={this.renderSubLabel("Request", "id " + request_id + " " + request_type)}>
            <Input disabled value={request_status} />
          </Label>

          <Label title={"Request Message"}>
            <Input disabled value={status_message} />
          </Label>

        </div>

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
          {this.renderRobotNetwork()}
          {this.renderMotors()}
          {this.renderRobotControl()}
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

export default NepiAppWpilibIF
