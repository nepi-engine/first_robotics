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
import Input from "./Input"
import BooleanIndicator from "./BooleanIndicator"
import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"

import NepiIFConnectData from "./Nepi_IF_ConnectData"
import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

@inject("ros")
@observer

// Obstacles Application page.
//
// Layout follows NepiAppWpilibIF.js: image viewers fill the left column, the
// selector column and config panel sit on the right, and the page owns no
// bespoke controls. Every connect namespace is <app>/<connect_name>, owned by the
// matching connect IF in obstacles_app_node.py:
//   Depth Map     -> <app>/depth_map_connect     (ConnectDepthMapIF, the ONE selector)
//   Color Image   -> <app>/color_image_connect   (ConnectColorImageIF, no selector, node-driven)
//   Targets       -> <app>/targets_connect       (ConnectTargetsIF, no selector, node-driven)
//   Targets Image -> <app>/targets_image_connect (ConnectColorImageIF, no selector, node-driven)
//   NavPose       -> <app>/navpose_connect       (ConnectNavPoseIF, no selector, node-driven)
// Each connect IF is constructed with an explicit connect_name in the node so the
// binding is greppable from both sides; the names above must match those
// node-side names character for character.
//
// ONE rule governs this page: the operator selects a DEPTH MAP, and everything
// else -- the targets source, the NavPose source, and all three images -- is
// derived from it node-side and rendered read-only here. The chain is spelled out
// in obstacles_app_node.py; in short, the selected depth map's DepthMapStatus
// names its sibling color image (image_topic) and its NavPose (navpose_topic), the
// targets source is the AI detector's targets topic inside that color image's
// namespace, and each image is named by the status of the source it belongs to.
// A selector on any of those could only offer the operator a way to disagree with
// the depth map they just picked, and a targets source belonging to a different
// camera than the depth map is not a configuration worth being able to express.
//
// So this page renders exactly one <Select> for a source: the Depth Map one, which
// Nepi_IF_ConnectData owns. The Targets and NavPose rows are read-only text plus
// the same Connected indicator they carried as selectors, and every viewer reads
// its topic off this app's status message (depth_map_image_topic / image_topic /
// targets_image_topic), the NepiAppStereoCam pattern.
//
// Two ConnectIFStatus subscriptions of this page's own back the read-only rows --
// the same subscription the removed Nepi_IF_Connect* components made, on the same
// namespaces, for the same field. They are needed because those components render
// their title and Connected indicator INSIDE renderSelector(), so mounting one
// with show_selector={false} renders nothing at all. A third, on the depth map
// connect, is what makes requirement "clear before re-deriving" hold across the
// window between an operator moving the Depth Map selector and the node
// republishing: while that connect's live selected_topic disagrees with the
// depth_map_topic in the app status message, every derived value on this page is
// rendered CLEARED rather than at its previous depth map's value. Node-side the
// clear is automatic -- publish_status() recomputes every derived topic from the
// current selection in one pass, and each derivation's ownership gate forces
// 'None' the instant the selection changes.
class NepiAppObstacles extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_obstacles",
      appNamespace: null,

      // Connect names of the three source connect IFs this page binds to. Only
      // the first still carries a selector; the other two back read-only rows.
      // The node's two image connect IFs are absent here on purpose -- nothing on
      // this page binds to them; their viewers read the app status message.
      depthMapConnectName: "depth_map_connect",
      targetsConnectName: "targets_connect",
      navposeConnectName: "navpose_connect",

      status_msg: null,
      connected: false,

      statusListener: null,
      needs_update: true,

      // ConnectIFStatus of each source connect namespace, keyed by connect name.
      // Only 'connected' and 'selected_topic' are read: the derived TOPIC always
      // comes from the app status message, never from the IF's own selection --
      // see connectStatusListener().
      connect_status_msgs: {},
      connectStatusListeners: {},
      connectListenerNamespaces: {},
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getConnectNamespace = this.getConnectNamespace.bind(this)
    this.getActiveControlsNamespace = this.getActiveControlsNamespace.bind(this)
    this.getExampleControlsNamespace = this.getExampleControlsNamespace.bind(this)
    this.renderExampleControls = this.renderExampleControls.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.connectStatusListener = this.connectStatusListener.bind(this)
    this.updateConnectStatusListener = this.updateConnectStatusListener.bind(this)
    this.updateConnectStatusListeners = this.updateConnectStatusListeners.bind(this)
    this.getConnectStatusMsg = this.getConnectStatusMsg.bind(this)
    this.getDerivedAreCurrent = this.getDerivedAreCurrent.bind(this)
    this.getDerivedTopic = this.getDerivedTopic.bind(this)
    this.getDerivedConnected = this.getDerivedConnected.bind(this)
    this.renderDerivedSource = this.renderDerivedSource.bind(this)
    this.onSelectProcess = this.onSelectProcess.bind(this)
    this.onReloadProcesses = this.onReloadProcesses.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderProcessSelector = this.renderProcessSelector.bind(this)
    this.renderProcessControls = this.renderProcessControls.bind(this)
    this.isTopic = this.isTopic.bind(this)
    this.renderImageViewer = this.renderImageViewer.bind(this)
    this.renderImageViewers = this.renderImageViewers.bind(this)
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

  // Process selector handlers. Both publish to this app's own topics, the pair
  // nepi_app_stereo_cam uses: a String carrying the process NAME, and an Empty
  // trigger to re-import the module. The node is the authority on both -- neither
  // handler touches local state, so the dropdown only moves once the node has
  // accepted the change and republished its status.
  onSelectProcess(event) {
    const { sendStringMsg } = this.props.ros
    const appNamespace = this.getAppNamespace()
    if (appNamespace != null) {
      sendStringMsg(appNamespace + "/set_selected_process", event.target.value)
    }
  }

  onReloadProcesses() {
    const { sendTriggerMsg } = this.props.ros
    const appNamespace = this.getAppNamespace()
    if (appNamespace != null) {
      sendTriggerMsg(appNamespace + "/reload_processes")
    }
  }

  // Controls namespace of the ACTIVE obstacle process, or null when there is none
  // to mount.
  //
  // This is the whole of the one-set-at-a-time rule on the RUI side: the node owns
  // one ControlsIF per process in nepi_obstacles.PROCESSES_DICT, names the active
  // one here, and this page mounts a Nepi_IF_Controls on that namespace only. The
  // inactive process's component is never mounted, which is what actually hides it
  // -- the node's set_controls_hidden() call is intent, not suppression, because
  // Nepi_IF_Controls reads each control's own hidden flag and never
  // ControlsStatus.hidden.
  //
  // Falls back to <app>/<selected_process> if the namespace field is unset but the
  // process name is known, for the same reason as above; null when neither is
  // usable, and render() then mounts nothing rather than a dead subscription.
  getActiveControlsNamespace() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    if (this.isTopic(status_msg.active_controls_namespace)) {
      return status_msg.active_controls_namespace
    }
    const appNamespace = this.getAppNamespace()
    if (appNamespace != null && this.isTopic(status_msg.selected_process)) {
      return appNamespace + "/" + status_msg.selected_process
    }
    return null
  }

  // Namespace of this app's example ControlsIF -- the control set that belongs to
  // no process. Distinct from getActiveControlsNamespace() above, which names the
  // ACTIVE process's set and changes as the operator switches processes; this one
  // is fixed for the life of the node. Taken from the app status, which publishes
  // it fully qualified, with the conventional <app>/example_controls path as the
  // fallback for the window before the first status message arrives. Mirrors
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
        "nepi_app_obstacles/NepiAppObstaclesStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ appNamespace: namespace, needs_update: false })
    this.updateConnectStatusListeners(namespace)
  }

  // Callback for a source connect namespace's ConnectIFStatus, stored under the
  // connect name it came from.
  //
  // NOTE what is NOT taken from here: the derived source topic. ConnectNodeIF's
  // discovery tick auto-selects available_topics[0] whenever the current
  // selection is 'None' -- unconditionally, ignoring auto_select_enabled
  // (connect_node_if.py _updaterCb) -- so within a second of selecting a depth
  // map that has no targets source of its own, this message's selected_topic
  // names some unrelated detector. The app status message reports the DERIVATION
  // instead, and that is the only field this page displays. See
  // getDerivedConnected() for what that means for the indicator.
  // The functional setState form is load-bearing: three connect namespaces publish
  // into this one map from three independent subscriptions, and reading
  // this.state.connect_status_msgs to build the replacement would drop a sibling's
  // message whenever two callbacks land in the same React batch.
  connectStatusListener(connectName, message) {
    this.setState((prevState) => {
      var connect_status_msgs = Object.assign({}, prevState.connect_status_msgs)
      connect_status_msgs[connectName] = message
      return { connect_status_msgs: connect_status_msgs }
    })
  }

  // Release any listener held for one connect name and subscribe the new
  // namespace, returning the new listener handle (or null). Sets no state itself --
  // updateConnectStatusListeners() below collects all three and commits them in one
  // setState, because three setState calls in a row inside one lifecycle pass would
  // each read the same pre-batch state and only the last would survive, leaking the
  // other two subscriptions.
  updateConnectStatusListener(connectName, namespace) {
    const connectStatusListeners = this.state.connectStatusListeners
    const listener = (connectStatusListeners != null) ? connectStatusListeners[connectName] : null
    if (listener != null) {
      listener.unsubscribe()
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1) {
      return this.props.ros.setupStatusListener(
        namespace + '/status',
        "nepi_interfaces/ConnectIFStatus",
        (message) => this.connectStatusListener(connectName, message)
      )
    }
    return null
  }

  // Re-point all three source connect listeners at the given app namespace. Called
  // from updateStatusListener() so the connect subscriptions move with the app
  // status subscription and can never be left pointed at a previous device.
  updateConnectStatusListeners(appNamespace) {
    const connectNames = [
      this.state.depthMapConnectName,
      this.state.targetsConnectName,
      this.state.navposeConnectName,
    ]
    var connectStatusListeners = {}
    var connectListenerNamespaces = {}
    var connect_status_msgs = {}
    for (var i = 0; i < connectNames.length; i++) {
      const connectName = connectNames[i]
      const namespace = (appNamespace != null) ? appNamespace + "/" + connectName : null
      connectStatusListeners[connectName] = this.updateConnectStatusListener(connectName, namespace)
      connectListenerNamespaces[connectName] = namespace
      connect_status_msgs[connectName] = null
    }
    this.setState({
      connectStatusListeners: connectStatusListeners,
      connectListenerNamespaces: connectListenerNamespaces,
      connect_status_msgs: connect_status_msgs,
    })
  }

  getConnectStatusMsg(connectName) {
    const connect_status_msgs = this.state.connect_status_msgs
    if (connect_status_msgs == null) {
      return null
    }
    const message = connect_status_msgs[connectName]
    return (message !== undefined) ? message : null
  }

  // True when the derived values in the app status message belong to the depth
  // map that is selected RIGHT NOW.
  //
  // This is the "clear before re-deriving" guard. The operator changes the depth
  // map through Nepi_IF_ConnectData, which publishes select_topic straight to the
  // connect namespace; the node then re-derives and republishes. Between those two
  // moments this page still holds a status message whose targets and NavPose
  // belong to the PREVIOUS depth map, and rendering it would put one depth map's
  // label next to another's for as long as that window lasts. Comparing the depth
  // map connect's live selected_topic against the depth_map_topic the status
  // message was computed from closes it: every derived slot renders cleared until
  // the two agree again.
  //
  // Returns false until both messages exist, which also covers page load.
  getDerivedAreCurrent() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return false
    }
    const connect_status_msg = this.getConnectStatusMsg(this.state.depthMapConnectName)
    if (connect_status_msg == null) {
      return false
    }
    return (connect_status_msg.selected_topic === status_msg.depth_map_topic)
  }

  // A derived topic off the app status message, or 'None Available' when there is
  // none to show -- either because the node could not derive one for the selected
  // depth map, or because the derivation is stale per getDerivedAreCurrent().
  // 'None Available' is already a not-a-topic value to isTopic(), so a caller can
  // hand the result straight to renderImageViewer().
  getDerivedTopic(field) {
    if (this.getDerivedAreCurrent() === false) {
      return 'None Available'
    }
    const topic = this.state.status_msg[field]
    return (this.isTopic(topic) === true) ? topic : 'None Available'
  }

  // Connection state of a derived source, for its Connected indicator.
  //
  // ConnectIFStatus.connected alone is not the answer here the way it was for a
  // selector: the connect IF auto-selects an unrelated source when the derivation
  // is 'None' (see connectStatusListener()), and it reports itself connected to
  // that one. So connected is reported only while the IF is actually subscribed to
  // the topic this page is displaying -- which is what the operator reads the
  // indicator to mean.
  getDerivedConnected(connectName, derivedTopic) {
    if (this.isTopic(derivedTopic) === false) {
      return false
    }
    const connect_status_msg = this.getConnectStatusMsg(connectName)
    if (connect_status_msg == null) {
      return false
    }
    if (connect_status_msg.selected_topic !== derivedTopic) {
      return false
    }
    return (connect_status_msg.connected === true)
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
    const connectStatusListeners = this.state.connectStatusListeners
    if (connectStatusListeners != null) {
      const connectNames = Object.keys(connectStatusListeners)
      for (var i = 0; i < connectNames.length; i++) {
        const listener = connectStatusListeners[connectNames[i]]
        if (listener != null) {
          listener.unsubscribe()
        }
      }
    }
  }

  // One read-only row for a source the node derives from the selected depth map.
  //
  // Deliberately shaped to read as the row it replaces: a bold title and the same
  // green "Connected" BooleanIndicator on one line, then the source itself on the
  // line below -- the show_connect_header={true} layout of Nepi_IF_Connect*, with
  // a disabled Input where the Select used to be. Same layout, same indicator, no
  // choice to make.
  //
  // The disabled Input is the RUI's read-only value display (the same widget
  // Nepi_IF_ConnectTargets.renderData() uses for every field it shows), so an
  // unavailable source reads 'None Available' there rather than blank, and the
  // caption line below says why. The caption is the page's existing affordance --
  // the two lines under the Depth Map row are the same thing.
  renderDerivedSource(title, valueLabel, connectName, derivedTopic, availableCaption, unavailableCaption) {
    const available = this.isTopic(derivedTopic)
    const connected = this.getDerivedConnected(connectName, derivedTopic)

    return (
      <React.Fragment>

        <Columns>
          <Column>

            <Label title={title} labelStyle={{fontWeight: 'bold'}}/>

          </Column>
          <Column>

            <Label title={"Connected"}>
              <BooleanIndicator value={connected} />
            </Label>

          </Column>
        </Columns>

        <Columns>
          <Column>

            <Label title={valueLabel}>
              <Input disabled value={derivedTopic} />
            </Label>

          </Column>
        </Columns>

        <Label title={(available === true) ? availableCaption : unavailableCaption}/>

      </React.Fragment>
    )
  }

  // The CONNECTIONS panel: ONE source selector and two derived, read-only rows --
  // then the obstacle process selector this page renders itself (a dropdown plus
  // Reload Processes, on this app's own topics), and the controls of whichever
  // process it selects, a Nepi_IF_Controls bound to a ControlsIF namespace read off
  // this app's status message.
  //
  // Only the Depth Map row still carries a Select. Targets and NavPose are derived
  // from that depth map node-side and rendered read-only here, for the reason given
  // at the top of this file: they are properties of the depth map the operator
  // picked, not independent choices, and a targets source belonging to a different
  // camera than the depth map is not worth being able to express.
  //
  // Nepi_IF_ConnectTargets / Nepi_IF_ConnectNavPose are gone from this page rather
  // than mounted with show_selector={false}: both render their title and Connected
  // indicator INSIDE renderSelector(), so with the selector off they render nothing
  // at all. renderDerivedSource() reproduces that header row from this page's own
  // ConnectIFStatus subscription on the same namespace.
  //
  // The node's two image connect IFs get no row -- they carry no selector and the
  // node drives them. The three source rows live inside ONE bordered Section: they
  // are a single panel of source connections, so the box goes around the whole set,
  // each row separated from the next by the standard RUI divider, and
  // make_section={false} keeps Nepi_IF_ConnectData from drawing a bordered box of
  // its own inside that panel. The process selector and process controls sit
  // OUTSIDE that box -- they are this app's own controls, not source connections.
  renderControls() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    const targets_topic = this.getDerivedTopic('targets_topic')
    const navpose_topic = this.getDerivedTopic('navpose_topic')

    return (
      <React.Fragment>

        <Section title={"Connections"}>

          <NepiIFConnectData
            namespace={this.getConnectNamespace(this.state.depthMapConnectName)}
            title={"Depth Map"}
            show_selector={true}
            show_data={false}
            show_connect_header={true}
            make_section={false}
          />
          <Label title={"Its depth map image feeds the top viewer"}/>
          <Label title={"Its color image feeds the middle viewer"}/>
          <Label title={"Its targets and NavPose are set below"}/>

          {divider}

          {this.renderDerivedSource(
            "Targets",
            "Targeter",
            this.state.targetsConnectName,
            targets_topic,
            "Its targets image feeds the bottom viewer",
            "No targets source for the selected Depth Map"
          )}

          {divider}

          {this.renderDerivedSource(
            "NavPose",
            "NavPose Source",
            this.state.navposeConnectName,
            navpose_topic,
            "From the selected Depth Map",
            "No NavPose source for the selected Depth Map"
          )}

        </Section>

        {this.renderProcessSelector()}

        {this.renderProcessControls(divider)}

      </React.Fragment>
    )
  }

  // Obstacle process selector: a native dropdown plus a Reload Processes button,
  // the same block NepiAppStereoCam-Controls.js renders and the one exception to
  // this page owning no controls of its own.
  //
  // It is deliberately NOT a Nepi_IF_Controls Selection control. Which process runs
  // is app state that outlives either control set, a reload trigger is unreachable
  // through Nepi_IF_Controls (it sends UpdateString to a topic ControlsIF subscribes
  // as UpdateTrigger, and Store.js has no sendUpdateTriggerMsg), and a selector on a
  // plain topic keeps working when a ControlsIF does not.
  //
  // Both lists come off the app status message, so the menu cannot drift from the
  // processes nepi_obstacles actually registers. The dropdown is disabled while the
  // node reports process_ready false, i.e. mid-reload.
  renderProcessSelector() {
    const status_msg = this.state.status_msg
    const available_processes = (status_msg != null && status_msg.available_processes != null &&
                                 status_msg.available_processes.length > 0)
      ? status_msg.available_processes : ["None"]
    const selected_process = (status_msg != null && this.isTopic(status_msg.selected_process))
      ? status_msg.selected_process : "None"
    const process_ready = (status_msg != null) ? status_msg.process_ready : false

    return (
      <React.Fragment>

        <Label title={"Obstacle Process"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <Label title={"Process"}>
          <Select
            id="set_selected_process"
            onChange={this.onSelectProcess}
            value={selected_process}
            disabled={process_ready === false}
          >
            {available_processes.map((opt) => (
              <Option key={opt} value={opt}>{opt}</Option>
            ))}
          </Select>
        </Label>

        <ButtonMenu>
          <Button onClick={this.onReloadProcesses}>{"Reload Processes"}</Button>
        </ButtonMenu>

      </React.Fragment>
    )
  }

  // Controls of the ACTIVE obstacle process, or nothing.
  //
  // One Nepi_IF_Controls on the namespace the node reports as active. The node
  // owns one controls namespace per process and only this one is ever mounted, so
  // the operator is shown exactly the active process's controls and never the
  // other's -- see getActiveControlsNamespace().
  //
  // key={namespace} makes a process switch REMOUNT the component rather than
  // repoint it, the same guard renderImageViewer() uses on image topics.
  // Nepi_IF_Controls does resubscribe when its namespace prop changes, but it
  // carries per-control edit state (editValues / pending) that belongs to the set
  // it was showing; remounting drops that state with the component instead of
  // letting one process's in-progress edit reconcile against the other's status.
  //
  // allways_show_controls suppresses the component's own "Show Controls" toggle: a
  // collapse toggle inside an already-labelled panel is noise, and the panel's bold
  // Labels are the grouping affordance on this page.
  renderProcessControls(divider) {
    const namespace = this.getActiveControlsNamespace()
    if (namespace == null) {
      return null
    }
    const status_msg = this.state.status_msg
    const title = (status_msg != null && this.isTopic(status_msg.selected_process))
      ? ("Controls: " + status_msg.selected_process)
      : "Process Controls"

    return (
      <React.Fragment>

        {divider}

        <Label title={title} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFControls
          key={namespace}
          namespace={namespace}
          make_section={false}
          allways_show_controls={true}
        />

      </React.Fragment>
    )
  }

  // True when a topic string off the status message names a real topic. The node
  // reports 'None' for an unselected source and for a selected depth map that has
  // no depth map image, and an un-set ROS string field arrives as ''.
  isTopic(topic) {
    return (topic != null && topic !== '' && topic !== 'None' && topic !== 'None Available')
  }

  // One image viewer, or an empty column when there is no topic to mount it on.
  //
  // Nepi_IF_ImageViewer must NOT be mounted with an invalid topic.
  // updateImageSource() gates on `if (this.props.image_topic)` and the string
  // 'None' is truthy, so it would build a web_video_server URL ending in
  // '...None&type=mjpeg' and then leave whatever frame is already on its canvas
  // in place -- nothing clears the canvas when a stream stops. Unmounting instead
  // is the guard Nepi_IF_ConnectData.renderData() uses (empty Columns/Column when
  // the topic is null or 'None'), and it destroys the canvas with the component,
  // so no stale frame can survive a selection change.
  //
  // key={topic} makes a change between two live topics remount the viewer as
  // well, rather than repointing this.image.src underneath a canvas still holding
  // the previous source's last frame.
  renderImageViewer(topic, title) {
    if (this.isTopic(topic) === false) {
      return (
        <Columns>
          <Column>
            <Label title={title} />
          </Column>
        </Columns>
      )
    }

    return (
      <Columns>
        <Column>
          <Label title={title} />
          <NepiIFImageViewer
            key={topic}
            namespace={topic}
            image_topic={topic}
            title={title}
          />
        </Column>
      </Columns>
    )
  }

  // Three viewers stacked vertically, all three topics read off the app status
  // message: the selected depth map's own depth map image on top, that same depth
  // map's sibling color image in the middle, and the targets image of the targets
  // source derived from that depth map at the bottom. Not one of them is hard-wired
  // and not one has a selector -- the node derives each from the status message of
  // the source it belongs to and reports 'None' when it cannot name a real topic. A
  // 'None' leaves that viewer unmounted through the isTopic() gate in
  // renderImageViewer(); a viewer never falls back to a neighbor's stream, so what
  // each one shows is either the image that belongs to its source or nothing at
  // all.
  //
  // Every topic goes through getDerivedTopic(), which adds the second half of that
  // guarantee: while a depth map change is still in flight it returns 'None
  // Available' for all three, so the bottom viewer cannot keep streaming the
  // previous depth map's targets image into the window between the operator's
  // selection and the node's next status message. Unmounting destroys the canvas
  // with the component, which is the only thing that actually clears it -- nothing
  // clears a canvas when a stream merely stops.
  renderImageViewers() {
    const depth_map_image_topic = this.getDerivedTopic('depth_map_image_topic')
    const image_topic = this.getDerivedTopic('image_topic')
    const targets_image_topic = this.getDerivedTopic('targets_image_topic')

    return (
      <React.Fragment>

        {this.renderImageViewer(depth_map_image_topic, "Depth Map Image (from selected Depth Map)")}

        {this.renderImageViewer(image_topic, "Color Image (from selected Depth Map)")}

        {this.renderImageViewer(targets_image_topic, "Targets Image (from derived Targets source)")}

      </React.Fragment>
    )
  }

  // A copy of the controls sandbox app's Controls box, bound to this app's example
  // ControlsIF. Same component, mounted the same way the sandbox page mounts it --
  // make_section={false} inside a Section of its own -- so it looks and behaves
  // identically; the "Show Controls" toggle at the top of the box is
  // Nepi_IF_Controls' own, not something this page adds. Only the Section title
  // differs.
  //
  // This is the page's SECOND Nepi_IF_Controls, and the one that is always mounted.
  // renderProcessControls() mounts the other on the active process's namespace and
  // only while there is one; this box belongs to no process, so it never unmounts.
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

  // Standard NEPI device-panel split: image viewers in the left ~75%, a small
  // gutter, and the selectors/config in the right ~23% column.
  renderBody() {
    return (
      <div style={{ display: 'flex' }}>

        <div style={{ width: "75%" }}>
          {this.renderImageViewers()}
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

export default NepiAppObstacles
