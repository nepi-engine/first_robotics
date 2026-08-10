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

import NepiIFConfig from "./Nepi_IF_Config"

import NepiAppObstaclesImages from "./NepiAppObstacles-Images"
import NepiAppObstaclesData from "./NepiAppObstacles-Data"
import NepiAppObstaclesControls from "./NepiAppObstacles-Controls"

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
// targets source is whichever discovered AI detector reports that color image
// among the images it is running on (a detector publishes targets under its OWN
// node namespace, never under the image's -- getTargetsTopic() searches rather
// than joins), and each image is named by the status of the source it belongs to.
// A selector on any of those could only offer the operator a way to disagree with
// the depth map they just picked, and a targets source belonging to a different
// camera than the depth map is not a configuration worth being able to express.
//
// So this page renders exactly one <Select> for a source: the Depth Map one, which
// Nepi_IF_ConnectData owns, mounted by NepiAppObstacles-Data.js. The Targets and
// NavPose rows are read-only text plus the same Connected indicator they carried as
// selectors, and every viewer in NepiAppObstacles-Images.js reads its topic off
// this app's status message (depth_map_image_topic / image_topic /
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
//
// STATE OWNERSHIP -- a deliberate departure from Nepi_IF_ConnectPTX. Do not "fix"
// this back to child-owned subscriptions.
//
// This page is split into three children the way Nepi_IF_ConnectPTX splits into
// Nepi_IF_PTX-Data and Nepi_IF_PTX-Controls, with ONE difference: in ConnectPTX
// each child owns its own device status subscription, and here they own none. This
// page is the SOLE owner of every ROS subscription on the page -- the app's own
// NepiAppObstaclesStatus, and the three ConnectIFStatus subscriptions on
// depth_map_connect, targets_connect and navpose_connect -- and the sole owner of
// the derivation helpers getDerivedAreCurrent(), getDerivedTopic(),
// getDerivedConnected() and isTopic(). The three children take already-derived
// values as PROPS and are presentational.
//
// The reason is the "clear before re-deriving" guard documented above. It requires
// that every derived value on the page agree about whether the current derivation
// is stale, and staleness is decided by ONE comparison: the depth map connect's
// live selected_topic against the depth_map_topic in the app status message. Two or
// three components each computing that comparison from their own independently
// batched subscriptions can disagree within a render pass -- which is exactly the
// split-brain the guard exists to prevent: one viewer clearing while another keeps
// streaming the previous depth map's image, or a read-only row naming one depth
// map's targets source beside another depth map's picture. One owner, one answer,
// passed down. A depth map change clears all three image viewers, the NavPose
// viewer and both derived rows in the same render pass, and each child receives
// 'None Available' rather than a stale topic for the duration of that window.
//
// Each child carries its own copy of isTopic() because each needs the mount gate;
// that is safe where a second derivation would not be, because isTopic() is a pure
// string predicate over a value this page already decided -- the same arrangement
// NepiAppStereoCam-Controls.js has with NepiAppStereoCam.js.
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
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.connectStatusListener = this.connectStatusListener.bind(this)
    this.updateConnectStatusListener = this.updateConnectStatusListener.bind(this)
    this.updateConnectStatusListeners = this.updateConnectStatusListeners.bind(this)
    this.getConnectStatusMsg = this.getConnectStatusMsg.bind(this)
    this.getDerivedAreCurrent = this.getDerivedAreCurrent.bind(this)
    this.getDerivedTopic = this.getDerivedTopic.bind(this)
    this.getDerivedConnected = this.getDerivedConnected.bind(this)
    this.isTopic = this.isTopic.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
    this.renderBody = this.renderBody.bind(this)
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
  // hand the result straight to a child as a topic prop: the isTopic() gate in
  // NepiAppObstacles-Images.renderImageViewer() rejects it and leaves that viewer
  // unmounted.
  getDerivedTopic(field) {
    if (this.getDerivedAreCurrent() === false) {
      return 'None Available'
    }
    const topic = this.state.status_msg[field]
    return (this.isTopic(topic) === true) ? topic : 'None Available'
  }

  // Connection state of a derived source, for its Connected indicator in
  // NepiAppObstacles-Data.js.
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

  // True when a topic string off the status message names a real topic. The node
  // reports 'None' for an unselected source and for a selected depth map that has
  // no depth map image, and an un-set ROS string field arrives as ''. Each child
  // carries its own copy for its mount gate -- see the STATE OWNERSHIP note above
  // for why duplicating this predicate is safe and duplicating a derivation is not.
  isTopic(topic) {
    return (topic != null && topic !== '' && topic !== 'None' && topic !== 'None Available')
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
  //
  // Every derived value the children render is computed HERE, in one pass, so all
  // of them see the same answer from getDerivedAreCurrent() -- see the STATE
  // OWNERSHIP note above. NepiAppObstacles-Controls.js takes the app namespace and
  // the raw status message instead, because nothing it renders is derived from the
  // selected depth map; its Example Controls box is the last thing in the right
  // column, below everything else the page puts there.
  renderBody() {
    const depth_map_image_topic = this.getDerivedTopic('depth_map_image_topic')
    const image_topic = this.getDerivedTopic('image_topic')
    const targets_image_topic = this.getDerivedTopic('targets_image_topic')
    const targets_topic = this.getDerivedTopic('targets_topic')
    const navpose_topic = this.getDerivedTopic('navpose_topic')

    return (
      <div style={{ display: 'flex' }}>

        <div style={{ width: "75%" }}>
          <NepiAppObstaclesImages
            depth_map_image_topic={depth_map_image_topic}
            image_topic={image_topic}
            targets_image_topic={targets_image_topic}
            navpose_topic={navpose_topic}
          />
        </div>

        <div style={{ width: '2%' }}>
          {}
        </div>

        <div style={{ width: "23%" }}>
          <NepiAppObstaclesData
            depthMapConnectNamespace={this.getConnectNamespace(this.state.depthMapConnectName)}
            targets_topic={targets_topic}
            navpose_topic={navpose_topic}
            targets_connected={this.getDerivedConnected(this.state.targetsConnectName, targets_topic)}
            navpose_connected={this.getDerivedConnected(this.state.navposeConnectName, navpose_topic)}
          />
          {this.renderConfig()}
          <NepiAppObstaclesControls
            appNamespace={this.getAppNamespace()}
            status_msg={this.state.status_msg}
          />
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
