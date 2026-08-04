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

import NepiIFConnectData from "./Nepi_IF_ConnectData"
import NepiIFConnectNavPose from "./Nepi_IF_ConnectNavPose"
import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFConfig from "./Nepi_IF_Config"

@inject("ros")
@observer

// Obstacles Application page.
//
// Layout follows NepiAppWpilibIF.js: image viewers fill the left column, the
// selector column and config panel sit on the right, and the page owns no
// bespoke controls. The three selectors are the reusable Nepi_IF_Connect*
// components, each bound to the connect namespace <app>/<connect_name> owned by
// the matching connect IF in obstacles_app_node.py:
//   Depth Map -> <app>/depth_map_connect    (ConnectDepthMapIF)
//   Image     -> <app>/color_image_connect  (ConnectColorImageIF)
//   NavPose   -> <app>/navpose_connect      (ConnectNavPoseIF)
// Each connect IF is constructed with an explicit connect_name in the node so the
// binding is greppable from both sides; the names above must match those
// node-side names character for character.
//
// The operator selects a depth map, not a depth map image. The node reads the
// associated image topic off the selected depth map's DepthMapStatus and reports
// it in the app status message, so there is no separate image selector.
//
// The two viewers read their topics off this app's status message
// (depth_map_image_topic / image_topic) rather than each subscribing to a
// connect status of its own, the NepiAppStereoCam pattern.
class NepiAppObstacles extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_obstacles",
      appNamespace: null,

      // Connect names, one per connect IF in the node
      depthMapConnectName: "depth_map_connect",
      colorImageConnectName: "color_image_connect",
      navposeConnectName: "navpose_connect",

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
  // renders its own selector row: the source Select in the left column and the
  // green "Connected" BooleanIndicator in the right column, both driven by that
  // connect namespace's ConnectIFStatus. Nepi_IF_ConnectData reads no
  // show_controls prop, so only show_selector and show_data are passed to it.
  // make_section={false} keeps each component from drawing a bordered box of its
  // own -- the rows share this one panel, separated by the standard RUI divider,
  // with the bold Label carrying the name the Section title used to.
  renderControls() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    return (
      <React.Fragment>

        <Label title={"Depth Map"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectData
          namespace={this.getConnectNamespace(this.state.depthMapConnectName)}
          title={"Depth Map"}
          show_selector={true}
          show_data={false}
          make_section={false}
        />
        <Label title={"Its depth map image feeds the top viewer"} align={"left"} textAlign={"left"}/>

        {divider}

        <Label title={"Image"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectData
          namespace={this.getConnectNamespace(this.state.colorImageConnectName)}
          title={"Image"}
          show_selector={true}
          show_data={false}
          make_section={false}
        />

        {divider}

        <Label title={"NavPose"} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}/>
        <NepiIFConnectNavPose
          namespace={this.getConnectNamespace(this.state.navposeConnectName)}
          title={"NavPose"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          make_section={false}
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

  // Two viewers stacked vertically: the selected depth map's own depth map image
  // on top, the selected color image below. Both topics come from the app status
  // message, so the viewers follow whatever the selectors above connect to and
  // neither is hard-wired. depth_map_image_topic is the depth map image belonging
  // to the SELECTED DEPTH MAP -- the node derives it from that depth map's
  // DepthMapStatus rather than from a selector of its own, and reports 'None'
  // when the selected depth map has no image or nothing is selected. The top
  // viewer then shows nothing; it never falls back to the color image below it.
  renderImageViewers() {
    const status_msg = this.state.status_msg
    const depth_map_image_topic = (status_msg != null) ? status_msg.depth_map_image_topic : "None"
    const image_topic = (status_msg != null) ? status_msg.image_topic : "None"

    return (
      <React.Fragment>

        {this.renderImageViewer(depth_map_image_topic, "Depth Map Image (from selected Depth Map)")}

        {this.renderImageViewer(image_topic, "Image")}

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
