/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
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
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import AsyncToggle from "./AsyncToggle"
import BooleanIndicator from "./BooleanIndicator"
import { SliderAdjustment } from "./AdjustmentWidgets"

import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFSaveData from "./Nepi_IF_SaveData"
import NepiIFConfig from "./Nepi_IF_Config"

import { createMenuFirstLastNames } from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
}

@inject("ros")
@observer

// Obstacles app main panel. Left column is the overlay image viewer, right
// column is the app panel: depth map source selection, process controls, the
// algorithm's ControlsIF box, overlay toggles and status.
//
// This page binds to ONE app node, not to a manager list. The status topic is
// <app>/obstacles/status carrying nepi_app_obstacles/ObstaclesStatus, and every
// command topic hangs off the namespace that message reports in
// process_status.namespace -- which is <app>/obstacles, the namespace
// ObstaclesIF registers its subscribers on. The algorithm's own controls are
// rendered by the shared Nepi_IF_Controls against status_msg.controls_topic.
class NepiAppObstacles extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_obstacles",
      appNamespace: null,

      status_msg: null,
      process_status_msg: null,
      connected: false,

      sources_list_viewable: true,

      selected_display_topic: "None",
      selected_display_text: "None",

      statusListener: null,
      needs_update: false
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getProcessNamespace = this.getProcessNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.getSaveNamespace = this.getSaveNamespace.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

    this.createSourceTopicsOptions = this.createSourceTopicsOptions.bind(this)
    this.toggleSourcesListViewable = this.toggleSourcesListViewable.bind(this)
    this.onSourceTopicSelected = this.onSourceTopicSelected.bind(this)

    this.getDisplayImgOptions = this.getDisplayImgOptions.bind(this)
    this.onDisplayImgSelected = this.onDisplayImgSelected.bind(this)
    this.getSegmentImgTopics = this.getSegmentImgTopics.bind(this)

    this.renderApp = this.renderApp.bind(this)
    this.renderAppSettings = this.renderAppSettings.bind(this)
  }

  getBaseNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null) {
      baseNamespace = "/" + namespacePrefix + "/" + deviceId
    }
    return baseNamespace
  }

  getAppNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null) {
      if (this.props.namespace !== undefined) {
        appNamespace = this.props.namespace
      } else {
        appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
      }
    }
    return appNamespace
  }

  // Namespace every obstacles command topic hangs off. Prefer what the node
  // reports so the two can never drift; fall back to the conventional path.
  getProcessNamespace() {
    const process_status_msg = this.state.process_status_msg
    if (process_status_msg != null && process_status_msg.namespace) {
      return process_status_msg.namespace
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/obstacles" : null
  }

  getControlsNamespace() {
    const status_msg = this.state.status_msg
    if (status_msg != null && status_msg.controls_topic) {
      return status_msg.controls_topic
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/controls" : null
  }

  getSaveNamespace() {
    const process_status_msg = this.state.process_status_msg
    if (process_status_msg != null && process_status_msg.save_data_topic) {
      return process_status_msg.save_data_topic
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/save_data" : "None"
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      status_msg: message,
      process_status_msg: message.process_status,
      connected: true
    })
  }

  // Function for configuring and subscribing to Status
  updateStatusListener(namespace) {
    const statusNamespace = namespace + "/obstacles/status"
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    var statusListener = this.props.ros.setupStatusListener(
          statusNamespace,
          "nepi_app_obstacles/ObstaclesStatus",
          this.statusListener
        )
    this.setState({
      appNamespace: namespace,
      statusListener: statusListener,
    })
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated || this.state.needs_update === true) {
      if (namespace !== null && namespace.indexOf('null') === -1) {
        this.setState({ needs_update: false })
        this.updateStatusListener(namespace)
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    this.setState({
      status_msg: null,
      process_status_msg: null,
      connected: false,
      statusListener: null,
      selected_display_topic: "None",
      selected_display_text: "None"
    })
  }

  //////////////////////////
  // Source selection

  // Options come from the app's own available_source_topics, which ObstaclesIF
  // fills by discovering DepthMapStatus publishers. The RUI does not do its own
  // topic filtering -- the node is the single source of truth for what this
  // process can consume.
  createSourceTopicsOptions() {
    const process_status_msg = this.state.process_status_msg
    var items = []
    items.push(<Option value={'None'}>{'None'}</Option>)
    if (process_status_msg == null) {
      return items
    }
    const source_options = process_status_msg.available_source_topics
    if (source_options.length === 0) {
      return items
    }
    items.push(<Option value={'All'}>{'All'}</Option>)
    const sourceShortnames = createMenuFirstLastNames(source_options)
    for (var i = 0; i < source_options.length; i++) {
      items.push(<Option value={source_options[i]}>{sourceShortnames[i]}</Option>)
    }
    return items
  }

  toggleSourcesListViewable() {
    const set = !this.state.sources_list_viewable
    this.setState({ sources_list_viewable: set })
  }

  onSourceTopicSelected(event) {
    const { sendStringMsg, sendStringArrayMsg } = this.props.ros
    const process_namespace = this.getProcessNamespace()
    const process_status_msg = this.state.process_status_msg
    if (process_namespace == null || process_status_msg == null) {
      return
    }
    const source_options = process_status_msg.available_source_topics
    const selected_sources = process_status_msg.selected_sources
    const source_topic = event.target.value

    if (source_topic === "None") {
      sendStringArrayMsg(process_namespace + "/remove_source_topics", source_options)
    }
    else if (source_topic === "All") {
      sendStringArrayMsg(process_namespace + "/add_source_topics", source_options)
    }
    else if (selected_sources.indexOf(source_topic) === -1) {
      sendStringMsg(process_namespace + "/add_source_topic", source_topic)
    }
    else {
      sendStringMsg(process_namespace + "/remove_source_topic", source_topic)
    }
  }

  //////////////////////////
  // App panel

  renderApp() {
    const status_msg = this.state.status_msg

    return (
      <Section title={"Obstacles"}>

        <div hidden={(status_msg != null)}>
          <pre style={{ height: "50px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
            {"Loading..."}
          </pre>
        </div>

        {(status_msg != null) ? this.renderAppSettings() : null}

      </Section>
    )
  }

  renderAppSettings() {
    const { sendBoolMsg } = this.props.ros

    const status_msg = this.state.status_msg
    const process_status_msg = this.state.process_status_msg
    const process_namespace = this.getProcessNamespace()
    const controls_namespace = this.getControlsNamespace()

    const enabled = process_status_msg.enabled
    const running = process_status_msg.running
    const processing = process_status_msg.state

    const max_process_rate_hz = process_status_msg.max_process_rate_hz
    const max_image_pub_rate_hz = process_status_msg.max_image_pub_rate_hz

    const imaging_enabled = process_status_msg.image_pub_enabled
    const use_last_image = process_status_msg.use_last_image

    const auto_select_active = process_status_msg.auto_select_active

    const selected_sources = process_status_msg.selected_sources

    const source_selected = process_status_msg.source_selected
    const source_connected = process_status_msg.source_connected

    const avg_process_latency = round(process_status_msg.avg_process_latency, 3)
    const avg_process_rate = round(process_status_msg.avg_process_rate, 3)

    const source_options = this.createSourceTopicsOptions()

    const navpose_connected = status_msg.navpose_topic_connected

    const full_screen_enabled = status_msg.full_screen_enabled
    const show_ground_enabled = status_msg.show_ground_enabled
    const show_obstacles_enabled = status_msg.show_obstacles_enabled

    // 0.0 fully opaque through 1.0 invisible on the wire; the sliders below run
    // 0-100 with scaled 0.01, the RUI's standard ratio slider shape.
    const ground_transparency = status_msg.ground_transparency
    const obstacles_transparency = status_msg.obstacles_transparency

    return (
      <Columns>
      <Column>

        <Columns>
        <Column>

          <Label title="Auto Select Source">
            <AsyncToggle
              checked={auto_select_active === true}
              onClick={() => sendBoolMsg(process_namespace + "/set_auto_select_enable", !auto_select_active)}>
            </AsyncToggle>
          </Label>

          <Label title={"Select Depth Maps"} />

        </Column>
        <Column>

          <div style={{ marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

          <div onClick={this.toggleSourcesListViewable} style={{ backgroundColor: Styles.vars.colors.grey0 }}>
            <Select style={{ width: "10px" }} />
          </div>
          <div hidden={this.state.sources_list_viewable === false}>
          {source_options.map((source) =>
            <div onClick={this.onSourceTopicSelected}
              style={{
                textAlign: "center",
                padding: `${Styles.vars.spacing.xs}`,
                color: Styles.vars.colors.black,
                backgroundColor: (selected_sources.indexOf(source.props.value) !== -1) ?
                  Styles.vars.colors.blue : Styles.vars.colors.grey0,
                cursor: "pointer",
              }}>
              <body source-topic={source} style={{ color: Styles.vars.colors.black }}>{source}</body>
            </div>
          )}
          </div>

        </Column>
        </Columns>

        <Columns>
        <Column>

          <Label title="Enable">
            <AsyncToggle
              checked={enabled === true}
              onClick={() => sendBoolMsg(process_namespace + "/enable", !enabled)}>
            </AsyncToggle>
          </Label>

        </Column>
        <Column>
        </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"STATUS"}></Label>

        <div style={{ display: 'flex' }}>
          <div style={{ width: '40%' }}>
            <Label title={"Source Selected"}>
              <BooleanIndicator value={source_selected} />
            </Label>
            <Label title={"NavPose"}>
              <BooleanIndicator value={navpose_connected} />
            </Label>
          </div>

          <div style={{ width: '20%' }}>
            {}
          </div>

          <div style={{ width: '40%' }}>
            <Label title={"Source Connected"}>
              <BooleanIndicator value={source_connected} />
            </Label>
          </div>
        </div>

        <div style={{ display: 'flex' }}>
          <div style={{ width: '40%' }}>
            <Label title={"Running"}>
              <BooleanIndicator value={running} />
            </Label>
          </div>

          <div style={{ width: '20%' }}>
            {}
          </div>

          <div style={{ width: '40%' }}>
            <Label title={"Detect State"}>
              <BooleanIndicator value={processing} />
            </Label>
          </div>
        </div>

        <pre style={{ height: "60px" }} align={"left"} textAlign={"left"}>
        {"\n Avg Process Rate: " + avg_process_rate +
         "\n Avg Process Latency: " + avg_process_latency}
        </pre>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <label style={{ fontWeight: 'bold' }} align={"left"} textAlign={"left"}>
          {"Process Settings"}
        </label>

        <SliderAdjustment
          title={"Max Process Rate"}
          msgType={"std_msgs/Float32"}
          adjustment={max_process_rate_hz}
          topic={process_namespace + "/set_max_process_rate"}
          scaled={1.0}
          min={1}
          max={20}
          disabled={false}
          tooltip={"Sets obstacle process max rate in hz"}
          unit={"Hz"}
        />

        <SliderAdjustment
          title={"Max Image Publish Rate"}
          msgType={"std_msgs/Float32"}
          adjustment={max_image_pub_rate_hz}
          topic={process_namespace + "/set_max_image_pub_rate"}
          scaled={1.0}
          min={1}
          max={20}
          disabled={false}
          tooltip={"Sets overlay image max publish rate in hz"}
          unit={"Hz"}
        />

        <Columns>
        <Column>

          <Label title="Publish Image">
            <AsyncToggle
              checked={imaging_enabled === true}
              onClick={() => sendBoolMsg(process_namespace + "/set_image_pub", imaging_enabled === false)}>
            </AsyncToggle>
          </Label>

          <div hidden={imaging_enabled === false}>
            <Label title="Use Last Image">
              <AsyncToggle
                checked={use_last_image === true}
                onClick={() => sendBoolMsg(process_namespace + "/set_use_last_image", use_last_image === false)}>
              </AsyncToggle>
            </Label>

            <Label title="Show Ground">
              <AsyncToggle
                checked={show_ground_enabled === true}
                onClick={() => sendBoolMsg(process_namespace + "/set_show_ground", show_ground_enabled === false)}>
              </AsyncToggle>
            </Label>

            {/* Each overlay's transparency slider sits under its own show
                toggle and is hidden while that overlay is off -- with nothing
                drawn there is nothing for it to adjust. */}
            <div hidden={show_ground_enabled === false}>
              <SliderAdjustment
                title={"Ground Transparency"}
                msgType={"std_msgs/Float32"}
                adjustment={ground_transparency}
                topic={process_namespace + "/set_ground_transparency"}
                scaled={0.01}
                min={0}
                max={100}
                disabled={false}
                tooltip={"Sets ground overlay transparency, 100% is invisible"}
                unit={"%"}
              />
            </div>

            <Label title="Show Obstacles">
              <AsyncToggle
                checked={show_obstacles_enabled === true}
                onClick={() => sendBoolMsg(process_namespace + "/set_show_obstacles", show_obstacles_enabled === false)}>
              </AsyncToggle>
            </Label>

            <div hidden={show_obstacles_enabled === false}>
              <SliderAdjustment
                title={"Obstacles Transparency"}
                msgType={"std_msgs/Float32"}
                adjustment={obstacles_transparency}
                topic={process_namespace + "/set_obstacles_transparency"}
                scaled={0.01}
                min={0}
                max={100}
                disabled={false}
                tooltip={"Sets obstacles overlay transparency, 100% is invisible"}
                unit={"%"}
              />
            </div>

            <Label title="Full Screen">
              <AsyncToggle
                checked={full_screen_enabled === true}
                onClick={() => sendBoolMsg(process_namespace + "/set_full_screen", full_screen_enabled === false)}>
              </AsyncToggle>
            </Label>
          </div>

        </Column>
        <Column>
        </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        {(controls_namespace != null) ?
          <NepiIFControls
            namespace={controls_namespace}
            title={"Obstacle Detection Controls"}
          />
        : null}

        <NepiIFConfig
          namespace={this.getAppNamespace()}
          title={"Nepi_IF_Config"}
        />

      </Column>
      </Columns>
    )
  }

  //////////////////////////
  // Image viewer

  // The overlay image topics are reported by the node in
  // process_status.imaging_pub_topics, one per active source.
  getDisplayImgOptions() {
    const { imageTopics } = this.props.ros
    var items = []
    const process_status_msg = this.state.process_status_msg

    var selected_image_topic = this.state.selected_display_topic
    const selected_image_topic_found = (imageTopics.indexOf(selected_image_topic)) !== -1

    if (process_status_msg == null) {
      items.push(<Option value={"None"}>{"None"}</Option>)
      return items
    }

    const image_pub_topics = process_status_msg.imaging_pub_topics
    const image_names = createMenuFirstLastNames(image_pub_topics)
    if (image_pub_topics.length === 0) {
      items.push(<Option value={"None"}>{"None"}</Option>)
      if (selected_image_topic !== 'None') {
        this.setState({ selected_display_topic: "None", selected_display_text: "None" })
      }
      return items
    }

    if (selected_image_topic_found === false) {
      selected_image_topic = image_pub_topics[0]
      if (imageTopics.indexOf(selected_image_topic) !== -1) {
        this.setState({ selected_display_topic: selected_image_topic, selected_display_text: image_names[0] })
      }
    }
    for (var i = 0; i < image_pub_topics.length; i++) {
      if (imageTopics.indexOf(image_pub_topics[i]) !== -1) {
        items.push(<Option value={image_pub_topics[i]}>{image_names[i]}</Option>)
        if ((selected_image_topic === "None" || selected_image_topic === '') && i === 0) {
          this.setState({ selected_display_topic: image_pub_topics[i], selected_display_text: image_names[i] })
        }
      }
    }
    return items
  }

  onDisplayImgSelected(event) {
    const source_topic = event.target.value
    const names = createMenuFirstLastNames([source_topic])
    this.setState({
      selected_display_topic: source_topic,
      selected_display_text: names[0]
    })
  }

  // The two segmentation viewers follow whatever source the main viewer is on,
  // so the operator picks a source once. The node builds imaging_pub_topics and
  // the two segmentation lists from the same active source order, so the index
  // of the selected overlay topic indexes both pairs. A selection that is not in
  // the list yet -- first render, or a source that has just been purged --
  // falls back to the first available pair.
  getSegmentImgTopics() {
    const status_msg = this.state.status_msg
    const process_status_msg = this.state.process_status_msg
    if (status_msg == null || process_status_msg == null) {
      return ["None", "None"]
    }
    const image_pub_topics = process_status_msg.imaging_pub_topics
    const ground_topics = status_msg.ground_image_pub_topics
    const obstacles_topics = status_msg.obstacles_image_pub_topics
    var index = image_pub_topics.indexOf(this.state.selected_display_topic)
    if (index === -1) {
      index = 0
    }
    const ground_topic = (ground_topics.length > index && ground_topics[index] !== '') ? ground_topics[index] : "None"
    const obstacles_topic = (obstacles_topics.length > index && obstacles_topics[index] !== '') ? obstacles_topics[index] : "None"
    return [ground_topic, obstacles_topic]
  }

  render() {
    const { imageTopics } = this.props.ros
    const img_options = this.getDisplayImgOptions()
    const selected_image_topic_topic = this.state.selected_display_topic
    const img_publishing = imageTopics.indexOf(selected_image_topic_topic) !== -1

    const selected_image_topic = (img_publishing === true && this.state.connected === true) ? selected_image_topic_topic : "None"
    const selected_image_topic_text = (selected_image_topic_topic === 'None') ? 'No Image Selected' :
      img_publishing ? this.state.selected_display_text : 'Waiting for image to publish'

    // Same publishing/connected gate as the main viewer above, so each
    // segmentation viewer shows a waiting title instead of a broken image
    // before its topic is advertised.
    const segment_img_topics = this.getSegmentImgTopics()
    const ground_img_topic = segment_img_topics[0]
    const obstacles_img_topic = segment_img_topics[1]

    const ground_publishing = imageTopics.indexOf(ground_img_topic) !== -1
    const ground_image_topic = (ground_publishing === true && this.state.connected === true) ? ground_img_topic : "None"
    const ground_image_topic_text = (ground_img_topic === 'None') ? 'No Ground Map Available' :
      ground_publishing ? 'Ground Depth Map' : 'Waiting for image to publish'

    const obstacles_publishing = imageTopics.indexOf(obstacles_img_topic) !== -1
    const obstacles_image_topic = (obstacles_publishing === true && this.state.connected === true) ? obstacles_img_topic : "None"
    const obstacles_image_topic_text = (obstacles_img_topic === 'None') ? 'No Obstacles Map Available' :
      obstacles_publishing ? 'Obstacles Depth Map' : 'Waiting for image to publish'

    const save_data_topic = this.getSaveNamespace()

    return (
      <Columns>
      <Column equalWidth={false}>

        <Columns>
        <Column>

          <Label title="Select Image">
            <Select id="ImgSelect" onChange={this.onDisplayImgSelected}
              value={selected_image_topic}
              disabled={false}>
              {img_options}
            </Select>
          </Label>

        </Column>
        <Column>
        </Column>
        </Columns>

        <NepiIFImageViewer
          image_topic={selected_image_topic}
          title={selected_image_topic_text}
          show_res_orient={false}
          save_data_topic={save_data_topic}
        />

        {(save_data_topic !== 'None' && this.state.connected === true) ?
          <NepiIFSaveData
            saveNamespace={save_data_topic}
            title={"Nepi_IF_SaveData"}
          />
        : null}

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        {/* The two segmentation renders for the source the main viewer is on.
            Their per-image save and render control rows are hidden so they fit
            the narrow side-by-side columns -- the same choice the stereo cam
            page makes for its two input previews. */}
        <Columns>
        <Column>

          <Label title={"Ground Depth Map"} />

          <NepiIFImageViewer
            image_topic={ground_image_topic}
            title={ground_image_topic_text}
            show_res_orient={false}
            show_save_controls={false}
            show_image_controls={false}
          />

        </Column>
        <Column>

          <Label title={"Obstacles Depth Map"} />

          <NepiIFImageViewer
            image_topic={obstacles_image_topic}
            title={obstacles_image_topic_text}
            show_res_orient={false}
            show_save_controls={false}
            show_image_controls={false}
          />

        </Column>
        </Columns>

      </Column>
      <Column>

        {this.renderApp()}

      </Column>
      </Columns>
    )
  }

}

export default NepiAppObstacles
