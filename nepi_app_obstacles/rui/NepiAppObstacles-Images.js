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

import { Columns, Column } from "./Columns"
import Label from "./Label"

import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFNavPose from "./Nepi_IF_NavPose"

@inject("ros")
@observer

// The viewer column of the Obstacles page: three stacked image viewers and the
// NavPose viewer below them.
//
// Presentational and stateless. This component owns no subscription of its own --
// NepiAppObstacles.js is the sole owner of every ROS subscription on the page and
// passes each of the four topics down ALREADY DERIVED, through its
// getDerivedTopic(). That is deliberate and is the design of this split: see the
// state-ownership comment at the top of NepiAppObstacles.js. What it buys this
// component is that all four topics agree about whether the current derivation is
// stale, so a depth map change clears every viewer here in the SAME render pass
// rather than leaving one viewer streaming the previous depth map's image while
// another has already cleared.
//
// Every topic prop therefore arrives either as a real topic or as the sentinel
// 'None Available', which isTopic() below rejects like 'None' and ''. There is
// deliberately no selector anywhere in this component: each image is a property of
// the depth map the operator picked, derived node-side -- see the header comment
// and NepiAppObstaclesStatus.msg.
class NepiAppObstaclesImages extends Component {

  constructor(props) {
    super(props)

    this.isTopic = this.isTopic.bind(this)
    this.renderImageViewer = this.renderImageViewer.bind(this)
    this.renderImageViewers = this.renderImageViewers.bind(this)
    this.renderNavPoseViewer = this.renderNavPoseViewer.bind(this)
  }

  // True when a topic string handed down by the parent names a real topic. The
  // node reports 'None' for an unselected source and for a selected depth map
  // that has no depth map image, an un-set ROS string field arrives as '', and
  // the parent's getDerivedTopic() substitutes 'None Available' while a
  // derivation is stale. Same predicate the parent applies -- a pure string test
  // with no state, so a local copy cannot disagree with the owner's; the
  // derivation it tests is what stays owned in one place.
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
  // message by the parent: the selected depth map's own depth map image on top,
  // that same depth map's sibling color image in the middle, and the targets image
  // of the targets source derived from that depth map at the bottom. Not one of
  // them is hard-wired and not one has a selector -- the node derives each from the
  // status message of the source it belongs to and reports 'None' when it cannot
  // name a real topic. A 'None' leaves that viewer unmounted through the isTopic()
  // gate in renderImageViewer(); a viewer never falls back to a neighbor's stream,
  // so what each one shows is either the image that belongs to its source or
  // nothing at all.
  //
  // Every topic went through the parent's getDerivedTopic(), which adds the second
  // half of that guarantee: while a depth map change is still in flight it returns
  // 'None Available' for all three, so the bottom viewer cannot keep streaming the
  // previous depth map's targets image into the window between the operator's
  // selection and the node's next status message. Unmounting destroys the canvas
  // with the component, which is the only thing that actually clears it -- nothing
  // clears a canvas when a stream merely stops.
  renderImageViewers() {
    const depth_map_image_topic = this.props.depth_map_image_topic
    const image_topic = this.props.image_topic
    const targets_image_topic = this.props.targets_image_topic

    return (
      <React.Fragment>

        {this.renderImageViewer(depth_map_image_topic, "Depth Map Image (from selected Depth Map)")}

        {this.renderImageViewer(image_topic, "Color Image (from selected Depth Map)")}

        {this.renderImageViewer(targets_image_topic, "Targets Image (from derived Targets source)")}

      </React.Fragment>
    )
  }

  // The NavPose of the selected depth map, displayed directly below the image
  // viewers.
  //
  // Nepi_IF_NavPose is the RUI's reusable NavPose display -- the same component
  // Nepi_IF_ImageViewer, NepiDeviceNPX and NepiSystemNavPose mount, given a
  // namespace and left read-only. It subscribes to <ns>/status as a
  // NavPoseStatus and to <ns> itself as a NavPose, so the namespace it wants is
  // the navpose topic this app already derives; nothing is rendered here that
  // the component does not already render everywhere else.
  //
  // Mounted through the same getDerivedTopic() / isTopic() gate as the three
  // image viewers, and for the same reason: an unavailable source must leave the
  // viewer UNMOUNTED rather than mounted on a dead namespace. When there is no
  // topic the page renders the same read-only message the NavPose Source row in
  // NepiAppObstacles-Data.js already uses, so the two say the same thing.
  // key={topic} remounts on a change between two live topics rather than
  // repointing the subscriptions underneath a component still holding the previous
  // source's values.
  //
  // There is deliberately no selector here. The NavPose stays derived from the
  // selected depth map -- see the header comment and NepiAppObstaclesStatus.msg.
  renderNavPoseViewer() {
    const navpose_topic = this.props.navpose_topic

    if (this.isTopic(navpose_topic) === false) {
      return (
        <Columns>
          <Column>
            <Label title={"NavPose (from selected Depth Map)"} />
            <Label title={"No NavPose source for the selected Depth Map"} />
          </Column>
        </Columns>
      )
    }

    return (
      <Columns>
        <Column>
          <Label title={"NavPose (from selected Depth Map)"} />
          <NepiIFNavPose
            key={navpose_topic}
            navposeNamespace={navpose_topic}
            title={"NavPose Data"}
            show_line={false}
            read_only={true}
            make_section={false}
          />
        </Column>
      </Columns>
    )
  }

  render() {
    return (
      <React.Fragment>

        {this.renderImageViewers()}

        {this.renderNavPoseViewer()}

      </React.Fragment>
    )
  }
}

export default NepiAppObstaclesImages
