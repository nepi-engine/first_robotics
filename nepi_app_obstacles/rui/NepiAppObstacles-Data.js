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
import Styles from "./Styles"

import NepiIFConnectData from "./Nepi_IF_ConnectData"

@inject("ros")
@observer

// The CONNECTIONS panel of the Obstacles page: ONE source selector and two
// derived, read-only rows.
//
// Presentational and stateless. It performs NO derivation and holds NO
// subscription -- NepiAppObstacles.js is the sole owner of every ROS subscription
// on the page, including the three ConnectIFStatus subscriptions behind these
// rows, and hands the derived topics and the per-source connected booleans down as
// props. See the state-ownership comment at the top of NepiAppObstacles.js for why
// the derivation has exactly one owner; the short version is that this panel and
// the image viewers must agree, within a single render pass, about whether the
// current derivation is stale.
//
// Only the Depth Map row carries a Select. Targets and NavPose are derived from
// that depth map node-side and rendered read-only here, for the reason given at the
// top of NepiAppObstacles.js: they are properties of the depth map the operator
// picked, not independent choices, and a targets source belonging to a different
// camera than the depth map is not worth being able to express.
//
// Nepi_IF_ConnectTargets / Nepi_IF_ConnectNavPose are absent from this panel rather
// than mounted with show_selector={false}: both render their title and Connected
// indicator INSIDE renderSelector(), so with the selector off they render nothing
// at all. renderDerivedSource() below reproduces that header row from the parent's
// own ConnectIFStatus subscription on the same namespace.
//
// The node's two image connect IFs get no row -- they carry no selector and the
// node drives them; their viewers live in NepiAppObstacles-Images.js and read the
// app status message. The three source rows live inside ONE bordered Section: they
// are a single panel of source connections, so the box goes around the whole set,
// each row separated from the next by the standard RUI divider, and
// make_section={false} keeps Nepi_IF_ConnectData from drawing a bordered box of its
// own inside that panel. The process selector and process controls sit OUTSIDE this
// box, in NepiAppObstacles-Controls.js -- they are this app's own controls, not
// source connections.
class NepiAppObstaclesData extends Component {

  constructor(props) {
    super(props)

    this.isTopic = this.isTopic.bind(this)
    this.renderDerivedSource = this.renderDerivedSource.bind(this)
  }

  // True when a topic string handed down by the parent names a real topic. The
  // node reports 'None' for an unselected source, an un-set ROS string field
  // arrives as '', and the parent's getDerivedTopic() substitutes 'None Available'
  // while a derivation is stale -- which is what makes an unavailable source read
  // as unavailable here rather than blank. Same predicate the parent applies -- a
  // pure string test with no state, so a local copy cannot disagree with the
  // owner's; the derivation it tests is what stays owned in one place.
  isTopic(topic) {
    return (topic != null && topic !== '' && topic !== 'None' && topic !== 'None Available')
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
  //
  // The connected flag is NOT this component's to compute: ConnectIFStatus.connected
  // alone is not the answer, because the connect IF auto-selects an unrelated source
  // when the derivation is 'None' and reports itself connected to that one. The
  // parent's getDerivedConnected() resolves that against the topic actually being
  // displayed and passes the result in.
  renderDerivedSource(title, valueLabel, derivedTopic, connected, availableCaption, unavailableCaption) {
    const available = this.isTopic(derivedTopic)

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

  render() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    const targets_topic = this.props.targets_topic
    const navpose_topic = this.props.navpose_topic

    return (
      <Section title={"Connections"}>

        <NepiIFConnectData
          namespace={this.props.depthMapConnectNamespace}
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
          targets_topic,
          this.props.targets_connected,
          "Its targets image feeds the bottom viewer",
          "No targets source for the selected Depth Map"
        )}

        {divider}

        {this.renderDerivedSource(
          "NavPose",
          "NavPose Source",
          navpose_topic,
          this.props.navpose_connected,
          "From the selected Depth Map",
          "No NavPose source for the selected Depth Map"
        )}

      </Section>
    )
  }
}

export default NepiAppObstaclesData
