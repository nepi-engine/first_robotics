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

/*
 * APP-LOCAL COPY of the core RUI component Nepi_IF_Data.js.
 *
 * This file is owned by nepi_app_controls_sandbox. It exists so this app's data
 * rendering can be changed and extended without editing core nepi_rui. Edits
 * here affect this app only; the shared Nepi_IF_Data.js is untouched.
 *
 * Two rules keep it working:
 *   1. This file must stay listed under RUI_DICT.rui_files in
 *      params/controls_sandbox_app_params.yaml. That listing is the only place
 *      a RUI file is registered; build_nepi_rui.sh generates the rest.
 *   2. Its basename must stay unique across all app packages. Every app's rui/
 *      directory installs flat into the same nepi_rui src folder, so a
 *      duplicate basename from any app silently overwrites this file.
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Toggle from "react-toggle"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import BooleanIndicator from "./BooleanIndicator"

import { round, onChangeSwitchStateValue } from "./Utilities"

@inject("ros")
@observer

// Component that contains the DataIF read-only data display. Renders one row
// per datum from a nepi_interfaces/DataStatus message.
//
// This component is display only. It has no publishers, no editable inputs and
// no calls into any Store.js send function. The node that owns the DataIF is
// the only writer of record. The single Toggle below drives local component
// state ("Show Data") and never touches ROS.
class NepiAppControlsSandboxData extends Component {
  constructor(props) {
    super(props)

    this.state = {
      dataNamespace: null,
      status_msg: null,

      // "Show Data" toggle state (Nepi_IF_Controls pattern). Defaults shown;
      // can be overridden via the show_data prop or forced on via
      // allways_show_data.
      show_data: (this.props.show_data !== undefined) ? this.props.show_data : true,

      statusListener: null,
      needs_update: false
    }

    this.getNamespace = this.getNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.renderDatum = this.renderDatum.bind(this)
  }

  getNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null) {
      if (this.props.namespace !== undefined) {
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  // Nothing in this component is editable, so the incoming status is tracked
  // directly -- there is no in-progress edit to reconcile against it.
  statusListener(message) {
    this.setState({ status_msg: message })
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
        "nepi_interfaces/DataStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ dataNamespace: namespace, needs_update: false })
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getNamespace()
    if ((namespace != null && namespace !== this.state.dataNamespace) || this.state.needs_update === true) {
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

  // Render a single datum given its type and Datum message. Every branch below
  // is read-only: a disabled Input value box for numbers and strings, a
  // BooleanIndicator for bools. Array types render one box (or one indicator)
  // per element in a single row, matching the side-by-side pattern in
  // Nepi_IF_PTX-Data.js.
  renderDatum(name, type, datum_msg, index) {
    const display_name = (datum_msg.display_name && datum_msg.display_name !== '') ? datum_msg.display_name : name
    const description = datum_msg.description || ''

    // round_display is the number of decimals the RUI formats a Float/Floats
    // value to. Default 2 when the datum does not carry a sane value.
    const decimals = (typeof datum_msg.round_display === 'number' && datum_msg.round_display >= 0) ? datum_msg.round_display : 2

    // BOOL -- read-only indicator (green on, grey off). Never a Toggle.
    if (type === "Bool") {
      return (
        <Label title={display_name} key={name}>
          <BooleanIndicator title={description} value={(datum_msg.value_bool === true)} />
        </Label>
      )
    }

    // BOOLS -- one read-only indicator per element, in element order.
    if (type === "Bools") {
      const values = datum_msg.value_bools || []
      return (
        <Label title={display_name} key={name}>
          <div>
            {values.map((v, i) => (
              <div key={name + '_' + i} style={{ display: "inline-block", marginRight: Styles.vars.spacing.regular }}>
                <BooleanIndicator title={description} value={(v === true)} />
              </div>
            ))}
          </div>
        </Label>
      )
    }

    // STRING / INT / FLOAT -- a single read-only value box.
    if (type === "String" || type === "Int" || type === "Float") {
      var value = ''
      if (type === "String") { value = datum_msg.value_string }
      else if (type === "Int") { value = datum_msg.value_int }
      else { value = round(datum_msg.value_float, decimals) }
      return (
        <Label title={display_name} key={name}>
          <Input disabled title={description} style={{ width: "100%" }} value={value} />
        </Label>
      )
    }

    // STRINGS / INTS / FLOATS -- one read-only value box per element, in
    // element order, side by side in one row.
    if (type === "Strings" || type === "Ints" || type === "Floats") {
      var values = []
      if (type === "Strings") { values = datum_msg.value_strings || [] }
      else if (type === "Ints") { values = datum_msg.value_ints || [] }
      else { values = (datum_msg.value_floats || []).map((v) => round(v, decimals)) }
      const boxWidth = (values.length > 0) ? Math.floor(90 / values.length) + "%" : "90%"
      return (
        <Label title={display_name} key={name}>
          {values.map((v, i) => (
            <Input
              key={name + '_' + i}
              disabled
              title={description}
              style={{ width: boxWidth, float: "left" }}
              value={v}
            />
          ))}
        </Label>
      )
    }

    return null
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const status_msg = this.state.status_msg

    // Show Data toggle (Nepi_IF_Controls pattern). The data set is shown when
    // DataStatus.show_data is true; the toggle is only offered when the node
    // says it has one (DataStatus.has_show_control). allways_show_data forces
    // the data open and hides the toggle.
    const allways_show_data = (this.props.allways_show_data !== undefined) ? this.props.allways_show_data : false
    const status_show_data = (status_msg != null) ? (status_msg.show_data === true) : true
    const has_show_control = (status_msg != null) ? (status_msg.has_show_control === true) : false
    const show_data = (allways_show_data === true) ? true : (this.state.show_data && status_show_data)

    // show_visibility_toggle lets a parent that mounts this box inside a tab
    // suppress the "Show Data" toggle so it does not appear alongside the tab
    // bar (the mockup's Data tab has no such toggle).
    const show_visibility_toggle = (this.props.show_visibility_toggle !== undefined) ? this.props.show_visibility_toggle : true

    const show_data_toggle = (allways_show_data === false && has_show_control === true && show_visibility_toggle === true) ? (
      <Columns>
        <Column>
          <Label title="Show Data">
            <Toggle
              checked={show_data === true}
              onClick={() => onChangeSwitchStateValue.bind(this)("show_data", this.state.show_data)}>
            </Toggle>
          </Label>
        </Column>
        <Column>
        </Column>
      </Columns>
    ) : null

    // Data rows, one per non-hidden datum, in data_name_list order. Only built
    // when the section is expanded and a status has arrived.
    var data_body = null
    if (show_data === true && status_msg != null) {
      const names = status_msg.data_name_list || []
      const types = status_msg.data_type_list || []
      const msgs = status_msg.data_msg_list || []
      const hiddens = status_msg.data_hidden_list || []
      data_body = (
        <Columns>
          <Column>
            {names.map((name, i) => {
              const datum_msg = msgs[i]
              if (datum_msg == null) { return null }
              // Hidden data are not shown in the Data box.
              if (hiddens[i] === true || datum_msg.hidden === true) { return null }
              return this.renderDatum(name, types[i], datum_msg, i)
            })}
          </Column>
        </Columns>
      )
    }

    const body = (
      <React.Fragment>
        {show_data_toggle}
        {data_body}
      </React.Fragment>
    )

    if (make_section === false) {
      return body
    }
    return (
      <Section title={(this.props.title !== undefined) ? this.props.title : "DATA"}>
        {body}
      </Section>
    )
  }
}

export default NepiAppControlsSandboxData
