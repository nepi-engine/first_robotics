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
import Button from "./Button"
import Toggle from "react-toggle"
import Select, { Option } from "./Select"
import Styles from "./Styles"

import { setElementStyleModified, clearElementStyleModified } from "./Utilities"

import NepiIFConfig from "./Nepi_IF_Config"

@inject("ros")
@observer

// Template Application page
class NepiAppTemplate extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_template",
      appNamespace: null,

      // Status fields — mirror NepiAppTemplateStatus.msg
      enabled: false,
      selected_option: "None",
      options: ["None"],
      value: 0.0,

      // Local edit buffer for the value Input (committed on Enter)
      value_edit: "0.0",

      statusListener: null,
      connected: false,
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.onToggleEnabled = this.onToggleEnabled.bind(this)
    this.onSelectOption = this.onSelectOption.bind(this)
    this.onUpdateValueText = this.onUpdateValueText.bind(this)
    this.onKeyValueText = this.onKeyValueText.bind(this)
    this.onTriggerAction = this.onTriggerAction.bind(this)
    this.renderControls = this.renderControls.bind(this)
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

  statusListener(message) {
    this.setState({
      enabled: message.enabled,
      selected_option: message.selected_option,
      options: (message.options && message.options.length > 0) ? message.options : this.state.options,
      value: message.value,
      // Seed the edit buffer on first connect only, so typing isn't clobbered
      value_edit: this.state.connected ? this.state.value_edit : String(message.value),
      connected: true,
    })
  }

  updateStatusListener(namespace) {
    const statusNamespace = namespace + '/status'
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    var statusListener = this.props.ros.setupStatusListener(
      statusNamespace,
      "nepi_app_template/NepiAppTemplateStatus",
      this.statusListener
    )
    this.setState({
      appNamespace: namespace,
      statusListener: statusListener,
    })
  }

  componentDidMount() {
    const namespace = this.getAppNamespace()
    if (namespace !== null) {
      this.updateStatusListener(namespace)
    }
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1) {
        this.updateStatusListener(namespace)
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }

  onToggleEnabled() {
    const { sendBoolMsg } = this.props.ros
    sendBoolMsg(this.getAppNamespace() + '/set_enabled', !this.state.enabled)
  }

  onSelectOption(event) {
    const { sendStringMsg } = this.props.ros
    const topic = this.getAppNamespace() + '/set_option'
    sendStringMsg(topic, event.target.value)
  }

  // Editable Input pattern: typing marks the box modified (red + bold),
  // Enter commits the value and clears the modified style.
  onUpdateValueText(e) {
    const el = document.getElementById("AppTemplateValue")
    if (el) {
      setElementStyleModified(el)
    }
    this.setState({ value_edit: e.target.value })
  }

  onKeyValueText(e) {
    const { sendFloatMsg } = this.props.ros
    if (e.key === 'Enter') {
      const el = document.getElementById("AppTemplateValue")
      const val = parseFloat(el.value)
      if (!isNaN(val)) {
        clearElementStyleModified(el)
        sendFloatMsg(this.getAppNamespace() + '/set_value', val)
      }
    }
  }

  onTriggerAction() {
    const { sendTriggerMsg } = this.props.ros
    sendTriggerMsg(this.getAppNamespace() + '/trigger_action')
  }

  renderControls() {
    // Options come from the app's status message, so the RUI always matches the node
    const appNamespace = this.getAppNamespace()
    const { connected, enabled, selected_option, options, value_edit } = this.state

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Columns>
          <Column>
            <Label title={"Enabled"} />
          </Column>
          <Column>
            <Toggle
              checked={enabled}
              onClick={this.onToggleEnabled}
              disabled={!connected}
            />
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"Option"} />
          </Column>
          <Column>
            <Select
              onChange={this.onSelectOption}
              value={selected_option}
              disabled={!connected}
            >
              {options.map((opt) => (
                <Option key={opt} value={opt}>{opt}</Option>
              ))}
            </Select>
          </Column>
        </Columns>

        <Columns>
          <Column>
            <Label title={"Value"} />
          </Column>
          <Column>
            <Input
              id={"AppTemplateValue"}
              value={value_edit}
              onChange={this.onUpdateValueText}
              onKeyDown={this.onKeyValueText}
              disabled={!connected}
            />
          </Column>
        </Columns>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Columns>
          <Column>
            <Button
              style={{}}
              onClick={this.onTriggerAction}
              disabled={!connected || !enabled}
            >
              Trigger Action
            </Button>
          </Column>
        </Columns>

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

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    if (make_section === false) {
      return (
        <Columns>
          <Column>
            {this.renderControls()}
            {this.renderConfig()}
          </Column>
        </Columns>
      )
    } else {
      return (
        <Section>
          {this.renderControls()}
          {this.renderConfig()}
        </Section>
      )
    }
  }
}

export default NepiAppTemplate
