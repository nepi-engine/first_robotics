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

import Toggle from "react-toggle"
import Section from "./Section"
import { Columns, Column } from "./Columns"
import Select, { Option } from "./Select"
import { SliderAdjustment } from "./AdjustmentWidgets"
import BooleanIndicator from "./BooleanIndicator"
import Label from "./Label"
import Input from "./Input"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import RangeAdjustment from "./RangeAdjustment"

import NepiIFAdminEnable from "./Nepi_IF_AdminEnable"
import NepiIFAdminModes from "./Nepi_IF_AdminModes"

import {setElementStyleModified, clearElementStyleModified, onChangeSwitchStateValue, onChangeChangeStateValue, onUpdateSetStateValue, round} from "./Utilities"
import {createMenuBaseNames, createMenuFirstLastNames, createMenuListFromStrLists, removeStringFromMenuNames} from "./Utilities"



@inject("ros")
@observer

// Component that contains the PTX controls
class NepiAppPTAutoControls extends Component {
  constructor(props) {
    super(props)

    this.state = {

      appName: 'app_pan_tilt_auto',
      appNamespace: null,  
      status_msg: null,  
   
      show_control: 'None',

      selected_pan_tilt: 'None',
      panGoto: null,
      panDisabled: null,
      tiltGoto: 0,
      tiltDisabled: null,
      linkSpeeds: true,




      track_source_connected: false,
      track_reset_time: null,
      track_goal_deg: null,
      track_move_deg: null,
      tracking_topic: 'ai_track',
      trackPanMin: -50,
      trackPanMax: 50,
      trackTiltMin: -50,
      trackTiltMax: 50,
      trackResetTime: null,

      stab_show_settings: false,
      stab_update_rate: null,
      stab_num_avg: null,
      stab_control_names: [],
      stab_control_values: [],
      stab_reset_time_sec: null,

      auto_show_settings: false,
      auto_update_rate: null,
      auto_control_names: [],
      auto_control_values: [],

      autoPanMin: null,
      autoPanMax: null,
      autoTiltMin: null,
      autoTiltMax: null,


      lockPanMin: -50,
      lockPanMax: 50,
      lockTiltMin: -50,
      lockTiltMax: 50,


      scanEnabled: false,
      scanPanEnabled: false,
      scanTiltEnabled: false,

      trackPanReady: false,
      trackTiltReady: false,

      trackEnabled: false,
      trackPanEnabled: false,
      trackTiltEnabled: false,

      trackPanRunning: false,
      trackTiltRunning: false,

      stabPanReady: false,
      stabTiltReady: false,

      stabEnabled: false,
      stabPanEnabled: false,
      stabTiltEnabled: false,

      stabPanRunning: false,
      stabTiltRunning: false,

      click_pan_enabled: false,
      click_tilt_enabled: false,

      hide_click_controls: true,

      /*
      sinPanEnabled: false,
      #sinTiltEnabled: false,
      */

      speed_pan_dps: 0,
      speed_tilt_dps: 0,


      statusListener: null,         
      needs_update: false

    }


    this.renderPTAuto = this.renderPTAuto.bind(this)
    this.renderPTControls = this.renderPTControls.bind(this)

    this.renderScanControls = this.renderScanControls.bind(this)

    this.renderNavPoseControls = this.renderNavPoseControls.bind(this)

    this.onMenuSelection = this.onMenuSelection.bind(this)
    this.renderTrackControls = this.renderTrackControls.bind(this)

    this.renderAutoControls = this.renderAutoControls.bind(this)
    this.onUpdateInputControlNameValue = this.onUpdateInputControlNameValue.bind(this)
    this.onKeySaveInputControlNameValue = this.onKeySaveInputControlNameValue.bind(this)
    this.renderAutoControlValues = this.renderAutoControlValues.bind(this)
    this.onAutoUpdateText = this.onAutoUpdateText.bind(this)
    this.onAutoKeyText = this.onAutoKeyText.bind(this)

    this.onPTUpdateText = this.onPTUpdateText.bind(this)
    this.onPTKeyText = this.onPTKeyText.bind(this)
    this.renderPTAutoControls = this.renderPTAutoControls.bind(this)


    this.getNamespace = this.getNamespace.bind(this)
    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.statusListener = this.statusListener.bind(this)

  }

  getNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var namespace = null
    if (namespacePrefix != null && deviceId != null){
      if (this.props.namespace !== undefined){
        namespace = this.props.namespace
      }
    }
    return namespace
  }

  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId 
    }
    return baseNamespace
  }


  // Callback for handling ROS Status3DX messages
  statusListener(message) {
    const last_status_msg = this.state.status_msg
    this.setState({
      status_msg: message,
      selected_pan_tilt: message.selected_pan_tilt,

      scanPanEnabled: message.scan_pan_enabled,
      scanTiltEnabled: message.scan_tilt_enabled,

      track_source_connected: message.track_source_connected,

      trackPanEnabled: message.track_pan_enabled,
      trackTiltEnabled: message.track_tilt_enabled,
      
      stabPanReady: message.stab_ready,
      stabTiltReady: message.stab_ready,

      stabPanEnabled: message.stab_pan_enabled,
      stabTiltEnabled: message.stab_tilt_enabled,

      stabPanRunning: message.pan_stabbing,
      stabTiltRunning: message.tilt_stabbing,

      click_pan_enabled: message.click_pan_enabled,
      click_tilt_enabled: message.click_tilt_enabled,

      
    })
  
    var auto_controls_changed = true
   if (last_status_msg != null) {
      auto_controls_changed = JSON.stringify(message.auto_control_values) !== JSON.stringify(last_status_msg.auto_control_values)
   }

    if (auto_controls_changed === true){
      this.setState({auto_control_names: message.auto_control_names,
                    auto_control_values: message.auto_control_values
      })
    }


  }




  // Function for configuring and subscribing to Status
  updateStatusListener(namespace) {
    
    if (this.state.statusListener != null) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener: null, status_msg: null})
    }
    if (namespace != null && namespace !== 'None' && namespace.indexOf('null') === -1){
        const statusNamespace = namespace + '/status'
        var statusListener = this.props.ros.setupStatusListener(
              statusNamespace,
              "nepi_app_pan_tilt_auto/PanTiltAutoAppStatus",
              this.statusListener
            )
    this.setState({ 
      statusListener: statusListener,
    })
    }
    this.setState({ 
      appNamespace: namespace,
      needs_update: false
    })
  }
  
// Lifecycle method called when compnent updates.
// Used to track changes in the topic
componentDidUpdate(prevProps, prevState, snapshot) {
  const namespace = this.getNamespace()
  if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true){
      this.updateStatusListener(namespace)
  }
}

  componentDidMount() {
    this.setState({ needs_update: true })
    }
  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status3DX message


componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({statusListener : null})
    }
  }






  onPTUpdateText(e) {
    var panElement = null
    var tiltElement = null
    if (e.target.id === "PTXPanGoto") 
      {
        panElement = document.getElementById("PTXPanGoto")
        setElementStyleModified(panElement)
        this.setState({panGoto: e.target.value})
             
      }
        
    else if  (e.target.id === "PTXTiltGoto")
        {
          tiltElement = document.getElementById("PTXTiltGoto")
          setElementStyleModified(tiltElement)
          this.setState({tiltGoto: e.target.value})         
          
        }

  }

  onPTKeyText(e) {
    const {ptxDevices, onSetPTXGotoPos, onSetPTXGotoPanPos, onSetPTXGotoTiltPos, onSetPTXHomePos, onSetPTXSoftStopPos} = this.props.ros
 const {sendFloatMsg} = this.props.ros
const namespace = this.getNamespace()
    const selected_pan_tilt = this.state.selected_pan_tilt
    const ptx_caps = ptxDevices[selected_pan_tilt]
    const has_timed_pos = ptx_caps && (ptx_caps.has_timed_positioning)
    const has_sep_pan_tilt = ptx_caps && (ptx_caps.has_seperate_pan_tilt_control)
    const has_abs_pos = ptx_caps && (ptx_caps.has_absolute_positioning === true)
    const has_homing = ptx_caps && (ptx_caps.has_homing)
    const has_speed_control = ptx_caps && (ptx_caps.has_adjustable_speed)
    const has_sep_speed = ptx_caps && (ptx_caps.has_seperate_pan_tilt_speed === true)
    //Unused const has_set_home = ptx_caps && (ptx_caps.has_set_home)

    var panElement = null
    var tiltElement = null
    if(e.key === 'Enter'){
      if (e.target.id === "PTXPanGoto") 
        {
          panElement = document.getElementById("PTXPanGoto")
          tiltElement = document.getElementById("PTXTiltGoto")                    
          sendFloatMsg(namespace + '/set_pan_pos_deg', Number(panElement.value),Number(tiltElement.value))   
          clearElementStyleModified(panElement)   
          this.setState({panGoto: null})    
          
        }
        else if  (e.target.id === "PTXTiltGoto")
          {
            
            panElement = document.getElementById("PTXPanGoto")
            tiltElement = document.getElementById("PTXTiltGoto")


              sendFloatMsg(namespace + '/set_tilt_pos_deg', Number(tiltElement.value))
          
            clearElementStyleModified(tiltElement)
            this.setState({tiltGoto: null})      
          
          }
    }
  }




  renderPTAuto() {
    const { ptxDevices, sendBoolMsg } = this.props.ros

    const { scanPanEnabled, scanTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg
    const topics = Object.keys(ptxDevices)
    const pt_connected_topics = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_pan_tilt)){
      pt_connected_topics.push(topics[i])
    }
  }
    
    const pt_connected = (pt_connected_topics.indexOf(selected_pan_tilt) !== -1)
    //console.log('pt_connected: ' + pt_connected)


    if (status_msg == null || pt_connected == false){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {


    const has_scan_pan = (status_msg.pt_status_msg.has_scan_pan)
    const has_scan_tilt = (status_msg.pt_status_msg.has_scan_tilt)
    const has_abs_pos = (status_msg.pt_status_msg.has_absolute_positioning)
    const has_homing = (status_msg.pt_status_msg.has_homing)
    const has_speed_control = (status_msg.pt_status_msg.has_adjustable_speed)
    const has_sep_speed = (status_msg.pt_status_msg.has_seperate_pan_tilt_speed)

    const disable_track_enable = ((track_source_connected === false || has_scan_pan === false || has_scan_tilt === false))

    const disable_stab_enable = false


      const scanEnabled = status_msg.scan_enabled
      const trackEnabled = status_msg.track_enabled
      const stabEnabled = status_msg.stab_enabled

      const pan_control_disabled = (status_msg.pan_control_disabled === true)
      const tilt_control_disabled = (status_msg.tilt_control_disabled === true)
      const speedRatio = status_msg.speed_ratio
      const speedPanRatio = status_msg.pan_speed_ratio
      const speedTiltRatio = status_msg.tilt_speed_ratio


      const show_control = this.state.show_control
        return (
          <React.Fragment>



          {/* { (has_homing === false) ?


          <ButtonMenu>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pt_stop')}>{"STOP"}</Button>
          </ButtonMenu>

          :

          <ButtonMenu>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pt_stop')}>{"STOP"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pan_home')}>{"PAN HOME"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/tilt_home')}>{"TILT HOME"}</Button>
          </ButtonMenu>

          } */}




              <Label title={"Stabilize"}>


                <div style={{ display: "inline-block", width: "45%", float: "left" }} >

                      <Toggle style={{justifyContent: "flex-left"}} 
                        disabled={this.state.stabPanReady === false}
                        checked={stabEnabled === true} 
                        onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_enable",!stabEnabled)} />
                  
                </div>
{/* 
                <div style={{ display: "inline-block", width: "45%", float: "left" }} >
                  <div hidden={this.state.stabPanReady === false}>
                      <Toggle style={{justifyContent: "flex-left"}} 
                        checked={stabPanEnabled === true} 
                        onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_pan_enable",!stabPanEnabled)} />
                  </div>
                </div>


                <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                  <div hidden={this.state.stabTiltReady === false}>
                    <Toggle style={{justifyContent: "flex-right"}} 
                      checked={stabTiltEnabled === true} 
                      onClick={() => sendBoolMsg.bind(this)(namespace + "/set_stab_tilt_enable",!stabTiltEnabled)} />
                  </div>
                </div> */}

              </Label>


            <Label title={"Sweep"}>

              <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} 
                  checked={scanEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_enable",!scanEnabled)} />
              </div>

              {/* <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                <Toggle style={{justifyContent: "flex-left"}} 
                  checked={scanPanEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_pan_enable",!scanPanEnabled)} />
              </div>


              <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                <Toggle style={{justifyContent: "flex-right"}} 
                  checked={scanTiltEnabled} 
                  onClick={() => sendBoolMsg.bind(this)(namespace + "/set_scan_tilt_enable",!scanTiltEnabled)} />
              </div> */}

            </Label>


              <Label title={"AI Tracking"}>


                <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                  <Toggle style={{justifyContent: "flex-left"}} 
                    disabled={disable_track_enable === true}
                    checked={trackEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_enable",!trackEnabled)} />
                </div>

                {/* <div style={{ display: "inline-block", width: "45%", float: "left" }}>
                  <Toggle style={{justifyContent: "flex-left"}} 
                    disabled={disable_track_enable === true}
                    checked={trackPanEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_pan_enable",!trackPanEnabled)} />
                </div>


                <div style={{ display: "inline-block", width: "45%", float: "right" }}>
                  <Toggle style={{justifyContent: "flex-right"}} 
                    disabled={disable_track_enable === true}
                    checked={trackTiltEnabled === true && disable_track_enable === false} 
                    onClick={() => sendBoolMsg.bind(this)(namespace + "/set_track_tilt_enable",!trackTiltEnabled)} />
                </div> */}

              </Label>



            {this.renderPTControls()}

            </React.Fragment>
        )
  }
}




  onClickToggleShowSettings(){
    const currentVal = this.state.showSettings 
    this.setState({showSettings: !currentVal})
    this.render()
  }




  renderPTAutoControls() {
    const { ptxDevices, sendBoolMsg } = this.props.ros
    const admin_mode_set = this.props.ros.systemAdminModeSet
    const { scanPanEnabled, scanTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg


    if (status_msg == null || namespace == null){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {
 


        const show_control = this.state.show_control
        return (
          <React.Fragment>
   

          { this.renderPTAuto() }
        
          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>



          <Label title={"Settings"}></Label>

                  <div style={{ display: 'flex' }} >
                    <div style={{ display: "inline-block", width: "20%"}}>{"Track"}</div>
                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>
                    <div style={{ display: "inline-block", width: "20%"}}>{"Auto"}</div>
                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>
                    <div style={{ display: "inline-block", width: "20%" }}>{}</div>
                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>
                    <div style={{ display: "inline-block", width: "20%" }}>{"Admin"}</div>
                  </div>

                  <div style={{ display: 'flex' }} >

                  <div style={{ display: "inline-block", width: "20%", float: "left" }}>

                        <Toggle
                        checked={(show_control === 'track')}
                        onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'track') ? 'None' : 'track' )}>
                        </Toggle>
                  </div>

                  <div style={{ display: "inline-block", width: "5%"}}>{}</div>


                    <div style={{ display: "inline-block", width: "20%", float: "left" }}>
                        <Toggle
                          checked={(show_control === 'auto')}
                          onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'auto') ? 'None' : 'auto' )}>
                        </Toggle>


                    </div>

                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>

                    <div style={{ display: "inline-block", width: "20%", float: "center" }}>
                      {}
                      {/* <Toggle
                      disabled={admin_mode_set === false}
                      checked={(show_control === 'scan')}
                      onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'scan') ? 'None' : 'scan' )}>
                      </Toggle> */}
                    </div>

                    <div style={{ display: "inline-block", width: "5%"}}>{}</div>

                    <div style={{ display: "inline-block", width: "20%", float: "right" }}>
                     <Toggle
                      checked={(show_control === 'admin')}
                      onClick={() => onChangeChangeStateValue.bind(this)("show_control",(show_control === 'admin') ? 'None' : 'admin' )}>
                      </Toggle>
                    </div>

              </div>


            <div hidden={(show_control !== 'track')}>
                  {this.renderTrackControls()}
            </div>

          {/* <div hidden={(show_control !== 'scan')}>
                {this.renderScanControls()}
          </div> */}


            <div hidden={(show_control !== 'auto')}>
                  {this.renderAutoControls()}
            </div>

      { ( show_control === 'admin' ) ?
      <NepiIFAdminEnable
        make_section={false}
        title={null}
        show_link_button={false}
        show_line={false}
        />
        : null}


        {(admin_mode_set === true  && show_control === 'admin' ) ? 
        
          <NepiIFAdminModes
          make_section={false}
          />
        : null} 


            </React.Fragment>
        )
  }
}




  renderPTControls() {
    const { ptxDevices, sendBoolMsg } = this.props.ros

    const { scanPanEnabled, scanTiltEnabled, trackPanEnabled, trackTiltEnabled,
            track_source_connected, stabPanEnabled, stabTiltEnabled,
            click_pan_enabled, click_tilt_enabled  } = this.state /*sinPanEnabled ,sinTiltEnabled*/

    const selected_pan_tilt = this.state.selected_pan_tilt

    //Unused const {sendTriggerMsg} = this.props.ros

    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg
    const topics = Object.keys(ptxDevices)
    const pt_connected_topics = []
    var i
    for (i = 0; i <topics.length; i++) {
    if (topics[i].includes(selected_pan_tilt)){
      pt_connected_topics.push(topics[i])
    }
  }
    
    const pt_connected = (pt_connected_topics.indexOf(selected_pan_tilt) !== -1)
    //console.log('pt_connected: ' + pt_connected)


    if (status_msg == null || pt_connected == false){
      return(

        <Columns>
        <Column>

        </Column>
        </Columns>

      )

    }
    else {


    const has_scan_pan = (status_msg.pt_status_msg.has_scan_pan)
    const has_scan_tilt = (status_msg.pt_status_msg.has_scan_tilt)
    const has_abs_pos = (status_msg.pt_status_msg.has_absolute_positioning)
    const has_homing = (status_msg.pt_status_msg.has_homing)
    const has_speed_control = (status_msg.pt_status_msg.has_adjustable_speed)
    const has_sep_speed = (status_msg.pt_status_msg.has_seperate_pan_tilt_speed)

    const disable_track_enable = ((track_source_connected === false || has_scan_pan === false || has_scan_tilt === false))

    const disable_stab_enable = false

      const panPosition = status_msg.pt_status_msg.pan_now_deg
      const tiltPosition = status_msg.pt_status_msg.tilt_now_deg

      const panPositionClean = panPosition + .001
      const tiltPositionClean = tiltPosition + .001

      if (this.state.panGoto == null){
        this.setState({panGoto: panPositionClean})
      }

      if (this.state.tiltGoto == null){
        this.setState({tiltGoto: tiltPositionClean})
      }

      const panMove = status_msg.pt_status_msg.pan_goal_deg
      const tiltMove = status_msg.pt_status_msg.tilt_goal_deg

      const panMoveClean = panMove + .001
      const tiltMoveClean = tiltMove + .001

      const pan_control_disabled = (status_msg.auto_pan_pos_disabled === true)
      
      if (pan_control_disabled !== this.state.panDisabled){
        this.setState({panGoto: panMoveClean, panDisabled: pan_control_disabled})
      }
      const pan_pos = this.state.panGoto //pan_control_disabled === true ? panMoveClean : this.state.panGoto

      const tilt_control_disabled = (status_msg.auto_tilt_pos_disabled === true)
      if (tilt_control_disabled !== this.state.tiltDisabled){
        this.setState({tiltGoto: tiltMoveClean, tiltDisabled: tilt_control_disabled})
      }      
      const tilt_pos = this.state.tiltGoto //tilt_control_disabled === true ? tiltMoveClean : this.state.tiltGoto


      const panCurSpeed = status_msg.pt_status_msg.speed_pan_dps
      const tiltCurSpeed = status_msg.pt_status_msg.speed_tilt_dps

      const panCurSpeedClean = panCurSpeed + .001
      const tiltCurSpeedClean = tiltCurSpeed + .001

      const speedRatio = status_msg.speed_ratio
      const speedPanRatio = status_msg.auto_pan_speed_ratio_set
      const speedTiltRatio = status_msg.auto_tilt_speed_ratio_set

      const maxSpeed = status_msg.pan_tilt_max_speed_dps
      const panSetSpeed = speedPanRatio * maxSpeed
      const tiltSetSpeed = speedTiltRatio * maxSpeed

      const panSetSpeedClean = panSetSpeed + .001
      const tiltSetSpeedClean = tiltSetSpeed + .001

        return (
          <React.Fragment>

          <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label>

          <ButtonMenu>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pt_stop')}>{"STOP"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/pan_home')}>{"HOME"}</Button>
            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + '/tilt_home')}>{"HOME"}</Button>
          </ButtonMenu>

          <div hidden={(has_abs_pos === false)}>

{/* 
              <Label title={"GoTo Position "}>
                <Input
                  disabled={pan_control_disabled === true}
                  id={"PTXPanGoto"}
                  style={{ width: "45%", float: "left" }}
                  value={round(pan_pos,1)}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
                <Input
                  disabled={tilt_control_disabled === true}
                  id={"PTXTiltGoto"}
                  style={{ width: "45%" }}
                  value={round(tilt_pos,1)}
                  onChange= {this.onPTUpdateText}
                  onKeyDown= {this.onPTKeyText}
                />
              </Label> */}


              <Label title={"Current Position"}>
                <Input
                  disabled={true}
                  style={{ width: "45%", float: "left" }}
                  value={round(panPositionClean, 1)}
                />
                <Input
                  disabled={true}
                  style={{ width: "45%" }}
                  value={round(tiltPositionClean, 1)}
                />
              </Label>

              <Label title={"Average Speed"}>
                <Input
                  disabled={true}
                  style={{ width: "45%", float: "left" }}
                  value={round(panCurSpeedClean, 1)}
                />
                <Input
                  disabled={true}
                  style={{ width: "45%" }}
                  value={round(tiltCurSpeedClean, 1)}
                />
              </Label>

  
          </div>



          <div hidden={(has_speed_control === false)}>

              <React.Fragment>
                <SliderAdjustment
                  disabled={status_msg.auto_pan_position_disabled}
                  title={"Pan Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedPanRatio}
                  topic={namespace + "/set_pan_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  displayValue={round(panSetSpeedClean,1)}
                  unit={""}
                />
                <SliderAdjustment
                  disabled={status_msg.auto_tilt_position_disabled}
                  title={"Tilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={speedTiltRatio}
                  topic={namespace + "/set_tilt_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  displayValue={round(tiltSetSpeedClean,1)}
                  unit={""}
                />
              </React.Fragment>
  
          </div>


            </React.Fragment>
        )
  }
}



onEnterSendPanScanRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appnamespace = this.getNamespace()
  const topic_namespace = appnamespace + '/' + topicName
  var min = 0
  var max = 0
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (entryName === "max"){
      if (value < this.state.autoPanMin && !isNaN(value)){
        console.log("invaled range")

        const cur_max = this.state.status_msg.scan_pan_max_deg
        this.setState({autoPanMax: cur_max })
      }
      else{
        min = other_val
        max = value
        publishRangeWindow(topic_namespace,min,max,false)
      }
    }
    else if (entryName === "min") {
      if (value > this.state.autoPanMax && !isNaN(value)){
        console.log("invaled range")

        const cur_min = this.state.status_msg.scan_pan_min_deg
        this.setState({autoPanMin: cur_min })
      }
      else {
        min = value
        max = other_val
        publishRangeWindow(topic_namespace,min,max,false)
    }
    }
  
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }

}

onEnterSendTiltScanRangeWindowValue(event, topicName, entryName, other_val) {
  const {publishRangeWindow} = this.props.ros
  const appnamespace = this.getNamespace()
  const topic_namespace = appnamespace + '/' + topicName
  var min = 0
  var max = 0
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (entryName === "max"){
      if (value < this.state.autoTiltMin && !isNaN(value)){
        console.log("invaled range")

        const cur_max = this.state.status_msg.scan_tilt_max_deg
        this.setState({autoTiltMax: cur_max })
      }
      else{
        min = other_val
        max = value
        publishRangeWindow(topic_namespace,min,max,false)
      }
    }
    else if (entryName === "min") {
      if (value > this.state.autoTiltMax && !isNaN(value)){
        console.log("invaled range")

        const cur_min = this.state.status_msg.scan_tilt_min_deg
        this.setState({autoTiltMin: cur_min })
      }
      else {
        min = value
        max = other_val
        publishRangeWindow(topic_namespace,min,max,false)
    }
    }
  
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }

}

  onMenuSelection(event){
    const {sendStringMsg} = this.props.ros

    const value = event.target.value
    const topic = event.target.id
    const namespace = this.getNamespace()
    const tracking_topic = this.state.tracking_topic
    const sendNamespace = namespace + '/' + tracking_topic + '/' +  topic
    sendStringMsg(sendNamespace,value)
  }



  renderTrackControls() {

    const admin_mode_set = this.props.ros.systemAdminModeSet
    const namespace = this.getNamespace()
    const track_namespace = namespace + '/' + this.state.tracking_topic

    const status_msg = this.state.status_msg
    

    
    const available_sources = status_msg.available_track_source_namespaces
    const has_sources = available_sources.length > 0
    const sources_names = createMenuFirstLastNames(available_sources)
    const sources_menu = createMenuListFromStrLists(available_sources,sources_names, ['None'], [],'None Available')
    const selected_source = status_msg.track_source_selected
    const source_connected = status_msg.track_source_connected
    const selected_image = status_msg.track_image_topic


    const threshold_filter = status_msg.track_threshold

    const available_best = status_msg.track_best_filter_options
    const best_menu = createMenuListFromStrLists(available_best,available_best, [], [], '')
    const best_filter = status_msg.track_best_filter


    const goal_deg = status_msg.track_goal_deg
    var track_goal_deg = this.state.track_goal_deg
    if (track_goal_deg == null){
      track_goal_deg = goal_deg
    }

    const move_deg = status_msg.track_move_deg
    var track_move_deg = this.state.track_move_deg
    if (track_move_deg == null){
      track_move_deg = move_deg
    }


    const move_ratio = status_msg.track_move_ratio
    const reset_time = status_msg.track_reset_time_sec
    var track_reset_time = this.state.track_reset_time
    if (track_reset_time == null){
      track_reset_time = reset_time
    }
    const track_pan_error = status_msg.track_pan_error
    const track_tilt_error = status_msg.track_tilt_error



        return (
          <React.Fragment>

    <div hidden={has_sources === false}>


            <Label title={'Track Source'}>
              <Select
                id="set_targets_topic"
                onChange={this.onMenuSelection}
                value={selected_source}
              >
                {sources_menu}
              </Select>
            </Label>


         

              <Label title={'Track Image'}>
                <Input
                  disabled={true}
                  value={selected_image}
                />
              </Label>



                  <Label title={'Track Filter'}>
                      <Select
                        disabled={admin_mode_set === false}
                        id="set_best_filter"
                        onChange={this.onMenuSelection}
                        value={best_filter}
                      >
                        {best_menu}
                      </Select>
                    </Label>



  

               

            <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                  <SliderAdjustment
                            title={"Track Sensitivity"}
                            msgType={"std_msgs/Float32"}
                            adjustment={threshold_filter}
                            topic={track_namespace + "/set_threshold_filter"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Sets target confidence threshold filter"}
                            unit={"%"}
                    />




 </div>



        <Columns>
            <Column>

                      <div hidden={has_sources === false}>
                        <ButtonMenu>
                          <Button onClick={() => window.open("/ai_detectors_mgr", "_blank")}>{"Open Detectors"}</Button>
                        </ButtonMenu>

                      </div>
            </Column>

            <Column>
                    <ButtonMenu>
                      <Button onClick={() => window.open("/ai_models_mgr", "_blank")}>{"Open Models"}</Button>
                    </ButtonMenu>          
            </Column>
        </Columns>




            </React.Fragment>
        )
  
}


  renderScanControls() {


    const namespace = this.getNamespace()

    const status_msg = this.state.status_msg

   

        return (
          <React.Fragment>


 <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
{/* 
          <label style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
                {"PT Frame - Angles in ENU frame (Tilt+:Down , Pan+:Left)"}
              </label> */}



              {/* <React.Fragment>
                <SliderAdjustment
                  disabled={false}
                  title={"Pan Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={status_msg.scan_pan_speed_ratio}
                  topic={namespace + "/set_scan_pan_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
                <SliderAdjustment
                  disabled={false}
                  title={"Tilt Speed"}
                  msgType={"std_msgs/Float32"}
                  adjustment={status_msg.scan_tilt_speed_ratio}
                  topic={namespace + "/set_scan_tilt_speed_ratio"}
                  scaled={0.01}
                  min={0}
                  max={100}
                  tooltip={"Speed as a percentage (0%=min, 100%=max)"}
                  unit={"%"}
                />
              </React.Fragment> */}
  



            <Label title={""}>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
              <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
            </Label>

            <Label title={"Min Scan Limit"}>

              <Input id="scan_pan_min" 
                  value={this.state.autoPanMin} 
                  style={{ width: "45%", float: "left" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"autoPanMin")} 
                  onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_scan_pan_window","min",Number(this.state.autoPanMax))} />

              <Input id="scan_tilt_min" 
                  value={this.state.autoTiltMin} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"autoTiltMin")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_scan_tilt_window","min",Number(this.state.autoTiltMax))} />

              
            </Label>


            <Label title={"Max Scan Limit"}>

              <Input id="scan_pan_max" 
                value={this.state.autoPanMax} 
                style={{ width: "45%", float: "left" }}
                onChange={(event) => onUpdateSetStateValue.bind(this)(event,"autoPanMax")} 
                onKeyDown= {(event) => this.onEnterSendPanScanRangeWindowValue(event,"/set_scan_pan_window","max",Number(this.state.autoPanMin))} />     


              <Input id="scan_tilt_max" 
                  value={this.state.autoTiltMax} 
                  style={{ width: "45%" }}
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"autoTiltMax")} 
                  onKeyDown= {(event) => this.onEnterSendTiltScanRangeWindowValue(event,"/set_scan_tilt_window","max",Number(this.state.autoTiltMin))} />                      
            </Label>





            </React.Fragment>
        )
  
}


  renderNavPoseControls() {
    const namespace = this.getNamespace()
    const baseNamespace = this.getBaseNamespace()
    const status_msg = this.state.status_msg

    const available_sources = status_msg.available_stab_source_namespaces
    var sources_names = available_sources
    sources_names = removeStringFromMenuNames(sources_names,baseNamespace + '/')
    sources_names = removeStringFromMenuNames(sources_names,'navposes/')
    sources_names = removeStringFromMenuNames(sources_names,'navpose/')
    sources_names = removeStringFromMenuNames(sources_names,'npx/')
    const sources_menu = createMenuListFromStrLists(available_sources, sources_names, ['None'], [], 'None Available')
    const selected_source = status_msg.selected_stab_source

    const source_connected = status_msg.stab_source_connected

    return (
      <React.Fragment>

<div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

     

        <Label title={""}>
          <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Ready"}</div>
          <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Running"}</div>
        </Label>


        <Label title={"Pan"}>
          <div style={{ width: "45%", float: "left" }}>
          <BooleanIndicator value={this.state.stabPanReady} />
          </div>
          <div style={{ width: "45%", float: "left" }}>
           <BooleanIndicator value={this.state.stabPanRunning} />
           </div>
        </Label>

        <Label title={"Tilt"}>
          <div style={{ width: "45%", float: "left" }}>
          <BooleanIndicator value={this.state.stabTiltReady}  />
          </div>
          <div style={{ width: "45%", float: "left" }}>
           <BooleanIndicator value={this.state.stabTiltRunning} />
           </div>
        </Label>


          <Label title={'NavPose Source'}>
          <Select
            id="set_stab_source"
            onChange={(e) => this.props.ros.sendStringMsg(namespace + "/set_stab_source", e.target.value)}
            value={selected_source}
          >
            {sources_menu}
          </Select>
        </Label>

        <Label title={"Source Connected"}>
          <BooleanIndicator value={source_connected} />
        </Label>


      </React.Fragment>
    )
  }





  onUpdateInputControlNameValue(event, name, index) {
    const value = event.target.value
    var auto_control_values = this.state.auto_control_values
    auto_control_values[index] = value
    this.setState({ auto_control_values: auto_control_values })
    document.getElementById(name).style.color = Styles.vars.colors.red
    //this.render()
  }

  onKeySaveInputControlNameValue(event, name, index) {
    const namespace = this.getNamespace() + '/set_auto_control_value'

    if(event.key === 'Enter'){
      const value = event.target.value
      const parsed = parseFloat(value)
      if (!Number.isNaN(parsed)) {
        this.props.ros.sendUpdateFloatMsg(namespace,name,parsed)
      }
      else {
        const auto_control_values = this.state.status_msg.auto_control_values
        this.setState({ auto_control_values: auto_control_values })
      }
      document.getElementById(name).style.color = Styles.vars.colors.black
    }
  }


  onAutoUpdateText(e) {
    const id = e.target.id
    const stateKey = {
      AutoUpdateRate: 'auto_update_rate'
    }[id]
    if (stateKey) {
      const element = document.getElementById(id)
      setElementStyleModified(element)
      this.setState({[stateKey]: e.target.value})
    }
  }

  onAutoKeyText(e) {
    const {sendFloatMsg} = this.props.ros
    const namespace = this.getNamespace()
    if (e.key === 'Enter') {
      const val = parseFloat(e.target.value)
      if (e.target.id === "AutoUpdateRate") {
        clearElementStyleModified(document.getElementById("AutoUpdateRate"))
        if (!isNaN(val)) { sendFloatMsg(namespace + "/set_auto_update_rate", val) }
        this.setState({auto_update_rate: null})
      }
    }
  }



    renderStabControlValues(name, value, index) {
    return (

      <React.Fragment>

        <Label title={name}>
          <Input id={name}
              style={{ width: "45%", float: "left" }}
              value={value}
              onChange={(event) => this.onUpdateInputControlNameValue(event,name,index)}
              onKeyDown= {(event) => this.onKeySaveInputControlNameValue(event,name,index)} />
        </Label>
      </React.Fragment>
    )
  }

  renderAutoControlValues(name, value, index) {
    return (
      <React.Fragment>
        <Label title={name}>
          <Input id={name}
              style={{ width: "45%", float: "left" }}
              value={value}
              onChange={(event) => this.onUpdateInputControlNameValue(event,name,index)}
              onKeyDown= {(event) => this.onKeySaveInputControlNameValue(event,name,index)} />
        </Label>
      </React.Fragment>
    )
  }




  renderAutoControls() {
    const { sendBoolMsg } = this.props.ros
    const namespace = this.getNamespace()
    const status_msg = this.state.status_msg
    const admin_mode_set = this.props.ros.systemAdminModeSet

    const available_processes = status_msg.available_auto_processes
    const processes_menu = createMenuListFromStrLists(available_processes, available_processes, ['None'], [], 'None Available')
    const selected_process = status_msg.selected_auto_process

    var auto_update_rate = this.state.auto_update_rate
    if (auto_update_rate == null) { auto_update_rate = status_msg.auto_update_rate }

    const show_settings = this.state.auto_show_settings
    const auto_control_names = this.state.auto_control_names
    const auto_control_values = this.state.auto_control_values

    const pan_deg = status_msg.pan_deg
    const tilt_deg = status_msg.tilt_deg
    const pan_deg_per_sec = status_msg.pan_deg_per_sec
    const tilt_deg_per_sec = status_msg.tilt_deg_per_sec

    const auto_pan_pos = status_msg.auto_pan_pos
    const auto_tilt_pos = status_msg.auto_tilt_pos

    const roll_deg = status_msg.roll_deg
    const pitch_deg = status_msg.pitch_deg
    const heading_deg = status_msg.heading_deg

    const auto_pan_adj = status_msg.auto_pan_adj
    const auto_tilt_adj = status_msg.auto_tilt_adj
    const auto_pan_goal = status_msg.auto_pan_goal
    const auto_pan_dps = status_msg.auto_pan_dps
    const auto_tilt_goal = status_msg.auto_tilt_goal
    const auto_tilt_dps = status_msg.auto_tilt_dps

    const auto_pan_pos_rate = status_msg.auto_pan_pos_rate
    const auto_pan_vel_rate = status_msg.auto_pan_vel_rate
    const auto_tilt_pos_rate = status_msg.auto_tilt_pos_rate
    const auto_tilt_vel_rate = status_msg.auto_tilt_vel_rate


    const dual_mode_supported = (this.state.status_msg != null) ? this.state.status_msg.has_dual_mode : false
    const night_mode_supported = (this.state.status_msg != null) ? this.state.status_msg.has_night_mode : false
    const zoom_mode_supported = (this.state.status_msg != null) ? this.state.status_msg.has_zoom_mode : false
    const auto_night_enabled = status_msg.auto_night_enabled

    const auto_lat = status_msg.auto_lat
    const auto_long = status_msg.auto_long
    const is_night = status_msg.is_night

    return (
      <React.Fragment>

<div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

          {/* <Label title={""} style={{fontWeight: 'bold'}} align={"left"} textAlign={"left"}>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pan"}</div>
            <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Tilt"}</div>
          </Label> */}

           <Label title={"Auto PT Pos"}>
            <Input disabled style={{ width: "45%", float: "left" }} value={round(auto_pan_pos, 2)} />
            <Input disabled style={{ width: "45%" }} value={round(auto_tilt_pos, 2)} />
          </Label>

           <Label title={"Auto Lat Long"}>
            <Input disabled style={{ width: "45%", float: "left" }} value={round(auto_lat, 8)} />
            <Input disabled style={{ width: "45%" }} value={round(auto_long, 8)} />
          </Label>


        <Label title={"Is Night"}>
          <BooleanIndicator value={is_night === true} />
        </Label>

        <Label title={'Enable Auto Night'}>
          <Toggle style={{justifyContent: "flex-right"}} 
            checked={auto_night_enabled === true} 
            onClick={() => sendBoolMsg.bind(this)(namespace + "/set_auto_night_enable",!auto_night_enabled)} />
        </Label>


        <div hidden={admin_mode_set === false}>


                    {this.renderNavPoseControls()}

                    <Label title={'Has Dual Mode'}>
                      <Toggle style={{justifyContent: "flex-right"}} 
                          checked={dual_mode_supported === true} 
                          onClick={() => sendBoolMsg.bind(this)(namespace + "/set_has_dual_mode",!dual_mode_supported)} />
                    </Label>

                    <Label title={'Has Night Mode'}>
                      <Toggle style={{justifyContent: "flex-right"}} 
                          checked={night_mode_supported === true} 
                          onClick={() => sendBoolMsg.bind(this)(namespace + "/set_has_night_mode",!night_mode_supported)} />
                    </Label>



                    <Label title={'Has Zoom Mode'}>
                      <Toggle style={{justifyContent: "flex-right"}} 
                          checked={zoom_mode_supported === true} 
                          onClick={() => sendBoolMsg.bind(this)(namespace + "/set_has_zoom_mode",!zoom_mode_supported)} />
                    </Label>

                    <div style={{ display: 'flex' }} >
                        <div style={{ width: '60%' }} >

                                <Label title="Show Process Settings">
                                    <Toggle
                                      checked={(show_settings)}
                                      onClick={() => onChangeSwitchStateValue.bind(this)("auto_show_settings",show_settings)}>
                                    </Toggle>
                                </Label>

                        </div>

                        <div style={{ width: '40%' }}>
                        </div>

                  </div>

                  <div hidden={(show_settings === false)}>





                              <Label title={'Select Process'}>
                                <Select
                                  id="set_auto_process"
                                  onChange={(e) => this.props.ros.sendStringMsg(namespace + "/set_auto_process", e.target.value)}
                                  value={selected_process}
                                >
                                  {processes_menu}
                                </Select>
                              </Label>

                          <ButtonMenu>
                            <Button onClick={() => this.props.ros.sendTriggerMsg(namespace + "/reload_auto_processes")}>{"Reload Processes"}</Button>
                          </ButtonMenu>

                        <div style={{ borderTop: "1px solid #000000", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                        <Label title={"Update Rate"}>
                          <Input
                            id={"AutoUpdateRate"}
                            style={{ width: "45%", float: "left" }}
                            value={auto_update_rate}
                            onChange={this.onAutoUpdateText}
                            onKeyDown={this.onAutoKeyText}
                          />
                        </Label>

                          <div>
                              {/* Map over the auto control names array */}
                              {auto_control_names.map((name, index) => (
                                this.renderAutoControlValues(name, auto_control_values[index], index)
                              ))}
                            </div>





                  <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


                  <Label title={""}>
                    <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Deg"}</div>
                    <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"DPS"}</div>
                  </Label>


                  <Label title={"Pan"}>
                    <Input disabled style={{ width: "45%", float: "left" }} value={round(pan_deg, 2)} />
                    <Input disabled style={{ width: "45%" }} value={round(pan_deg_per_sec, 2)} />
                  </Label>

                  <Label title={"Tilt"}>
                    <Input disabled style={{ width: "45%", float: "left" }} value={round(tilt_deg, 2)} />
                    <Input disabled style={{ width: "45%" }} value={round(tilt_deg_per_sec, 2)} />
                  </Label>

                  <Label title={""}>
                    <div style={{ display: "inline-block", width: "30%", float: "left" }}>{"Roll"}</div>
                    <div style={{ display: "inline-block", width: "30%", float: "center" }}>{"Pitch"}</div>
                    <div style={{ display: "inline-block", width: "30%", float: "right" }}>{"Yaw"}</div>
                  </Label>

                  <Label title={"Nav"}>
                    <Input disabled style={{ width: "30%", float: "left" }} value={round(roll_deg, 2)} />
                    <Input disabled style={{ width: "30%", float: "center" }} value={round(pitch_deg, 2)} />
                    <Input disabled style={{ width: "30%", float: "right" }} value={round(heading_deg, 2)} />
                  </Label>

                  <Label title={"Auto Updates"}>
                    <div style={{ display: "inline-block", width: "30%", float: "left" }}>{"Adj"}</div>
                    <div style={{ display: "inline-block", width: "30%", float: "center" }}>{"Goal"}</div>
                    <div style={{ display: "inline-block", width: "30%", float: "right" }}>{"Speed"}</div>
                  </Label>

                  <Label title={"Pan"}>
                    <Input disabled style={{ width: "30%", float: "left" }} value={round(auto_pan_adj, 2)} />
                    <Input disabled style={{ width: "30%", float: "center" }} value={round(auto_pan_goal, 2)} />
                    <Input disabled style={{ width: "30%", float: "right" }} value={round(auto_pan_dps, 2)} />
                  </Label>

                  <Label title={"Tilt"}>
                    <Input disabled style={{ width: "30%", float: "left" }} value={round(auto_tilt_adj, 2)} />
                    <Input disabled style={{ width: "30%", float: "center" }} value={round(auto_tilt_goal, 2)} />
                    <Input disabled style={{ width: "30%", float: "right" }} value={round(auto_tilt_dps, 2)} />
                  </Label>

                  <Label title={"Auto Rates"}>
                    <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Pos"}</div>
                    <div style={{ display: "inline-block", width: "45%", float: "left" }}>{"Speed"}</div>
                  </Label>


                  <Label title={"Pan"}>
                    <Input disabled style={{ width: "45%", float: "left" }} value={round(auto_pan_pos_rate, 2)} />
                    <Input disabled style={{ width: "45%" }} value={round(auto_pan_vel_rate, 2)} />
                  </Label>

                  <Label title={"Tilt"}>
                    <Input disabled style={{ width: "45%", float: "left" }} value={round(auto_tilt_pos_rate, 2)} />
                    <Input disabled style={{ width: "45%" }} value={round(auto_tilt_vel_rate, 2)} />
                  </Label>

                </div>
        </div>

    <div style={{ borderTop: "1px solid #777777", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

      </React.Fragment>
    )
  }


  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true

    const status_msg = this.state.status_msg
    if (status_msg == null){
      return (
        <Columns>
        <Column>
       
        </Column>
        </Columns>
      )


    }
    else if (make_section === false){

      return (

          <React.Fragment>

              { this.renderPTAutoControls()}


          </React.Fragment>
      )
    }
    else {
      return (

          <Section title={(this.props.title !== undefined) ? this.props.title : ""}>


              {this.renderPTAutoControls()}


        </Section>
     )
   }

  }


}

export default NepiAppPTAutoControls
