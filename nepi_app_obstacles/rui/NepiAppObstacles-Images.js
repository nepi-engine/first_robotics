/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
 */


import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
//import Select, { Option } from "./Select"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"
import Label from "./Label"
import Toggle from "react-toggle"
import { SliderAdjustment } from "./AdjustmentWidgets"

import {onChangeSwitchStateValue, onUpdateSetStateValue} from "./Utilities"

import NepiIFImageViewerSelector from "./NepiAppPTAuto-ImageViewerSelector"
import NepiIFSaveData from "./Nepi_IF_SaveData"



@inject("ros")
@observer

// MultiImageViewer 
class ImageViewersSelector extends Component {

  constructor(props) {
    super(props)

    this.state = {

      image_topics: ['None','None','None','None'],
      num_windows: 0,

      needs_update: false,

      show_image_controls: false,
      show_selectors: false


    }

    this.getAllSaveNamespace = this.getAllSaveNamespace.bind(this)

    this.renderControlBar = this.renderControlBar.bind(this)
    this.renderImageWindows = this.renderImageWindows.bind(this)
    this.renderSaveData = this.renderSaveData.bind(this)
    this.setNumWindows = this.setNumWindows.bind(this)
    this.resetWindows = this.resetWindows.bind(this)

    this.renderImageViewersSelection = this.renderImageViewersSelection.bind(this)
    
  }


  getAllSaveNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var allNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      allNamespace = "/" + namespacePrefix + "/" + deviceId + '/save_data'
    }
    return allNamespace
  }

  

  componentDidMount(){
      this.setState({needs_update: true})
    }

  // // Lifecycle method called when compnent updates.
  // // Used to track changes in the topic
  // componentDidUpdate(prevProps, prevState, snapshot) {

  // }


  // // Lifecycle method called just before the component umounts.
  // // Used to unsubscribe to Status message
  // componentWillUnmount() {

  // }


  setNumWindows(num_windows){
      var cur_num_windows = this.state.num_windows

      if (num_windows === 0 || num_windows > 4){
          num_windows = 1
      }

      if (num_windows !== cur_num_windows){
        const {sendIntMsg} = this.props.ros
        const num_windows_updated_topic = (this.props.num_windows_updated_topic !== undefined) ? this.props.num_windows_updated_topic : null
        this.setState({num_windows: num_windows})

        if (num_windows_updated_topic != null){
            sendIntMsg( num_windows_updated_topic,num_windows)
        }
        
      }

  }

  resetWindows(){
      const {sendTriggerMsg} = this.props.ros     
      const image_topics = (this.props.image_topics !== undefined) ? this.props.image_topics : [null,null,null,null]
      
      var image_topic = ''

      for (var i = 0; i < image_topics.length; i++) {
        image_topic = image_topics[i]
        if (image_topic != null && image_topic !== 'None' && image_topic !== ''){
          sendTriggerMsg( image_topic + "/reset_renders")
        }
      }
  }


  renderControlBar() {
    const { sendIntMsg, sendBoolMsg } = this.props.ros
    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    const {imageTopics, sendStringMsg} = this.props.ros
    const images_available = imageTopics.length > 0
    const show_controls_bar = (this.props.show_controls_bar !== undefined) ? this.props.show_controls_bar : true
    const num_windows = (this.props.num_windows !== undefined) ? this.props.num_windows : this.state.num_windows
    const switch_num_windows = (num_windows === 1) ? 2 : 1

    const show_selectors_option =  images_available === true
    const custom_selection_options = (this.props.custom_selection_options !== undefined) ? this.props.custom_selection_options : []
    const custom_selection_callback = (this.props.custom_selection_callback !== undefined) ? this.props.custom_selection_callback : null

    const full_screen_enabled = (this.props.full_screen_enabled !== undefined) ? this.props.full_screen_enabled : false
    const full_screen_update_topic = (this.props.full_screen_update_topic !== undefined) ? this.props.full_screen_update_topic : ''


    const dual_mode_supported = (this.props.dual_mode_supported !== undefined) ? this.props.dual_mode_supported : false
    const dual_mode_enabled = (this.props.dual_mode_enabled !== undefined) ? this.props.dual_mode_enabled : false
    const dual_mode_update_topic = (this.props.dual_mode_update_topic !== undefined) ? this.props.dual_mode_update_topic : ''
    const dual_mode_enabled_disabled = (dual_mode_update_topic === '')

    const night_mode_supported = (this.props.night_mode_supported !== undefined) ? this.props.night_mode_supported : false
    const night_mode_enabled = (this.props.night_mode_enabled !== undefined) ? this.props.night_mode_enabled : false
    const night_mode_update_topic = (this.props.night_mode_update_topic !== undefined) ? this.props.night_mode_update_topic : ''
    const night_mode_enabled_disabled = (night_mode_update_topic === '')

    const zoom_mode_supported = (this.props.zoom_mode_supported !== undefined) ? this.props.zoom_mode_supported : false
    const zoom_mode_enabled = (this.props.zoom_mode_enabled !== undefined) ? this.props.zoom_mode_enabled : false
    const zoom_mode_update_topic = (this.props.zoom_mode_update_topic !== undefined) ? this.props.zoom_mode_update_topic : ''
    const zoom_mode_enabled_disabled = (zoom_mode_update_topic === '')

    const detect_mode_supported = (this.props.detect_mode_supported !== undefined) ? this.props.detect_mode_supported : false
    const detect_mode_enabled = (this.props.detect_mode_enabled !== undefined) ? this.props.detect_mode_enabled: false
    const detect_mode_update_topic = (this.props.detect_mode_update_topic !== undefined) ? this.props.detect_mode_update_topic : ''
    const detect_mode_enabled_disabled = (detect_mode_update_topic === '')

    const image_stab_supported = (this.props.image_stab_supported !== undefined) ? this.props.image_stab_supported : false
    const image_stab_enabled = (this.props.image_stab_enabled !== undefined) ? this.props.image_stab_enabled : false
    const image_stab_update_topic = (this.props.image_stab_update_topic !== undefined) ? this.props.image_stab_update_topic : ''
    const image_stab_enabled_disabled = (image_stab_update_topic === '')

    const stream_quality_ratio = (this.props.stream_quality_ratio !== undefined) ? this.props.stream_quality_ratio : 1
    const stream_quality_update_topic = (this.props.stream_quality_update_topic !== undefined) ? this.props.stream_quality_update_topic : ''
    const stream_quality_disabled = (stream_quality_update_topic === '')

    const stream_rate_ratio = (this.props.stream_rate_ratio !== undefined) ? this.props.stream_rate_ratio : 1
    const stream_rate_update_topic = (this.props.stream_rate_update_topic !== undefined) ? this.props.stream_rate_update_topic : ''
    const stream_rate_disabled = (stream_rate_update_topic === '')


    const num_windows_updated_topic = (this.props.num_windows_updated_topic !== undefined) ? this.props.num_windows_updated_topic : null

    return (
      <React.Fragment>

          {show_controls_bar === true ?

                <Columns>
                  <Column>
                  
   
                <div style={{ display: 'flex' }}>

 

                      <div style={{ width: '10%' }} centered={"true"} hidden={dual_mode_supported === false}>


                          <Label title={'Dual View'}>
                            <Toggle style={{justifyContent: "flex-right"}} 
                                disabled={dual_mode_enabled_disabled === true}
                                checked={dual_mode_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(dual_mode_update_topic,!dual_mode_enabled)} />
                          </Label>

                      </div>

                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>

                      <div style={{ width: '10%' }} centered={"true"}  hidden={night_mode_supported === false}>

                            <Label title={'Night View'}>
                              <Toggle style={{justifyContent: "flex-right"}} 
                                disabled={night_mode_enabled_disabled === true}
                                checked={night_mode_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(night_mode_update_topic,!night_mode_enabled)} />
                            </Label>

                      </div>


                   <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>


                      <div style={{ width: '10%' }} centered={"true"} hidden={detect_mode_supported === false}>

                            <Label title={'Detect View'}>
                              <Toggle style={{justifyContent: "flex-right"}} 
                                disabled={detect_mode_enabled_disabled === true}
                                checked={detect_mode_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(detect_mode_update_topic,!detect_mode_enabled)} />
                            </Label>


                      </div>


                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>

                      <div style={{ width: '10%' }} centered={"true"}  hidden={zoom_mode_supported === false}>

                            <div hidden={num_windows > 1}>
                            <Label title={'Zoom View'}>
                              <Toggle style={{justifyContent: "flex-right"}} 
                                disabled={zoom_mode_enabled_disabled === true}
                                checked={zoom_mode_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(zoom_mode_update_topic,!zoom_mode_enabled)} />
                            </Label>

                            </div>



                      </div>

   
                    <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>
  
                      <div style={{ width: '10%' }} centered={"true"}  hidden={image_stab_supported === false}>

                          <Label title={'Image Stab'}>
                              <Toggle style={{justifyContent: "flex-right"}} 
                                checked={image_stab_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(image_stab_update_topic,!image_stab_enabled)} />
                            </Label>


                      </div>


                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>

                      <div style={{ width: '10%' }} centered={"true"} >

                          <Label title={'Full Screen'}>
                              <Toggle style={{justifyContent: "flex-right"}} 
                                checked={full_screen_enabled === true} 
                                onClick={() => sendBoolMsg.bind(this)(full_screen_update_topic,!full_screen_enabled)} />
                            </Label>


                      </div>


                      <div style={{ width: '5%' }} centered={"true"} >
                        {null}
                      </div>


                      <div style={{ width: '10%' }} centered={"true"} hidden={show_selectors_option === false}>


                      <SliderAdjustment
                        disabled={stream_quality_disabled === true}
                        title={"Stream Quality"}
                        msgType={"std_msgs/Float32"}
                        adjustment={stream_quality_ratio}
                        topic={stream_quality_update_topic}
                        scaled={0.01}
                        min={0}
                        max={100}
                        tooltip={"Pan as a percentage (0%=min, 100%=max)"}
                        unit={"%"}
                        noTextBox={true}
                      />


                      {/* <SliderAdjustment
                        disabled={stream_rate_disabled === true}
                        title={"Stream Rate"}
                        msgType={"std_msgs/Float32"}
                        adjustment={stream_rate_ratio}
                        topic={stream_rate_update_topic}
                        scaled={0.01}
                        min={0}
                        max={100}
                        tooltip={"Pan as a percentage (0%=min, 100%=max)"}
                        unit={"%"}
                        noTextBox={true}
                      /> */}

                    </div>


                </div>


                </Column>
                </Columns>
              :
                null
        }

      </React.Fragment>
    )

  }



  renderImageWindows() {
    const admin_mode_set = this.props.ros.systemAdminModeSet

    if (this.state.needs_update === true){
      this.setState({needs_update: false})
    }
    
    const num_windows = (this.props.num_windows !== undefined) ? this.props.num_windows : this.state.num_windows
    const reset_windows = (this.props.reset_windows !== undefined) ? this.props.reset_windows : true
    if (num_windows !== this.state.num_windows){
      if (num_windows > 1 && reset_windows === true){
        this.resetWindows()
      }
      this.setState({num_windows: num_windows})
    }

    const {imageTopics} = this.props.ros
    const images_available = imageTopics.length > 0

    var show_selector_buttons = false
    // if (this.state.num_windows === 1){
    //   show_selector_buttons = true
    // }



    var image_topics = (this.props.image_topics !== undefined) ? this.props.image_topics : ['None','None','None','None']
    if (images_available === false) {
      image_topics =  ['None Available','None Available','None Available','None Available']
    }
    const single_image_index = (this.props.single_image_index !== undefined) ? this.props.single_image_index : 0
    const titles = (this.props.titles !== undefined) ? this.props.titles : [null,null,null,null]
    const image_exclude_filters = (this.props.image_exclude_filters !== undefined) ? this.props.image_exclude_filters : []
    const include_filters = (this.props.include_filters !== undefined) ? this.props.include_filters : []

    const select_updated_topic = (this.props.select_updated_topic !== undefined) ? this.props.select_updated_topic : null
    const select_updated_topics = (this.props.select_updated_topics !== undefined) ? this.props.select_updated_topics : [select_updated_topic,select_updated_topic,select_updated_topic,select_updated_topic]
   
    const mouse_event_topic = (this.props.mouse_event_topic !== undefined) ? this.props.mouse_event_topic : null
    const mouse_event_topics = (this.props.mouse_event_topics !== undefined) ? this.props.mouse_event_topics : [mouse_event_topic,mouse_event_topic,mouse_event_topic,mouse_event_topic]

    const mouse_click_topic = (this.props.mouse_click_topic !== undefined) ? this.props.mouse_click_topic : null
    const mouse_click_topics = (this.props.mouse_click_topics !== undefined) ? this.props.mouse_click_topics : [mouse_click_topic,mouse_click_topic,mouse_click_topic,mouse_click_topic]

    const mouse_drag_topic = (this.props.mouse_drag_topic !== undefined) ? this.props.mouse_drag_topic : null
    const mouse_drag_topics = (this.props.mouse_drag_topics !== undefined) ? this.props.mouse_drag_topics : [mouse_drag_topic,mouse_drag_topic,mouse_drag_topic,mouse_drag_topic]

    const mouse_window_topic = (this.props.mouse_window_topic !== undefined) ? this.props.mouse_window_topic : null
    const mouse_window_topics = (this.props.mouse_window_topics !== undefined) ? this.props.mouse_window_topics : [mouse_window_topic,mouse_window_topic,mouse_window_topic,mouse_window_topic]
  
    const show_selectors = (admin_mode_set === true)
    const auto_select_image = false
    const show_image_controls  = (num_windows === 1)
    const show_info_controls = (this.props.show_info_controls !== undefined) ? this.props.show_info_controls : show_image_controls === true && admin_mode_set === true
    const show_config_controls = (this.props.show_config_controls !== undefined) ? this.props.show_config_controls : show_image_controls === true && admin_mode_set === true
    const show_navpose_controls = (this.props.show_navpose_controls !== undefined) ? this.props.show_navpose_controls : show_image_controls === true && admin_mode_set === true
    const show_render_controls = (this.props.show_render_controls !== undefined) ? this.props.show_render_controls : show_image_controls === true && admin_mode_set === true
    const show_overlay_controls = (this.props.show_overlay_controls !== undefined) ? this.props.show_overlay_controls : show_image_controls === true
    const show_save_controls = false
    const show_all_save_options = false
    const show_all_config_options = false
    const show_reset_button = (this.props.show_reset_button !== undefined) ? this.props.show_reset_button : (num_windows === 1)

    const show_browser_save_button = true
    const allow_pan_zoom = (num_windows === 1)

    const stream_quality_ratio = (this.props.stream_quality_ratio !== undefined) ? this.props.stream_quality_ratio : 1
    const stream_quality_ratio_adj = 0.10 + 0.90 * stream_quality_ratio 
    const streamingImageQuality = (num_windows > 1) ? Math.floor(stream_quality_ratio_adj * 50) : Math.floor(stream_quality_ratio_adj * 95)
    const stream_rate_ratio = (this.props.stream_rate_ratio !== undefined) ? this.props.stream_rate_ratio : 1
    const streamingImageRate =  Math.floor(1 + stream_rate_ratio * 19)
    const has_col_2 = (num_windows > 1) ? true : false
    const colFlexSize_1 = (has_col_2 === false)? "100%" : "49%"
    const colFlexSize_gap = (has_col_2 === false)? "0%" : "2%"
    const colFlexSize_2 = (has_col_2 === false)? "0%" : "49%"
    const make_section = true //(num_windows !== 1)
  
    const night_mode_enabled = (this.props.night_mode_enabled !== undefined) ? this.props.night_mode_enabled : false
    const zoom_mode_enabled = (this.props.zoom_mode_enabled !== undefined) ? this.props.zoom_mode_enabled : false
    const detect_mode_enabled = (this.props.detect_mode_enabled !== undefined) ? this.props.detect_mode_enabled : false
    

  if (num_windows === 1){
    return(

  <React.Fragment>
              <div style={{ display: 'flex' }}>
            
                  <div style={{ width: "100%" }}>

                      <div id="Image0Viewer">
                        <NepiIFImageViewerSelector
                          id={single_image_index}
                          image_index={0}
                          image_topic={image_topics[single_image_index]}
                          title={titles[single_image_index]}
                          streamingImageQuality={streamingImageQuality}
                          streamingImageRate={streamingImageRate}
                          image_exclude_filters={image_exclude_filters}
                          include_filters={include_filters}
                          show_image_controls={show_image_controls}
                          show_info_controls={show_info_controls}
                          show_config_controls={show_config_controls}
                          show_navpose_controls={show_navpose_controls}
                          show_render_controls={show_render_controls}
                          show_overlay_controls={show_overlay_controls}
                          show_selector={show_selectors}
                          show_selector_buttons={false}
                          show_browser_save_button={show_browser_save_button}
                          allow_pan_zoom={allow_pan_zoom}
                          mouse_event_topic={mouse_event_topics[single_image_index]}
                          mouse_click_topic={mouse_click_topics[single_image_index]}
                          mouse_drag_topic={mouse_drag_topics[single_image_index]}
                          mouse_window_topic={mouse_window_topics[single_image_index]}
                          select_updated_topic={select_updated_topics[single_image_index]}
                          auto_select_image={auto_select_image}
                          make_section={make_section}
                          show_save_controls={show_save_controls}
                          show_all_save_options={show_all_save_options}
                          show_all_config_options={show_all_config_options}
                          show_reset_button={show_reset_button}
                        />
                      </div>

                  </div>

   
          </div> 
        </React.Fragment>


    )


  }
  else{

      return (
     
        <React.Fragment>
              <div style={{ display: 'flex' }}>
            
                  <div style={{ width: "49%" }}>

                    {(night_mode_enabled === false)?
                      <div id="Image1Viewer">
                        <NepiIFImageViewerSelector
                          id="0"
                          image_index={0}
                          image_topic={image_topics[0]}
                          title={titles[0]}
                          streamingImageQuality={streamingImageQuality}
                          streamingImageRate={streamingImageRate}
                          image_exclude_filters={image_exclude_filters}
                          include_filters={include_filters}
                          show_image_controls={show_image_controls}
                          show_info_controls={show_info_controls}
                          show_config_controls={show_config_controls}
                          show_navpose_controls={show_navpose_controls}
                          show_render_controls={show_render_controls}
                          show_overlay_controls={show_overlay_controls}
                          show_selector={show_selectors}
                          show_selector_buttons={false}
                          show_browser_save_button={show_browser_save_button}
                          allow_pan_zoom={allow_pan_zoom}
                          mouse_event_topic={mouse_event_topics[0]}
                          mouse_click_topic={mouse_click_topics[0]}
                          mouse_drag_topic={mouse_drag_topics[0]}
                          mouse_window_topic={mouse_window_topics[0]}
                          select_updated_topic={select_updated_topics[0]}
                          auto_select_image={auto_select_image}
                          make_section={make_section}
                          show_save_controls={show_save_controls}
                          show_all_save_options={show_all_save_options}
                          show_all_config_options={show_all_config_options}
                          show_reset_button={show_reset_button}
                        />
                      </div>
                        : null
                        }
 
                  </div>


                  <div style={{ width: "2%" }}>
                        {}
                  </div>

                  <div style={{ width: "49%" }}>

                        {(night_mode_enabled === false )?
                          <div id="Image2Viewer">
                            <NepiIFImageViewerSelector
                              id="1"
                              image_index={1}
                              image_topic={image_topics[1]}
                              title={titles[1]}
                              streamingImageQuality={streamingImageQuality}
                              streamingImageRate={streamingImageRate}
                              image_exclude_filters={image_exclude_filters}
                              include_filters={include_filters}
                              show_image_controls={show_image_controls}
                              show_info_controls={show_info_controls}
                              show_config_controls={show_config_controls}
                              show_navpose_controls={show_navpose_controls}
                              show_render_controls={show_render_controls}
                              show_overlay_controls={show_overlay_controls}
                              show_selector={show_selectors}
                              show_selector_buttons={false}
                              show_browser_save_button={show_browser_save_button}
                              allow_pan_zoom={allow_pan_zoom}
                              mouse_event_topic={mouse_event_topics[1]}
                              mouse_click_topic={mouse_click_topics[1]}
                              mouse_drag_topic={mouse_drag_topics[1]}
                              mouse_window_topic={mouse_window_topics[1]}
                              select_updated_topic={select_updated_topics[1]}
                              auto_select_image={auto_select_image}
                             make_section={make_section}
                          show_save_controls={show_save_controls}
                          show_all_save_options={show_all_save_options}
                          show_all_config_options={show_all_config_options}
                          show_reset_button={show_reset_button}
                            />
                          </div>          
                        : null
                        }

                  </div>
   
          </div> 


              <div style={{ display: 'flex' }}>
            
                  <div style={{ width: "49%" }}>

                  

                        {(night_mode_enabled === true)?
                          <div id="Image3Viewer">
                            <NepiIFImageViewerSelector
                              id="2"
                              image_index={2}
                              image_topic={image_topics[2]}
                              title={titles[2]}
                              streamingImageQuality={streamingImageQuality}
                              streamingImageRate={streamingImageRate}
                              image_exclude_filters={image_exclude_filters}
                              include_filters={include_filters}
                              show_image_controls={show_image_controls}
                              show_info_controls={show_info_controls}
                              show_config_controls={show_config_controls}
                              show_navpose_controls={show_navpose_controls}
                              show_render_controls={show_render_controls}
                              show_overlay_controls={show_overlay_controls}
                              show_selector={show_selectors}
                              show_selector_buttons={false}
                              show_browser_save_button={show_browser_save_button}
                              allow_pan_zoom={allow_pan_zoom}
                              mouse_event_topic={mouse_event_topics[2]}
                              mouse_click_topic={mouse_click_topics[2]}
                              mouse_drag_topic={mouse_drag_topics[2]}
                              mouse_window_topic={mouse_window_topics[2]}
                              select_updated_topic={select_updated_topics[2]}
                              auto_select_image={auto_select_image}
                              make_section={make_section}
                          show_save_controls={show_save_controls}
                          show_all_save_options={show_all_save_options}
                          show_all_config_options={show_all_config_options}
                          show_reset_button={show_reset_button}
                            />
                          </div>        
                        : null
                        }
                  </div>


                  <div style={{ width: "2%" }}>
                        {}
                  </div>

                  <div style={{ width: "49%" }}>

                         {night_mode_enabled === true ?
                          <div id="Image4Viewer">
                            <NepiIFImageViewerSelector
                              id="3"
                              image_index={3}
                              image_topic={image_topics[3]}
                              title={titles[3]}
                              streamingImageQuality={streamingImageQuality}
                              streamingImageRate={streamingImageRate}
                              image_exclude_filters={image_exclude_filters}
                              include_filters={include_filters}
                              show_image_controls={show_image_controls}
                              show_info_controls={show_info_controls}
                              show_config_controls={show_config_controls}
                              show_navpose_controls={show_navpose_controls}
                              show_render_controls={show_render_controls}
                              show_overlay_controls={show_overlay_controls}
                              show_selector={show_selectors}
                              show_selector_buttons={false}
                              show_browser_save_button={show_browser_save_button}
                              allow_pan_zoom={allow_pan_zoom}
                              mouse_event_topic={mouse_event_topics[3]}
                              mouse_click_topic={mouse_click_topics[3]}
                              mouse_drag_topic={mouse_drag_topics[3]}
                              mouse_window_topic={mouse_window_topics[3]}
                              select_updated_topic={select_updated_topics[3]}
                              auto_select_image={auto_select_image}
                             make_section={make_section}
                          show_save_controls={show_save_controls}
                          show_all_save_options={show_all_save_options}
                          show_all_config_options={show_all_config_options}
                          show_reset_button={show_reset_button}
                            />
                          </div>          
                        : null
                        }
                  </div>
   
          </div> 
        </React.Fragment>

      )
    }
    
  }


    renderSaveData(){
      const allSaveNamespace = this.getAllSaveNamespace()
      const saveNamespace = (this.props.saveNamespace !== undefined) ? this.props.saveNamespace : allSaveNamespace

    
          return (
        
              <React.Fragment>
                          
                          <NepiIFSaveData
                            saveNamespace={saveNamespace}
                            make_section={false}
                            show_all_options={true}
                            show_topic_selector={true}
                          />
        
              </React.Fragment>

          )
        
  }


  renderImageViewersSelection() {
    const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : true
    const show_controls_bar = (this.props.show_controls_bar !== undefined) ? this.props.show_controls_bar : true
    const show_constrols_bar_bottom = (this.props.show_constrols_bar_bottom !== undefined) ? this.props.show_constrols_bar_bottom : false
    
     return (
        
        <React.Fragment>

            {(show_controls_bar === true && show_constrols_bar_bottom === false) ?
               this.renderControlBar()
            : null }      

            {(show_controls_bar === true && show_constrols_bar_bottom === false) ?
              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            : null }              
         
              
                {/* {(show_save_controls === true ) ?
                 this.renderSaveData()
              : null } */}

              {(show_save_controls === true ) ?
                <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
              : null }

              {this.renderImageWindows()}



              {(show_save_controls === true ) ?
                this.renderSaveData()
            : null }


            {(show_save_controls === true  && show_controls_bar === true && show_constrols_bar_bottom === true) ?
              <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
            : null }              


            {(show_controls_bar === true && show_constrols_bar_bottom === true) ?
               this.renderControlBar()
            : null }        

          </React.Fragment>

      )

  }





  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true
    
    if (make_section === false){
      return (
          <React.Fragment>
 
            {this.renderImageViewersSelection()}    
               

          </React.Fragment>
      )
    }
    else {
      return (

      <Section>

            {this.renderImageViewersSelection()} 

      </Section>
      )

    }
  }

}

export default ImageViewersSelector
