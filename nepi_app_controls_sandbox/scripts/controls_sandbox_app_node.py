#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
import os
import time
import sys
import math

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_interfaces.msg import MgrSystemStatus

from nepi_app_controls_sandbox.msg import ControlsSandboxStatus

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import ControlsIF
from nepi_api.data_if import DataIF


#########################################
# Node Class
#########################################

class NepiControlsSandboxApp(object):

  node_if = None
  controls_if = None
  data_if = None

  # Drives the demonstration data values from the DataIF updater callback.
  data_counter = 0

  status_msg = ControlsSandboxStatus()
  status_has_published = False

  active_nodes = []
  active_topics = []
  active_topic_types = []
  active_services = []

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_controls_sandbox"  # Can be overwritten by launch command

  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Controls Sandbox Initialization Processes")

    ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
        'init_callback': self.initCb,
        'reset_callback': self.resetCb,
        'factory_reset_callback': self.factoryResetCb,
        'init_configs': True,
        'namespace': self.node_namespace
    }

    # Params Config Dict ####################
    # This app carries no app-level params of its own; all persisted control
    # state is managed by the ControlsIF instance under <node>/controls, and all
    # persisted datum display state by the DataIF instance under <node>/data.
    self.PARAMS_DICT = dict()

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': ControlsSandboxStatus,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'system_status': {
            'msg': MgrSystemStatus,
            'namespace': self.base_namespace,
            'topic': 'status',
            'qsize': 5,
            'callback': self.systemStatusCb
        }
    }

    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT,
                    msg_if = self.msg_if
    )
    self.node_if.wait_for_ready()

    ##############################
    # Build the demonstration controls: one control of each CONTROL_TYPE.
    controls_init_dict = self.createControlsInitDict()

    # Instantiate a single ControlsIF. node_if is left as None so the IF builds
    # and owns its own NodeClassIF (the current device-IF convention); the app
    # does not share its node_if with the sub-IF.
    self.controls_if = ControlsIF(
                    controls_name = 'controls',
                    controls_display_name = 'Controls Sandbox',
                    controls_description = 'One control of every supported type',
                    controls_init_dict = controls_init_dict,
                    controls_updated_callback = self.controlsUpdatedCb,
                    show_controls = True,
                    has_show_control = False,
                    log_name = 'controls',
                    msg_if = self.msg_if
    )
    self.controls_if.wait_for_controls_ready()

    ##############################
    # Build the demonstration data: one datum of each DATUM_TYPE.
    data_init_dict = self.createDataInitDict()

    # Instantiate a single DataIF. node_if is left as None so the IF builds and
    # owns its own NodeClassIF, the same sharing choice made for ControlsIF
    # above. The RUI display of this data is read only; this node is the only
    # writer of record, and it writes from dataUpdaterCb below.
    self.data_if = DataIF(
                    data_name = 'data',
                    data_display_name = 'Data Sandbox',
                    data_description = 'One datum of every supported type',
                    data_init_dict = data_init_dict,
                    data_updated_callback = self.dataUpdatedCb,
                    data_updater_max_rate = 1,
                    data_updater_callback = self.dataUpdaterCb,
                    show_data = True,
                    has_show_control = False,
                    log_name = 'data',
                    msg_if = self.msg_if
    )
    self.data_if.wait_for_data_ready()

    ##############################
    self.initCb(do_updates = True)

    ##############################
    # Start app status publisher
    self.msg_if.pub_info("Starting status pub")
    nepi_sdk.start_timer_process(1.0, self.publishStatusCb)

    ##############################
    ## Initialization Complete
    self.msg_if.pub_info("Initialization Complete")

    # Spin forever
    nepi_sdk.spin()
    ##############################

  #######################
  ### Controls Definition

  def createControlsInitDict(self):
    # One entry per CONTROL_TYPE, each with a sensible default, bounds/options,
    # display_name and description. Insertion order sets the initial display order.
    controls_init_dict = {
        'demo_menu': {
            'type': 'Menu', 'default': 1, 'options': ['Off', 'Low', 'High'],
            'display_name': 'Demo Menu', 'description': 'Pick one menu option (index based).', 'hidden': False},

        'demo_selection': {
            'type': 'Selection', 'default': 'Bravo', 'options': ['Alpha', 'Bravo', 'Charlie'],
            'display_name': 'Demo Selection', 'description': 'Select a single option by name.', 'hidden': False},

        'demo_selections': {
            'type': 'Selections', 'default': ['Red', 'Blue'], 'options': ['Red', 'Green', 'Blue'],
            'display_name': 'Demo Selections', 'description': 'Select any number of options.', 'hidden': False},

        'demo_trigger': {
            'type': 'Trigger', 'default': 0,
            'display_name': 'Demo Trigger', 'description': 'Fire a one-shot trigger.', 'hidden': False},

        'demo_bool': {
            'type': 'Bool', 'default': True,
            'display_name': 'Demo Bool', 'description': 'Toggle a boolean on or off.', 'hidden': False},

        'demo_string': {
            'type': 'String', 'default': 'hello nepi',
            'display_name': 'Demo String', 'description': 'Free-form text value.', 'hidden': False},

        'demo_int': {
            'type': 'Int', 'default': 5, 'bounds': [0, 10],
            'display_name': 'Demo Int', 'description': 'Integer value within [0, 10].', 'hidden': False},

        'demo_float': {
            'type': 'Float', 'default': 2.5, 'bounds': [0.0, 10.0], 'round_value': 2,
            'display_name': 'Demo Float', 'description': 'Float value within [0.0, 10.0].', 'hidden': False},

        'demo_float_slider': {
            'type': 'FloatSlider', 'default': 50.0, 'bounds': [0.0, 100.0], 'round_value': 1,
            'display_name': 'Demo Float Slider', 'description': 'Single-value slider over [0, 100].', 'hidden': False},

        'demo_floats_slider': {
            'type': 'FloatSliders', 'default': [0.25, 0.75], 'bounds': [0.0, 1.0], 'round_value': 2,
            'display_name': 'Demo Floats Slider', 'description': 'Dual-value range slider (0.0-1.0 ratio).', 'hidden': False},
    }
    return controls_init_dict

  #######################
  ### Data Definition

  def createDataInitDict(self):
    # One entry per DATUM_TYPE. A datum has a value and a timestamp only -- no
    # bounds, options, factory or default values. Insertion order sets the
    # initial display order. round_value rounds the stored float;
    # round_display is the decimal count the RUI formats it to.
    data_init_dict = {
        'demo_bool_data': {
            'type': 'Bool', 'value': True,
            'display_name': 'Demo Bool', 'description': 'A boolean that toggles every update.', 'hidden': False},

        'demo_bools_data': {
            'type': 'Bools', 'value': [True, False],
            'display_name': 'Demo Bools', 'description': 'Two booleans, always opposite.', 'hidden': False},

        'demo_string_data': {
            'type': 'String', 'value': 'starting',
            'display_name': 'Demo String', 'description': 'A wall-clock timestamp string.', 'hidden': False},

        'demo_strings_data': {
            'type': 'Strings', 'value': ['starting', 'tick 0'],
            'display_name': 'Demo Strings', 'description': 'Timestamp plus update counter.', 'hidden': False},

        'demo_int_data': {
            'type': 'Int', 'value': 0,
            'display_name': 'Demo Int', 'description': 'A monotonic update counter.', 'hidden': False},

        'demo_ints_data': {
            'type': 'Ints', 'value': [0, 0],
            'display_name': 'Demo Ints', 'description': 'The counter and its negation.', 'hidden': False},

        'demo_float_data': {
            'type': 'Float', 'value': 0.0, 'round_value': 3, 'round_display': 3,
            'display_name': 'Demo Float', 'description': 'A sine wave over the update counter.', 'hidden': False},

        'demo_floats_data': {
            'type': 'Floats', 'value': [0.0, 0.0], 'round_value': 3, 'round_display': 3,
            'display_name': 'Demo Floats', 'description': 'The sine wave and its negation.', 'hidden': False},
    }
    return data_init_dict

  #######################
  ### App Config Functions

  def systemStatusCb(self, msg):
    self.active_nodes = msg.active_nodes
    self.active_topics = msg.active_topics
    self.active_topic_types = msg.active_topic_types
    self.active_services = msg.active_services

  def initCb(self, do_updates = False):
    if self.node_if is not None:
      pass
    if do_updates == True:
      pass
    self.publish_status()

  def resetCb(self, do_updates = True):
    self.msg_if.pub_warn("Resetting")
    if self.controls_if is not None:
      self.controls_if.reset()
    if self.data_if is not None:
      self.data_if.reset()
    self.initCb(do_updates = do_updates)

  def factoryResetCb(self, do_updates = True):
    self.msg_if.pub_warn("Factory Resetting")
    if self.controls_if is not None:
      self.controls_if.factory_reset()
    if self.data_if is not None:
      self.data_if.factory_reset()
    self.initCb(do_updates = do_updates)

  #######################
  ### Controls Callback

  def controlsUpdatedCb(self, control_name):
    # Called by ControlsIF after a control value/display change is applied.
    value = None
    if self.controls_if is not None:
      value = self.controls_if.get_control_value(control_name)
    self.msg_if.pub_info("Control '" + str(control_name) + "' updated to: " + str(value))

  #######################
  ### Data Callbacks

  def dataUpdaterCb(self):
    # Called once per DataIF updater loop. Drives every datum so the read-only
    # RUI display is visibly live: a counter, a sine, a toggling bool and a
    # wall-clock string. Returns False because set_datum_value() publishes
    # status on each write, so there is nothing left to publish here.
    if self.data_if is None:
      return False

    self.data_counter = self.data_counter + 1
    count = self.data_counter
    sine = math.sin(float(count) / 10.0)
    toggle = (count % 2 == 0)
    stamp = time.strftime('%H:%M:%S')

    self.data_if.set_datum_value('demo_bool_data', toggle)
    self.data_if.set_datum_value('demo_bools_data', [toggle, not toggle])
    self.data_if.set_datum_value('demo_string_data', stamp)
    self.data_if.set_datum_value('demo_strings_data', [stamp, 'tick ' + str(count)])
    self.data_if.set_datum_value('demo_int_data', count)
    self.data_if.set_datum_value('demo_ints_data', [count, -count])
    self.data_if.set_datum_value('demo_float_data', sine)
    self.data_if.set_datum_value('demo_floats_data', [sine, -sine])

    return False

  def dataUpdatedCb(self, datum_name):
    # Called by DataIF after a datum value is written. Left quiet on purpose:
    # the updater above writes eight data per second, so logging each one would
    # flood the message log.
    pass

  #######################
  ### Status

  def publish_status(self):
    status_msg = ControlsSandboxStatus()
    status_msg.name = self.node_name
    if self.controls_if is not None:
      status_msg.controls_namespace = self.controls_if.get_namespace()
      status_msg.controls_ready = self.controls_if.get_controls_ready_state()
    else:
      status_msg.controls_namespace = ''
      status_msg.controls_ready = False
    if self.data_if is not None:
      status_msg.data_namespace = self.data_if.get_namespace()
      status_msg.data_ready = self.data_if.get_data_ready_state()
    else:
      status_msg.data_namespace = ''
      status_msg.data_ready = False
    self.status_msg = status_msg
    if self.node_if is not None:
      if self.status_has_published == False:
        self.msg_if.pub_info("Publishing first Controls Sandbox app status")
        self.status_has_published = True
      self.node_if.publish_pub('status_pub', status_msg)

  def publishStatusCb(self, timer):
    self.publish_status()


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiControlsSandboxApp()
