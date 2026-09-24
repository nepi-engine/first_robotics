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
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data

from nepi_interfaces.msg import MgrSystemStatus

from nepi_app_controls_sandbox.msg import ControlsSandboxStatus

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import ControlsIF, DataIF
from nepi_api.connect_data_if import ConnectImageIF


#########################################
# Connect Names
#
# One connect name per connect IF. Each names the <app>/<connect_name> connect
# namespace that IF owns, and is the exact string the matching Nepi_IF_Connect*
# component in NepiAppControlsSandbox.js binds to. Declared rather than defaulted
# because ConnectImageIF inherits ConnectDataIF's generic 'data_connect' default,
# so it is passed explicitly to keep the RUI binding greppable from both sides.
# Matches the arrangement in nepi_app_auto_move.
IMAGE_CONNECT_NAME = "image_connect"


#########################################
# Node Class
#########################################

class NepiControlsSandboxApp(object):

  node_if = None
  controls_if = None
  data_if = None
  image_if = None

  # Latest image data dict and connect status message, stored on every
  # first-connection callback invocation. First-connection flag keeps the log to
  # one entry rather than one per received image.
  image_dict = None
  image_status = None
  got_first_image = False

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
    controls_init_dict = nepi_controls.EXAMPLE_INIT_DICT

    # Instantiate a single ControlsIF. node_if is left as None so the IF builds
    # and owns its own NodeClassIF (the current device-IF convention); the app
    # does not share its node_if with the sub-IF.
    self.controls_if = ControlsIF(
                    controls_name = 'controls',
                    controls_display_name = 'Controls Sandbox',
                    controls_description = 'One control of every supported type',
                    controls_init_dict = controls_init_dict,
                    controls_updated_callback = self.controlsUpdatedCb,
                    log_name = 'controls',
                    msg_if = self.msg_if
    )
    self.controls_if.wait_for_controls_ready()

    ##############################
    # Build the demonstration data: one datum of each DATUM_TYPE.
    data_init_dict = nepi_data.EXAMPLE_INIT_DICT

    # Instantiate a single DataIF. node_if is left as None so the IF builds and
    # owns its own NodeClassIF, the same sharing choice made for ControlsIF
    # above. The RUI display of this data is read only; this node is the only
    # writer of record, and it writes from dataUpdaterCb below.
    self.data_if = DataIF(
                    data_name = 'data',
                    data_display_name = 'Data Sandbox',
                    data_description = 'One datum of every supported type',
                    data_init_dict = data_init_dict,
                    log_name = 'data',
                    msg_if = self.msg_if
    )
    self.data_if.wait_for_data_ready()

    ##############################
    # Image source connect IF. Owns the <node>/image_connect namespace: it
    # discovers the image topics on the device, publishes them as the selector's
    # option list, and subscribes to whichever one the operator picks.
    #
    # show_selector=True is what puts the selector in the RUI; show_controls and
    # show_data stay False because this IF's own data panel is not what renders
    # the viewer -- the page mounts a SECOND Nepi_IF_ConnectData on this same
    # connect namespace with show_data=true for that, so one selection drives
    # both. See the renderImageViewer() comment in NepiAppControlsSandbox.js.
    self.image_if = ConnectImageIF(
                    connect_name = IMAGE_CONNECT_NAME,
                    show_selector = True,
                    show_controls = False,
                    show_data = False,
                    data_callback = self.imageConnectCb,
                    msg_if = self.msg_if
    )

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

  def controlsUpdatedCb(self, control_name, conrol_value):
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

  #######################
  ### Connect IF Callback

  def imageConnectCb(self, data_dict):
    # Called by ConnectImageIF with the received image data dict each time an
    # image arrives from the selected source. Stores the dict and the IF's
    # current connect status, and logs both once, on the first arrival only --
    # an image stream would otherwise flood the message log. The viewer in the
    # RUI does not go through here: it streams from web_video_server against the
    # topic the connect IF reports as selected.
    self.image_dict = data_dict
    self.image_status = self.image_if.get_status_msg()
    if self.got_first_image == True:
      return
    self.got_first_image = True
    self.msg_if.pub_info("Image first-connection data dict: " + str(self.image_dict))
    self.msg_if.pub_info("Image first-connection status message: " + str(self.image_status))

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
    self.dataUpdaterCb()
    self.publish_status()


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiControlsSandboxApp()
