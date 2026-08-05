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
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import time
import copy

from std_msgs.msg import Bool, Empty, String, Float32

from nepi_app_auto_move.msg import NepiAppAutoMoveStatus

from nepi_sdk import nepi_sdk

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import ControlsIF

from nepi_api.connect_device_if_motor import ConnectMotorsDeviceIF
from nepi_api.connect_data_if import ConnectNavPoseIF
from nepi_api.connect_data_if import ConnectImageIF


#########################################
# Factory Control Values
FACTORY_ENABLED = False
FACTORY_SELECTED_OPTION = "None"
FACTORY_VALUE = 0.0
FACTORY_OPTIONS = ["None", "Option_A", "Option_B", "Option_C"]

STATUS_PUBLISH_RATE_HZ = 1.0

#########################################
# Connect Names
#
# One connect name per connect IF. Each names the <app>/<connect_name> connect
# namespace that IF owns, and is the exact string the matching Nepi_IF_Connect*
# component in NepiAppAutoMove.js binds to. The four motor names and the image
# name are declared rather than defaulted because the class defaults collide --
# every ConnectMotorsDeviceIF would land on 'motor_connect' and ConnectImageIF
# inherits ConnectDataIF's 'data_connect' -- so each is passed explicitly to keep
# the RUI binding unambiguous and greppable from both sides.
MOTOR_1_CONNECT_NAME = "motor_1_connect"
MOTOR_2_CONNECT_NAME = "motor_2_connect"
MOTOR_3_CONNECT_NAME = "motor_3_connect"
MOTOR_4_CONNECT_NAME = "motor_4_connect"
NAVPOSE_CONNECT_NAME = "navpose_connect"
IMAGE_CONNECT_NAME = "image_connect"

#########################################
# Example Controls
#
# Controls name of the example control set: the <app>/example_controls namespace
# one ControlsIF owns and the Nepi_IF_Controls box at the bottom of this app's RUI
# column binds to. See setupExampleControlsIF().
EXAMPLE_CONTROLS_NAME = "example_controls"

#########################################
# Node Class
#########################################

class NepiAutoMoveApp(object):

    enabled = FACTORY_ENABLED
    selected_option = FACTORY_SELECTED_OPTION
    value = FACTORY_VALUE
    options = FACTORY_OPTIONS

    node_if = None

    # The one ControlsIF this app owns: the example control set copied from
    # nepi_app_controls_sandbox. See setupExampleControlsIF().
    example_controls_if = None

    # Per-IF first-connection flags. Each connect IF's callback prints the first
    # data dict and status msg it receives exactly once, then sets its flag.
    got_first_motor_1 = False
    got_first_motor_2 = False
    got_first_motor_3 = False
    got_first_motor_4 = False
    got_first_navpose = False
    got_first_image = False

    # Latest data dict and status msg per connect IF. Each callback stores both
    # on every invocation, so the rest of the app reads the most recent values
    # from here rather than re-querying the IF.
    motor_1_dict = None
    motor_1_status = None
    motor_2_dict = None
    motor_2_status = None
    motor_3_dict = None
    motor_3_status = None
    motor_4_dict = None
    motor_4_status = None
    navpose_dict = None
    navpose_status = None
    image_dict = None
    image_status = None

    DEFAULT_NODE_NAME = "app_auto_move"

    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Initialize Class Variables

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
        self.PARAMS_DICT = {
            'enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.enabled
            },
            'selected_option': {
                'namespace': self.node_namespace,
                'factory_val': self.selected_option
            },
            'value': {
                'namespace': self.node_namespace,
                'factory_val': self.value
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': NepiAppAutoMoveStatus,
                'qsize': 1,
                'latch': True
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_enabled': {
                'namespace': self.node_namespace,
                'topic': 'set_enabled',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setEnabledCb,
                'callback_args': ()
            },
            'set_option': {
                'namespace': self.node_namespace,
                'topic': 'set_option',
                'msg': String,
                'qsize': 10,
                'callback': self.setOptionCb,
                'callback_args': ()
            },
            'set_value': {
                'namespace': self.node_namespace,
                'topic': 'set_value',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setValueCb,
                'callback_args': ()
            },
            'trigger_action': {
                'namespace': self.node_namespace,
                'topic': 'trigger_action',
                'msg': Empty,
                'qsize': 10,
                'callback': self.triggerActionCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
            configs_dict=self.CFGS_DICT,
            params_dict=self.PARAMS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT
        )

        self.node_if.wait_for_ready()

        ##############################
        # Surface NEPI connect (consumer) IFs in this app's RUI. Each of the six
        # connect IFs is built with show_selector=True (expose the source/device
        # selector panel) and show_controls=False / show_data=False (hide the
        # controls and data panels), plus a per-IF first-connection callback.
        # ConnectNavPoseIF and ConnectImageIF consume their data products; the
        # four ConnectMotorsDeviceIF instances have no separate data product, so
        # their callback fires with the status dict.
        self.setupInterfaceIFs()

        ##############################
        # Example controls, built after the app's NodeClassIF and before initCb so
        # the first status publish can already report the controls namespace.
        self.setupExampleControlsIF()

        ##############################
        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    ###################
    ## App Callbacks

    def setEnabledCb(self, msg):
        self.msg_if.pub_info(str(msg))
        self.enabled = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('enabled', self.enabled)
            self.node_if.save_config()

    def setOptionCb(self, msg):
        self.msg_if.pub_info(str(msg))
        option = msg.data
        if option in self.options:
            self.selected_option = option
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_option', self.selected_option)
                self.node_if.save_config()

    def setValueCb(self, msg):
        self.msg_if.pub_info(str(msg))
        self.value = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('value', self.value)
            self.node_if.save_config()

    def triggerActionCb(self, msg):
        # The app's action is to apply its current state: the selected option at
        # the current value. Disabled means the trigger is a no-op rather than a
        # silent one -- an operator pressing it deserves to know why nothing
        # happened. Status is republished so the RUI reflects the trigger.
        if self.enabled is False:
            self.msg_if.pub_warn("Action ignored: app is disabled")
            return
        self.msg_if.pub_info("Action triggered: " + str(self.selected_option) +
                             " = " + str(self.value))
        self.publish_status()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            self.enabled = self.node_if.get_param('enabled')
            self.selected_option = self.node_if.get_param('selected_option')
            self.value = self.node_if.get_param('value')
        if do_updates:
            pass
        self.publish_status()

    def resetCb(self, do_updates=True):
        self.msg_if.pub_warn("Resetting")
        if self.example_controls_if is not None:
            self.example_controls_if.reset()
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        if self.example_controls_if is not None:
            self.example_controls_if.factory_reset()
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)


    ###################
    ## Example Controls
    #
    # A copy of nepi_app_controls_sandbox's demonstration control set, rendered as
    # the Example Controls box at the bottom of this app's RUI column. It drives
    # nothing here: the values are logged when they change and read by nothing
    # else, so the widgets, persistence and update callbacks behave exactly as they
    # do on the sandbox page.
    #
    # node_if is left as None so the IF builds and owns its own NodeClassIF -- the
    # current device-IF convention, and what the sandbox app does. Every registry
    # and param key a ControlsIF creates is prefixed from its own controls_name
    # (system_if.py builds node_if_prefix that way), so it cannot overwrite this
    # app's own entries -- the collision the 2026-07 DECISION LOG entry warns about.

    def setupExampleControlsIF(self):
        self.example_controls_if = ControlsIF(
                        controls_name = EXAMPLE_CONTROLS_NAME,
                        controls_display_name = 'Example Controls',
                        controls_description = 'One control of every supported type',
                        controls_init_dict = self.createExampleControlsInitDict(),
                        controls_updated_callback = self.exampleControlsUpdatedCb,
                        show_controls = True,
                        has_show_control = False,
                        log_name = EXAMPLE_CONTROLS_NAME,
                        msg_if = self.msg_if)
        self.example_controls_if.wait_for_controls_ready()

    # One entry per CONTROL_TYPE, copied from controls_sandbox_app_node.py
    # createControlsInitDict() so the box renders identically to the sandbox page.
    # Insertion order sets the display order.
    #
    # demo_floats_slider is carried for fidelity with the sandbox but does not
    # reach the RUI: nepi_controls' FloatSliders branch references an undefined
    # name and silently drops the control. It is dropped on the sandbox page too.
    def createExampleControlsInitDict(self):
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

    def exampleControlsUpdatedCb(self, control_name):
        # Called by ControlsIF after a control value/display change is applied.
        value = None
        if self.example_controls_if is not None:
            value = self.example_controls_if.get_control_value(control_name)
        self.msg_if.pub_info("Example control '" + str(control_name) + "' updated to: " + str(value))

    # Fully-qualified namespace of the example ControlsIF.
    #
    # Built from self.node_namespace rather than read off
    # ControlsIF.get_namespace(). That method returns
    # create_namespace(node_NAME, controls_name) -- 'app_auto_move/example_controls',
    # with no leading slash. It resolves correctly where the IF itself uses it,
    # since rospy resolves a relative name against the node's parent namespace, but
    # this app publishes the value for the RUI, which appends '/status' and hands it
    # to rosbridge -- where a name with no leading slash resolves at the global root
    # instead of under /<prefix>/<device_id>.
    def getExampleControlsNamespace(self):
        return nepi_sdk.create_namespace(self.node_namespace, EXAMPLE_CONTROLS_NAME)

    def getExampleControlsReadyState(self):
        if self.example_controls_if is None:
            return False
        return self.example_controls_if.get_controls_ready_state()


    ###################
    ## Interface IF Setup

    def setupInterfaceIFs(self):
        # Connect-side (consumer) IFs, in RUI display order: the four motors,
        # NavPose, then the image. Each auto-discovers the matching producer
        # topics and exposes a selector in this app's RUI (show_selector=True),
        # with the controls and data panels hidden (show_controls=False,
        # show_data=False). Each connect_name is passed explicitly so the RUI
        # binding is greppable from here. Each is given a per-IF first-connection
        # callback that fires with the received data dict. The connect IFs take no
        # device_info dict or driver callbacks -- they consume the wire contract
        # rather than produce it.
        #
        # The four motor IFs are four independent instances of the same class,
        # distinguished only by connect_name. Each owns its own connect namespace
        # and its own selector, so the operator picks a motorized device per
        # channel rather than one device for all four.
        self.motor_1_if = ConnectMotorsDeviceIF(
                        connect_name = MOTOR_1_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.motor1ConnectCb,
                        msg_if = self.msg_if)

        self.motor_2_if = ConnectMotorsDeviceIF(
                        connect_name = MOTOR_2_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.motor2ConnectCb,
                        msg_if = self.msg_if)

        self.motor_3_if = ConnectMotorsDeviceIF(
                        connect_name = MOTOR_3_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.motor3ConnectCb,
                        msg_if = self.msg_if)

        self.motor_4_if = ConnectMotorsDeviceIF(
                        connect_name = MOTOR_4_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.motor4ConnectCb,
                        msg_if = self.msg_if)

        self.navpose_if = ConnectNavPoseIF(
                        connect_name = NAVPOSE_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.navposeConnectCb,
                        msg_if = self.msg_if)

        # The image IF's own data panel stays hidden here; the RUI page mounts a
        # SECOND Nepi_IF_ConnectData on this same connect namespace with
        # show_data=true to render the viewer in its left column.
        self.image_if = ConnectImageIF(
                        connect_name = IMAGE_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.imageConnectCb,
                        msg_if = self.msg_if)


    ###################
    ## Connect IF First-Connection Callbacks
    #
    # Each connect IF invokes its dataCB with a single data dict. The callback
    # stores that dict and the IF's current status message (via get_status_msg())
    # on every invocation. On the FIRST invocation per IF it also logs both, then
    # sets the got_first flag so it logs only once. The motor IFs have no separate
    # data product, so their callback receives the status dict.

    def motor1ConnectCb(self, data_dict):
        self.motor_1_dict = data_dict
        self.motor_1_status = self.motor_1_if.get_status_msg()
        if self.got_first_motor_1 is True:
            return
        self.got_first_motor_1 = True
        self.msg_if.pub_info("Motor 1 first-connection data dict: " + str(self.motor_1_dict))
        self.msg_if.pub_info("Motor 1 first-connection status message: " + str(self.motor_1_status))

    def motor2ConnectCb(self, data_dict):
        self.motor_2_dict = data_dict
        self.motor_2_status = self.motor_2_if.get_status_msg()
        if self.got_first_motor_2 is True:
            return
        self.got_first_motor_2 = True
        self.msg_if.pub_info("Motor 2 first-connection data dict: " + str(self.motor_2_dict))
        self.msg_if.pub_info("Motor 2 first-connection status message: " + str(self.motor_2_status))

    def motor3ConnectCb(self, data_dict):
        self.motor_3_dict = data_dict
        self.motor_3_status = self.motor_3_if.get_status_msg()
        if self.got_first_motor_3 is True:
            return
        self.got_first_motor_3 = True
        self.msg_if.pub_info("Motor 3 first-connection data dict: " + str(self.motor_3_dict))
        self.msg_if.pub_info("Motor 3 first-connection status message: " + str(self.motor_3_status))

    def motor4ConnectCb(self, data_dict):
        self.motor_4_dict = data_dict
        self.motor_4_status = self.motor_4_if.get_status_msg()
        if self.got_first_motor_4 is True:
            return
        self.got_first_motor_4 = True
        self.msg_if.pub_info("Motor 4 first-connection data dict: " + str(self.motor_4_dict))
        self.msg_if.pub_info("Motor 4 first-connection status message: " + str(self.motor_4_status))

    def navposeConnectCb(self, data_dict):
        self.navpose_dict = data_dict
        self.navpose_status = self.navpose_if.get_status_msg()
        if self.got_first_navpose is True:
            return
        self.got_first_navpose = True
        self.msg_if.pub_info("NavPose first-connection data dict: " + str(self.navpose_dict))
        self.msg_if.pub_info("NavPose first-connection status message: " + str(self.navpose_status))

    def imageConnectCb(self, data_dict):
        self.image_dict = data_dict
        self.image_status = self.image_if.get_status_msg()
        if self.got_first_image is True:
            return
        self.got_first_image = True
        self.msg_if.pub_info("Image first-connection data dict: " + str(self.image_dict))
        self.msg_if.pub_info("Image first-connection status message: " + str(self.image_status))


    ###################
    ## Status Publishers

    def statusPublishCb(self, timer):
        self.publish_status()

    def publish_status(self):
        status_msg = NepiAppAutoMoveStatus()
        status_msg.enabled = self.enabled
        status_msg.options = self.options
        status_msg.selected_option = self.selected_option
        status_msg.value = self.value
        # Example controls namespace, fully qualified -- see
        # getExampleControlsNamespace() for why it is not ControlsIF.get_namespace().
        # Reported before the IF exists too: initCb() publishes status while
        # NodeClassIF is still being constructed, ahead of setupExampleControlsIF().
        status_msg.example_controls_namespace = self.getExampleControlsNamespace()
        status_msg.example_controls_ready = self.getExampleControlsReadyState()
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions
    #######################

    def cleanup_actions(self):
        self.msg_if.pub_info("AUTO_MOVE: Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiAutoMoveApp()
