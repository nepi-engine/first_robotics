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

from nepi_app_wpilib_if.msg import NepiAppWpilibIFStatus

from nepi_sdk import nepi_sdk

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF

from nepi_api.connect_detections_if import ConnectDetectionsIF
from nepi_api.connect_targets_if import ConnectTargetsIF
from nepi_api.connect_device_if_rbx import ConnectRBXDeviceIF
from nepi_api.connect_device_if_motor import ConnectMotorsDeviceIF
from nepi_api.connect_data_if import ConnectNavPoseIF


#########################################
# Factory Control Values
FACTORY_ENABLED = False
FACTORY_SELECTED_OPTION = "None"
FACTORY_VALUE = 0.0
FACTORY_OPTIONS = ["None", "Option_A", "Option_B", "Option_C"]

STATUS_PUBLISH_RATE_HZ = 1.0

#########################################
# Node Class
#########################################

class NepiWpilibApp(object):

    enabled = FACTORY_ENABLED
    selected_option = FACTORY_SELECTED_OPTION
    value = FACTORY_VALUE
    options = FACTORY_OPTIONS

    node_if = None

    # Per-IF first-connection flags. Each connect IF's callback prints the first
    # data dict and status msg it receives exactly once, then sets its flag.
    got_first_detections = False
    got_first_targets = False
    got_first_rbx = False
    got_first_motors = False
    got_first_navpose = False

    # Latest data dict and status msg per connect IF. Each callback stores both
    # on every invocation, so the rest of the app reads the most recent values
    # from here rather than re-querying the IF.
    detections_dict = None
    detections_status = None
    targets_dict = None
    targets_status = None
    rbx_dict = None
    rbx_status = None
    motors_dict = None
    motors_status = None
    navpose_dict = None
    navpose_status = None

    DEFAULT_NODE_NAME = "app_wpilib"

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
                'msg': NepiAppWpilibIFStatus,
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
        # Surface NEPI connect (consumer) IFs in this app's RUI. Each of the five
        # connect IFs is built with show_selector=True (expose the source/device
        # selector panel) and show_controls=False / show_data=False (hide the
        # controls and data panels), plus a per-IF first-connection callback.
        # ConnectDetectionsIF/ConnectTargetsIF/ConnectNavPoseIF consume their
        # data products; ConnectRBXDeviceIF/ConnectMotorsDeviceIF have no
        # separate data product, so their callback fires with the status dict.
        self.setupInterfaceIFs()

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
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)


    ###################
    ## Interface IF Setup

    def setupInterfaceIFs(self):
        # Connect-side (consumer) IFs. Each auto-discovers the matching producer
        # topic and exposes a selector in this app's RUI (show_selector=True),
        # with the controls and data panels hidden (show_controls=False,
        # show_data=False). Each is given a per-IF first-connection callback that
        # fires once with the received data dict. The connect IFs take no
        # device_info dict or driver callbacks -- they consume the wire contract
        # rather than produce it.
        self.detections_if = ConnectDetectionsIF(
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        callback_function = self.detectionsConnectCb,
                        msg_if = self.msg_if)

        self.targets_if = ConnectTargetsIF(
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        callback_function = self.targetsConnectCb,
                        msg_if = self.msg_if)

        self.navpose_if = ConnectNavPoseIF(
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        callback_function = self.navposeConnectCb,
                        msg_if = self.msg_if)

        # RBX and Motors connect IFs have no separate data product, so their
        # callback_function fires with their status dict on each status update.
        self.rbx_if = ConnectRBXDeviceIF(
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        callback_function = self.rbxConnectCb,
                        msg_if = self.msg_if)

        self.motors_if = ConnectMotorsDeviceIF(
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        callback_function = self.motorsConnectCb,
                        msg_if = self.msg_if)


    ###################
    ## Connect IF First-Connection Callbacks
    #
    # Each connect IF invokes its callback_function with a single data dict. The
    # callback stores that dict and the IF's current status message (via
    # get_status_msg()) on every invocation. On the FIRST invocation per IF it
    # also logs both, then sets the got_first flag so it logs only once. RBX and
    # Motors have no separate data product, so their callback receives the
    # status dict.

    def detectionsConnectCb(self, data_dict):
        self.detections_dict = data_dict
        self.detections_status = self.detections_if.get_status_msg()
        if self.got_first_detections is True:
            return
        self.got_first_detections = True
        self.msg_if.pub_info("Detections first-connection data dict: " + str(self.detections_dict))
        self.msg_if.pub_info("Detections first-connection status message: " + str(self.detections_status))

    def targetsConnectCb(self, data_dict):
        self.targets_dict = data_dict
        self.targets_status = self.targets_if.get_status_msg()
        if self.got_first_targets is True:
            return
        self.got_first_targets = True
        self.msg_if.pub_info("Targets first-connection data dict: " + str(self.targets_dict))
        self.msg_if.pub_info("Targets first-connection status message: " + str(self.targets_status))

    def rbxConnectCb(self, data_dict):
        self.rbx_dict = data_dict
        self.rbx_status = self.rbx_if.get_status_msg()
        if self.got_first_rbx is True:
            return
        self.got_first_rbx = True
        self.msg_if.pub_info("RBX first-connection data dict: " + str(self.rbx_dict))
        self.msg_if.pub_info("RBX first-connection status message: " + str(self.rbx_status))

    def motorsConnectCb(self, data_dict):
        self.motors_dict = data_dict
        self.motors_status = self.motors_if.get_status_msg()
        if self.got_first_motors is True:
            return
        self.got_first_motors = True
        self.msg_if.pub_info("Motors first-connection data dict: " + str(self.motors_dict))
        self.msg_if.pub_info("Motors first-connection status message: " + str(self.motors_status))

    def navposeConnectCb(self, data_dict):
        self.navpose_dict = data_dict
        self.navpose_status = self.navpose_if.get_status_msg()
        if self.got_first_navpose is True:
            return
        self.got_first_navpose = True
        self.msg_if.pub_info("NavPose first-connection data dict: " + str(self.navpose_dict))
        self.msg_if.pub_info("NavPose first-connection status message: " + str(self.navpose_status))


    ###################
    ## Status Publishers

    def statusPublishCb(self, timer):
        self.publish_status()

    def publish_status(self):
        status_msg = NepiAppWpilibIFStatus()
        status_msg.enabled = self.enabled
        status_msg.options = self.options
        status_msg.selected_option = self.selected_option
        status_msg.value = self.value
        status_msg.heartbeat = True
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions
    #######################

    def cleanup_actions(self):
        self.msg_if.pub_info("WPILIB: Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiWpilibApp()
