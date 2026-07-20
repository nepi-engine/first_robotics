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

from std_msgs.msg import Bool, Empty, String, Float32

from nepi_app_template.msg import NepiAppTemplateStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_template'


class ConnectAppTemplate:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    #######################
    ### IF Initialization

    def __init__(self, namespace=None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'namespace': self.namespace
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'set_enabled': {
                'namespace': self.namespace,
                'topic': 'set_enabled',
                'msg': Bool,
                'qsize': 1
            },
            'set_option': {
                'namespace': self.namespace,
                'topic': 'set_option',
                'msg': String,
                'qsize': 1
            },
            'set_value': {
                'namespace': self.namespace,
                'topic': 'set_value',
                'msg': Float32,
                'qsize': 1
            },
            'trigger_action': {
                'namespace': self.namespace,
                'topic': 'trigger_action',
                'msg': Empty,
                'qsize': 1
            },
            'save_config': {
                'namespace': self.namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'reset_config': {
                'namespace': self.namespace,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'factory_reset_config': {
                'namespace': self.namespace,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': NepiAppTemplateStatus,
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        # NOTE: ConnectNodeClassIF takes no 'namespace' kwarg -- the target
        # app's namespace is carried by the per-entry 'namespace' fields in
        # the dicts above (and CFGS_DICT['namespace']). Some shipped
        # connect_app_* files pass namespace=/log_class_name= anyway; those
        # kwargs don't exist and would TypeError if ever instantiated.
        self.con_node_if = ConnectNodeClassIF(
            configs_dict=self.CFGS_DICT,
            services_dict=self.SRVS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
            msg_if=self.msg_if
        )

        self.con_node_if.wait_for_ready()

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        return self.ready

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def check_status_connection(self):
        return self.status_connected

    def get_status_dict(self):
        if self.status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.status_msg)
        return None

    def set_enabled(self, enabled):
        """Enable or disable the app."""
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_enabled', msg)

    def set_option(self, option):
        """Set the selected option string."""
        msg = String()
        msg.data = option
        self.con_node_if.publish_pub('set_option', msg)

    def set_value(self, value):
        """Set the float value."""
        msg = Float32()
        msg.data = value
        self.con_node_if.publish_pub('set_value', msg)

    def trigger_action(self):
        """Trigger the one-shot action."""
        self.con_node_if.publish_pub('trigger_action', Empty())

    def save_config(self):
        self.con_node_if.publish_pub('save_config', Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config', Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config', Empty())

    def unregister(self):
        self._unregisterNode()


    ###############################
    # Class Private Methods
    ###############################

    def _unregisterNode(self):
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.status_connected = False
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister: " + str(e))

    def _statusCb(self, status_msg):
        self.status_connected = True
        self.connected = True
        self.status_msg = status_msg
