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

import time

from std_msgs.msg import Bool, Empty, String, Float32

from nepi_interfaces.msg import StringArray

from nepi_app_obstacles.msg import ObstaclesStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_obstacles'
OBSTACLES_TOPIC = 'obstacles'


class ConnectObstaclesIF:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    # Optional consumer callback, invoked from _statusCb with the status dict.
    # The obstacles app publishes no data product this interface subscribes to,
    # so the status dict IS the data dict here.
    dataCB = None

    #######################
    ### IF Initialization

    def __init__(self, namespace = None, dataCB = None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
        # The app node namespace carries the obstacles data topic; the
        # <app>/obstacles namespace carries the status and every command.
        self.app_namespace = nepi_sdk.get_full_namespace(namespace)
        self.namespace = nepi_sdk.create_namespace(self.app_namespace, OBSTACLES_TOPIC)

        self.dataCB = dataCB

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'namespace': self.app_namespace
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'enable': {
                'namespace': self.namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 1
            },
            'set_auto_select_enable': {
                'namespace': self.namespace,
                'topic': 'set_auto_select_enable',
                'msg': Bool,
                'qsize': 1
            },
            'set_source_topic': {
                'namespace': self.namespace,
                'topic': 'set_source_topic',
                'msg': String,
                'qsize': 1
            },
            'set_source_topics': {
                'namespace': self.namespace,
                'topic': 'set_source_topics',
                'msg': StringArray,
                'qsize': 1
            },
            'add_source_topic': {
                'namespace': self.namespace,
                'topic': 'add_source_topic',
                'msg': String,
                'qsize': 1
            },
            'remove_source_topic': {
                'namespace': self.namespace,
                'topic': 'remove_source_topic',
                'msg': String,
                'qsize': 1
            },
            'set_max_process_rate': {
                'namespace': self.namespace,
                'topic': 'set_max_process_rate',
                'msg': Float32,
                'qsize': 1
            },
            'set_max_image_pub_rate': {
                'namespace': self.namespace,
                'topic': 'set_max_image_pub_rate',
                'msg': Float32,
                'qsize': 1
            },
            'set_image_pub': {
                'namespace': self.namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 1
            },
            'set_use_last_image': {
                'namespace': self.namespace,
                'topic': 'set_use_last_image',
                'msg': Bool,
                'qsize': 1
            },
            'save_config': {
                'namespace': self.app_namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'reset_config': {
                'namespace': self.app_namespace,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'factory_reset_config': {
                'namespace': self.app_namespace,
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
                'msg': ObstaclesStatus,
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
            configs_dict = self.CFGS_DICT,
            services_dict = self.SRVS_DICT,
            pubs_dict = self.PUBS_DICT,
            subs_dict = self.SUBS_DICT,
            msg_if = self.msg_if
        )

        self.con_node_if.wait_for_ready()

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        """Return True once this connect interface has finished initializing."""
        return self.ready

    def get_namespace(self):
        """Return the ``<app>/obstacles`` namespace this interface talks to."""
        return self.namespace

    def check_connection(self):
        """Return True if any status message has been received."""
        return self.connected

    def check_status_connection(self):
        """Return True if the obstacles status topic is publishing."""
        return self.status_connected

    def get_status_dict(self):
        """Return the last received ObstaclesStatus message as a dictionary.

        Returns:
            dict: The status message converted to a dict, or None if no status
                message has arrived yet.
        """
        if self.status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.status_msg)
        return None

    def get_status_msg(self):
        """Return the last received ObstaclesStatus message.

        Returns:
            ObstaclesStatus: The most recent status message, or None if no
                status message has arrived yet.
        """
        return self.status_msg

    def set_enabled(self, enabled):
        """Enable or disable obstacle processing.

        Args:
            enabled (bool): True to enable processing, False to disable it.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('enable', msg)

    def set_auto_select_enabled(self, enabled):
        """Enable or disable automatic depth map source selection.

        Args:
            enabled (bool): True to let the app pick a source, False to select
                sources explicitly.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_auto_select_enable', msg)

    def set_source_topic(self, source_topic):
        """Select a single depth map source topic, replacing any current selection.

        Args:
            source_topic (str): Fully-qualified depth map topic name.
        """
        msg = String()
        msg.data = source_topic
        self.con_node_if.publish_pub('set_source_topic', msg)

    def set_source_topics(self, source_topics):
        """Replace the selected depth map source list.

        Args:
            source_topics (list of str): Fully-qualified depth map topic names.
        """
        msg = StringArray()
        msg.array = list(source_topics)
        self.con_node_if.publish_pub('set_source_topics', msg)

    def add_source_topic(self, source_topic):
        """Add one depth map source topic to the selection.

        Args:
            source_topic (str): Fully-qualified depth map topic name.
        """
        msg = String()
        msg.data = source_topic
        self.con_node_if.publish_pub('add_source_topic', msg)

    def remove_source_topic(self, source_topic):
        """Remove one depth map source topic from the selection.

        Args:
            source_topic (str): Fully-qualified depth map topic name.
        """
        msg = String()
        msg.data = source_topic
        self.con_node_if.publish_pub('remove_source_topic', msg)

    def set_max_process_rate(self, rate_hz):
        """Set the maximum obstacle process rate.

        Args:
            rate_hz (float): Requested rate in Hz. The app clamps to 1-20 Hz.
        """
        msg = Float32()
        msg.data = rate_hz
        self.con_node_if.publish_pub('set_max_process_rate', msg)

    def set_max_image_pub_rate(self, rate_hz):
        """Set the maximum overlay image publish rate.

        Args:
            rate_hz (float): Requested rate in Hz. The app clamps to 1-20 Hz.
        """
        msg = Float32()
        msg.data = rate_hz
        self.con_node_if.publish_pub('set_max_image_pub_rate', msg)

    def set_image_pub_enabled(self, enabled):
        """Enable or disable overlay image publishing.

        Args:
            enabled (bool): True to publish the obstacles overlay image.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_image_pub', msg)

    def set_use_last_image(self, enabled):
        """Select whether the overlay renders on the previous frame.

        Args:
            enabled (bool): True to overlay onto the previous frame, which
                aligns the boxes with the frame they were computed from.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_use_last_image', msg)

    def save_config(self):
        """Persist the app's current configuration."""
        self.con_node_if.publish_pub('save_config', Empty())

    def reset_config(self):
        """Reset the app's configuration to the last saved values."""
        self.con_node_if.publish_pub('reset_config', Empty())

    def factory_reset_config(self):
        """Reset the app's configuration to factory values."""
        self.con_node_if.publish_pub('factory_reset_config', Empty())

    def unregister(self):
        """Unregister every publisher and subscriber this interface created."""
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

        if self.dataCB is not None:
            self.dataCB(self.get_status_dict())
