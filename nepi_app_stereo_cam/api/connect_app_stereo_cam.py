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

from nepi_interfaces.msg import UpdateFloat

from nepi_app_stereo_cam.msg import NepiAppStereoCamStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_stereo_cam'


class ConnectAppStereoCam:
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
            'set_selected_process': {
                'namespace': self.namespace,
                'topic': 'set_selected_process',
                'msg': String,
                'qsize': 1
            },
            'reload_processes': {
                'namespace': self.namespace,
                'topic': 'reload_processes',
                'msg': Empty,
                'qsize': 1
            },
            # Stereo calibration (same topics the RUI calibration panel drives)
            'set_calib_board_value': {
                'namespace': self.namespace,
                'topic': 'set_calib_board_value',
                'msg': UpdateFloat,
                'qsize': 1
            },
            'set_calib_file': {
                'namespace': self.namespace,
                'topic': 'set_calib_file',
                'msg': String,
                'qsize': 1
            },
            'capture_calib_frame': {
                'namespace': self.namespace,
                'topic': 'capture_calib_frame',
                'msg': Empty,
                'qsize': 1
            },
            'solve_calib': {
                'namespace': self.namespace,
                'topic': 'solve_calib',
                'msg': Empty,
                'qsize': 1
            },
            'clear_calib': {
                'namespace': self.namespace,
                'topic': 'clear_calib',
                'msg': Empty,
                'qsize': 1
            },
            'load_calib': {
                'namespace': self.namespace,
                'topic': 'load_calib',
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
                'msg': NepiAppStereoCamStatus,
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

    def set_selected_process(self, process_name):
        """Select the active stereo process by name."""
        msg = String()
        msg.data = process_name
        self.con_node_if.publish_pub('set_selected_process', msg)

    def reload_processes(self):
        """Trigger a reload of the stereo processes module."""
        self.con_node_if.publish_pub('reload_processes', Empty())

    def set_calib_board(self, board_cols=None, board_rows=None, square_mm=None):
        """Describe the chessboard: INNER corner counts + printed square size.

        Changing the corner counts clears any captured views in the app node.
        """
        for name, value in (('board_cols', board_cols),
                            ('board_rows', board_rows),
                            ('square_mm', square_mm)):
            if value is None:
                continue
            msg = UpdateFloat()
            msg.name = name
            msg.value = float(value)
            self.con_node_if.publish_pub('set_calib_board_value', msg)

    def set_calib_file(self, calib_file):
        """Set where the calibration .npz is written / read."""
        msg = String()
        msg.data = calib_file
        self.con_node_if.publish_pub('set_calib_file', msg)

    def capture_calib_frame(self):
        """Find the board in the current L/R pair and keep it for the solve.

        Call this once per board pose (~10-20 poses at varied distances, angles
        and image corners). Results land in the status message's calib_message /
        calib_capture_count.
        """
        self.con_node_if.publish_pub('capture_calib_frame', Empty())

    def solve_calib(self):
        """Solve from the captured views, save the .npz, and start rectifying."""
        self.con_node_if.publish_pub('solve_calib', Empty())

    def clear_calib(self):
        """Drop the captured views (does not delete a saved .npz)."""
        self.con_node_if.publish_pub('clear_calib', Empty())

    def load_calib(self):
        """Reload the calibration .npz at the configured path."""
        self.con_node_if.publish_pub('load_calib', Empty())

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
