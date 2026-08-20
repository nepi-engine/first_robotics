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

from std_msgs.msg import Bool, Empty, String, Float32, Int32

from nepi_app_wpilib_if.msg import NepiAppWpilibIFStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_wpilib_if'


class ConnectAppWpilibIF:
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
            'set_obstacles_namespace': {
                'namespace': self.namespace,
                'topic': 'set_obstacles_namespace',
                'msg': String,
                'qsize': 1
            },
            'set_team_number': {
                'namespace': self.namespace,
                'topic': 'set_team_number',
                'msg': Int32,
                'qsize': 1
            },
            'set_motor_slot_count': {
                'namespace': self.namespace,
                'topic': 'set_motor_slot_count',
                'msg': Int32,
                'qsize': 1
            },
            'set_motor_ids': {
                'namespace': self.namespace,
                'topic': 'set_motor_ids',
                'msg': String,
                'qsize': 1
            },
            'set_motor_names': {
                'namespace': self.namespace,
                'topic': 'set_motor_names',
                'msg': String,
                'qsize': 1
            },
            'set_rbx_enabled': {
                'namespace': self.namespace,
                'topic': 'set_rbx_enabled',
                'msg': Bool,
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
                'msg': NepiAppWpilibIFStatus,
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

    def get_motor_slots(self):
        """Return the app's motor slot mapping and per-slot feedback.

        One entry per slot in slot order, joining the configured RoboRIO
        motor_id and display name to the per-slot RoboRIO fields that have no
        nepi_interfaces/MotorStatus home (control_mode, commanded_output,
        current_amps). The mappable feedback is on the standard motor contract
        instead -- see get_motors_namespace().

        Returns:
            list: One dict per slot, or an empty list before the first status
                message arrives.
        """
        if self.status_msg is None:
            return []
        slots = []
        for feedback_msg in self.status_msg.motor_feedback:
            slots.append(nepi_sdk.convert_msg2dict(feedback_msg))
        return slots

    def get_motors_namespace(self):
        """Return where this app publishes the standard NEPI motor contract.

        Returns:
            str: Namespace of the app's motors interface, publishing
                nepi_interfaces/MotorsStatus at <namespace>/motor_status. "None"
                if the interface is not up, or None before the first status
                message.
        """
        if self.status_msg is None:
            return None
        return self.status_msg.motors_namespace

    def get_seen_motor_ids(self):
        """Return the RoboRIO motor_ids currently present on NetworkTables.

        These are the ids that actually exist on the robot, whether or not they
        are mapped to a slot.

        Returns:
            list: int motor ids, or an empty list before the first status message.
        """
        if self.status_msg is None:
            return []
        return list(self.status_msg.nt_motor_ids)

    def check_rbx_enabled(self):
        """Return whether the app's RBX device is enabled.

        Returns:
            bool: True if the operator has enabled the RBX device.
        """
        if self.status_msg is None:
            return False
        return self.status_msg.rbx_enabled

    def get_rbx_namespace(self):
        """Return where this app's RBX device is advertised.

        Returns:
            str: RBX device namespace, "None" while the device is not built, or
                None before the first status message.
        """
        if self.status_msg is None:
            return None
        return self.status_msg.rbx_namespace

    def get_navpose_topic(self):
        """Return where the robot's NavPose is published.

        There is one owner of that pose and it is the RBX device, so this reads
        "None" while the RBX device is disabled.

        Returns:
            str: NavPose namespace, or None before the first status message.
        """
        if self.status_msg is None:
            return None
        return self.status_msg.navpose_topic

    def get_rbx_feedback_dict(self):
        """Return the RBX Feedback input group as the app last read it.

        Returns:
            dict: supported_capabilities, active_request_id,
                active_request_type, request_status and status_message, or None
                before the first status message.
        """
        if self.status_msg is None:
            return None
        return dict(
            supported_capabilities=list(self.status_msg.supported_capabilities),
            active_request_id=self.status_msg.active_request_id,
            active_request_type=self.status_msg.active_request_type,
            request_status=self.status_msg.request_status,
            status_message=self.status_msg.status_message)

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

    def set_obstacles_namespace(self, namespace):
        """Select the obstacles app namespace the app connects to.

        Args:
            namespace (str): Obstacles app node namespace, or "None" to
                disconnect the obstacles path.
        """
        msg = String()
        msg.data = namespace
        self.con_node_if.publish_pub('set_obstacles_namespace', msg)

    def set_team_number(self, team_number):
        """Set the FRC team number the NetworkTables client connects to.

        Args:
            team_number (int): FRC team number, 1 or greater.
        """
        msg = Int32()
        msg.data = int(team_number)
        self.con_node_if.publish_pub('set_team_number', msg)

    def set_motor_slot_count(self, slot_count):
        """Set how many motor slots the app maps.

        Resizing also resizes the motor id and motor name lists, so new slots
        come up unmapped and unnamed.

        Args:
            slot_count (int): Number of motor slots, 0 or greater.
        """
        msg = Int32()
        msg.data = int(slot_count)
        self.con_node_if.publish_pub('set_motor_slot_count', msg)

    def set_motor_ids(self, motor_ids):
        """Set the ordered RoboRIO motor_id per slot.

        Slot order is the NEPI motor index: it is what
        nepi_interfaces/MotorControl.motor_ind selects and what names motor_0,
        motor_1, ... in MotorsStatus. The whole list is sent in one message, so a
        partial edit can never leave slot order ambiguous.

        Args:
            motor_ids (list): RoboRIO motor ids in slot order. Use -1 for an
                unmapped slot. Accepts a list or an already-joined
                comma-separated string.
        """
        msg = String()
        if isinstance(motor_ids, str):
            msg.data = motor_ids
        else:
            msg.data = ','.join([str(int(motor_id)) for motor_id in motor_ids])
        self.con_node_if.publish_pub('set_motor_ids', msg)

    def set_motor_names(self, motor_names):
        """Set the ordered operator display name per slot.

        These are display names only. MotorStatus.motor_name stays positional
        (motor_0, motor_1, ...).

        Args:
            motor_names (list): Display names in slot order; a blank entry means
                no name. Accepts a list or an already-joined comma-separated
                string.
        """
        msg = String()
        if isinstance(motor_names, str):
            msg.data = motor_names
        else:
            msg.data = ','.join([str(name) for name in motor_names])
        self.con_node_if.publish_pub('set_motor_names', msg)

    def set_rbx_enabled(self, enabled):
        """Enable or disable the app's RBX device.

        Enabling presents the robot to the rest of NEPI as a commandable RBX
        device, and is also what publishes its NavPose. The device appears once
        the RoboRIO has reported its supported capabilities; read
        rbx_namespace off the status message to find where.

        Args:
            enabled (bool): True to present the RBX device.
        """
        msg = Bool()
        msg.data = bool(enabled)
        self.con_node_if.publish_pub('set_rbx_enabled', msg)

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
