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

import copy
import math
import threading
import time

from std_msgs.msg import Bool, Empty, String, Float32, Int32

from nepi_app_wpilib_if.msg import NepiAppWpilibIFStatus, WpilibMotorFeedback

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import ControlsIF
from nepi_api.device_if_motor import MotorsDeviceIF

from nepi_api.connect_detections_if import ConnectDetectionsIF
from nepi_api.connect_data_if import ConnectNavPoseIF
# nepi_app_obstacles' CMakeLists installs its api/*.py flat into nepi_api, so at
# runtime ConnectObstaclesIF sits beside the two above despite living in that
# app's source tree.
from nepi_api.connect_obstacles_if import ConnectObstaclesIF

# The NetworkTables access layer and the RBX device, both installed beside this
# file (scripts/ is what deploy_app.sh live-syncs, and CMakeLists installs all
# three into the same lib/<app> directory), so a plain module import resolves
# them from this script's own directory.
import nepi_wpilib
from wpilib_rbx_if import WpilibRbxIF


#########################################
# Factory Control Values
FACTORY_ENABLED = False
FACTORY_SELECTED_OPTION = "None"
FACTORY_VALUE = 0.0
FACTORY_OPTIONS = ["None", "Option_A", "Option_B", "Option_C"]

# Unselected state of the obstacles app selector. The RUI sends this string when
# the operator picks the blank entry, and it is what the node reports back on
# status while nothing is connected.
NONE_NAMESPACE = "None"

STATUS_PUBLISH_RATE_HZ = 1.0

#########################################
# Robot Interface Factory Values

# FRC team number the NetworkTables client derives the RoboRIO address from.
FACTORY_TEAM_NUMBER = nepi_wpilib.TEAM_NUMBER

# Operator-facing motor model: an ordered list of slots, each holding one
# RoboRIO motor_id. Slot order IS the NEPI motor index -- it is what
# MotorControl.motor_ind selects and what names motor_0, motor_1, ... in
# MotorsStatus. Four slots by default, any length allowed.
# See docs/WPILIB_IF_DESIGN.md, Decision 2.
FACTORY_MOTOR_SLOT_COUNT = 4
MAX_MOTOR_SLOT_COUNT = 32

# A slot holding this motor_id is unmapped: no command is written for it and it
# reports as not enabled. Setting a slot to this is the one way to turn a slot
# off -- there is deliberately no second per-slot enable flag to disagree with it.
UNMAPPED_MOTOR_ID = -1

# The RBX device is off by default. Enabling it is what presents this robot to
# the rest of NEPI as a commandable RBX device (and, per Decision 4, what
# publishes its NavPose).
FACTORY_RBX_ENABLED = False

# How often the NetworkTables input groups are read into the node's caches.
# Matches the 10 Hz navpose update rate the RBX device is built with, so the
# pose its goto convergence loop reads is never older than one poll.
NT_POLL_RATE_HZ = 10.0

# How often NEPI writes the heartbeat True. The RoboRIO answers with False after
# nepi_wpilib.HEARTBEAT_RESPOND_DELAY_SECONDS.
HEARTBEAT_PUBLISH_RATE_HZ = float(1) / nepi_wpilib.HEARTBEAT_PERIOD_SECONDS

# Freshness bounds. Age is measured locally from the last observed change in a
# group's field set, which requires the RoboRIO to advance that group's
# timestamp field on every publish -- which is what the field is for.
#
# NavPose at 1.0 s is ten poll periods: long enough that a scheduling hiccup
# does not invalidate the pose, short enough that a stopped robot code shows up
# before a goto has run far on stale position. Motor feedback is given 2.0 s
# because per-motor telemetry is a readout, not a control input.
NAVPOSE_STALE_SEC = 1.0
MOTOR_STALE_SEC = 2.0

# How far the RoboRIO's own group timestamp may sit from this device's clock
# before it is treated as a different clock domain and the locally observed
# change time is used instead. On an FRC network NEPI's NTP source is the
# roboRIO itself, so the two normally agree closely; a full minute of tolerance
# accepts ordinary sync error while still rejecting an unset or nonsense stamp.
NAVPOSE_CLOCK_SKEW_SEC = 60.0

# Device name reported in MotorsStatus and RBX info/status.
ROBOT_DEVICE_NAME = "roborio"

# The COMMAND_TYPE_* codes handed to the RBX module, so that module never has to
# know the NetworkTables contract it is writing into.
RBX_COMMAND_TYPES = {
    'chassis_speeds': nepi_wpilib.COMMAND_TYPE_CHASSIS_SPEEDS,
    'target_pose': nepi_wpilib.COMMAND_TYPE_TARGET_POSE,
    'named_action': nepi_wpilib.COMMAND_TYPE_NAMED_ACTION,
    'stop': nepi_wpilib.COMMAND_TYPE_STOP,
}

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

class NepiWpilibApp(object):

    enabled = FACTORY_ENABLED
    selected_option = FACTORY_SELECTED_OPTION
    value = FACTORY_VALUE
    options = FACTORY_OPTIONS
    connected = False

    node_if = None

    # The one ControlsIF this app owns: the example control set copied from
    # nepi_app_controls_sandbox. See setupExampleControlsIF().
    example_controls_if = None

    # Per-IF first-connection flags. Each connect IF's callback prints the first
    # data dict and status msg it receives exactly once, then sets its flag.
    got_first_detections = False
    got_first_obstacles = False
    got_first_navpose = False

    # Latest data dict and status msg per connect IF. Each callback stores both
    # on every invocation, so the rest of the app reads the most recent values
    # from here rather than re-querying the IF.
    detections_dict = None
    detections_status = None
    obstacles_dict = None
    obstacles_status = None
    navpose_dict = None
    navpose_status = None

    # Obstacles connect path. ConnectObstaclesIF is not a ConnectNodeIF: it has
    # no selector or auto-discovery of its own, so the operator picks an
    # obstacles app namespace in the RUI and this node builds the IF against it.
    # obstacles_namespace is the current selection, reported back on status.
    obstacles_if = None
    obstacles_namespace = NONE_NAMESPACE

    #############################
    # Robot interface state

    # Robot interface configuration, all persisted as params.
    team_number = FACTORY_TEAM_NUMBER
    motor_slot_count = FACTORY_MOTOR_SLOT_COUNT
    motor_ids = []
    motor_names = []
    rbx_enabled = FACTORY_RBX_ENABLED

    # The one NetworkTables client this process owns.
    nt_instance = None
    nt_conn_handle = None
    nt_heartbeat_entry = None

    heartbeat_waiting_for_response = False
    heartbeat_last_assert_time = 0.0
    heartbeat_last_response_time = 0.0
    demo_bool_value = None

    # Live NetworkTables input-group caches, one per group, following the
    # per-IF latest-value pattern the connect callbacks above already use: every
    # poll stores the newest dict here and the rest of the app reads from these
    # rather than touching NetworkTables itself. None means never seen.
    #
    # Streaming telemetry lives here and NOT in params -- see
    # docs/WPILIB_IF_DESIGN.md, Decision 1.
    motor_feedback_dict = {}      # RoboRIO motor_id -> feedback dict
    nt_motor_ids = []             # motor_ids currently present on NetworkTables
    position_dict = None
    velocity_dict = None
    orientation_dict = None
    rbx_feedback_dict = None

    # Per-group first-connection flags, matching the connect-IF pattern above:
    # the first dict received per group is logged exactly once.
    got_first_motor_feedback = False
    got_first_position = False
    got_first_velocity = False
    got_first_orientation = False
    got_first_rbx_feedback = False

    # The standard NEPI multi-motor producer this app hosts, and the RBX device.
    motors_if = None
    rbx_if = None

    DEFAULT_NODE_NAME = "app_wpilib_if"

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

        # Guards RBX device construction against teardown -- the two are reached
        # from different threads (the NT poll timer and a subscriber callback).
        self.rbx_lock = threading.Lock()

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
            },
            # Robot interface configuration. Configuration belongs in params;
            # the telemetry it describes does not (Decision 1).
            'team_number': {
                'namespace': self.node_namespace,
                'factory_val': FACTORY_TEAM_NUMBER
            },
            'motor_slot_count': {
                'namespace': self.node_namespace,
                'factory_val': FACTORY_MOTOR_SLOT_COUNT
            },
            'motor_ids': {
                'namespace': self.node_namespace,
                'factory_val': self.getFactoryMotorIds()
            },
            'motor_names': {
                'namespace': self.node_namespace,
                'factory_val': self.getFactoryMotorNames()
            },
            'rbx_enabled': {
                'namespace': self.node_namespace,
                'factory_val': FACTORY_RBX_ENABLED
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
            },
            'set_obstacles_namespace': {
                'namespace': self.node_namespace,
                'topic': 'set_obstacles_namespace',
                'msg': String,
                'qsize': 10,
                'callback': self.setObstaclesNamespaceCb,
                'callback_args': ()
            },
            # Robot interface setters. Each persists through node_if.set_param
            # plus save_config exactly as the setters above do.
            'set_team_number': {
                'namespace': self.node_namespace,
                'topic': 'set_team_number',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setTeamNumberCb,
                'callback_args': ()
            },
            'set_motor_slot_count': {
                'namespace': self.node_namespace,
                'topic': 'set_motor_slot_count',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setMotorSlotCountCb,
                'callback_args': ()
            },
            # The ordered per-slot lists arrive as one comma-separated String
            # each, e.g. "1,2,-1,4" and "left front,right front,,right rear".
            # One message carries the whole list, so a partial edit can never
            # leave the two lists disagreeing about slot order, and the RUI --
            # which already holds the full list from status -- sends the whole
            # list back after editing one slot. std_msgs has no ordered
            # int-plus-string pair type, and a per-slot topic would need one
            # topic per slot for a list whose length is itself configurable.
            'set_motor_ids': {
                'namespace': self.node_namespace,
                'topic': 'set_motor_ids',
                'msg': String,
                'qsize': 10,
                'callback': self.setMotorIdsCb,
                'callback_args': ()
            },
            'set_motor_names': {
                'namespace': self.node_namespace,
                'topic': 'set_motor_names',
                'msg': String,
                'qsize': 10,
                'callback': self.setMotorNamesCb,
                'callback_args': ()
            },
            'set_rbx_enabled': {
                'namespace': self.node_namespace,
                'topic': 'set_rbx_enabled',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setRbxEnabledCb,
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
        # Surface NEPI connect (consumer) IFs in this app's RUI. The two
        # nepi_api connect IFs -- ConnectDetectionsIF and ConnectNavPoseIF --
        # are built with show_selector=True (expose the source selector panel)
        # and show_controls=False / show_data=False (hide the controls and data
        # panels), plus a per-IF first-connection callback that fires with the
        # data product they consume. The third connect path, Obstacles, is not a
        # ConnectNodeIF and has no selector of its own: ConnectObstaclesIF is
        # built on demand against the app namespace the operator picks in the
        # RUI. See connectObstacles().
        self.setupInterfaceIFs()

        ##############################
        # Example controls, built after the app's NodeClassIF and before initCb so
        # the first status publish can already report the controls namespace.
        self.setupExampleControlsIF()

        ##############################
        self.initCb(do_updates=True)

        ##############################
        # The one NetworkTables client for this process, plus the standard NEPI
        # multi-motor producer that republishes what it reads. Both are started
        # after initCb, so the client opens against the persisted team number and
        # the motor slots are already mapped when the first status is published.
        self.startNetworkTables()
        self.setupMotorsIF()

        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)
        nepi_sdk.start_timer_process(float(1) / NT_POLL_RATE_HZ, self.ntPollCb)
        nepi_sdk.start_timer_process(float(1) / HEARTBEAT_PUBLISH_RATE_HZ,
                                     self.heartbeatPublishCb)

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

    def setObstaclesNamespaceCb(self, msg):
        self.msg_if.pub_info(str(msg))
        self.connectObstacles(msg.data)
        self.publish_status()

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


    ###################
    ## Robot Interface Callbacks

    def setTeamNumberCb(self, msg):
        self.msg_if.pub_info(str(msg))
        team_number = int(msg.data)
        if team_number < 1:
            self.msg_if.pub_warn("Ignoring team number " + str(team_number) +
                                 ": must be 1 or greater")
            return
        self.team_number = team_number
        # Repoint the running client rather than restarting it: NT resolves the
        # new server itself and the connection state follows.
        if self.nt_instance is not None:
            nepi_wpilib.set_server_team(self.nt_instance, self.team_number)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('team_number', self.team_number)
            self.node_if.save_config()

    def setMotorSlotCountCb(self, msg):
        self.msg_if.pub_info(str(msg))
        slot_count = int(msg.data)
        if slot_count < 0 or slot_count > MAX_MOTOR_SLOT_COUNT:
            self.msg_if.pub_warn("Ignoring motor slot count " + str(slot_count) +
                                 ": must be 0 to " + str(MAX_MOTOR_SLOT_COUNT))
            return
        self.motor_slot_count = slot_count
        # Both per-slot lists are resized to match, so slot order stays
        # unambiguous: new slots come up unmapped and unnamed rather than
        # inheriting whatever was at that index before.
        self.motor_ids = self.resizeMotorIds(self.motor_ids, slot_count)
        self.motor_names = self.resizeMotorNames(self.motor_names, slot_count)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('motor_slot_count', self.motor_slot_count)
            self.node_if.set_param('motor_ids', self.motor_ids)
            self.node_if.set_param('motor_names', self.motor_names)
            self.node_if.save_config()

    def setMotorIdsCb(self, msg):
        self.msg_if.pub_info(str(msg))
        motor_ids = self.parseMotorIdList(msg.data)
        if motor_ids is None:
            self.msg_if.pub_warn("Ignoring motor ids '" + str(msg.data) +
                                 "': expected a comma separated list of integers")
            return
        self.motor_ids = self.resizeMotorIds(motor_ids, self.motor_slot_count)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('motor_ids', self.motor_ids)
            self.node_if.save_config()

    def setMotorNamesCb(self, msg):
        self.msg_if.pub_info(str(msg))
        motor_names = [name.strip() for name in str(msg.data).split(',')]
        if str(msg.data).strip() == '':
            motor_names = []
        self.motor_names = self.resizeMotorNames(motor_names, self.motor_slot_count)
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('motor_names', self.motor_names)
            self.node_if.save_config()

    def setRbxEnabledCb(self, msg):
        self.msg_if.pub_info(str(msg))
        self.rbx_enabled = bool(msg.data)
        if self.rbx_enabled is False:
            self.teardownRbxIF()
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('rbx_enabled', self.rbx_enabled)
            self.node_if.save_config()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            self.enabled = self.node_if.get_param('enabled')
            self.selected_option = self.node_if.get_param('selected_option')
            self.value = self.node_if.get_param('value')

            self.team_number = int(self.node_if.get_param('team_number'))
            self.motor_slot_count = int(self.node_if.get_param('motor_slot_count'))
            self.motor_ids = self.resizeMotorIds(self.node_if.get_param('motor_ids'),
                                                 self.motor_slot_count)
            self.motor_names = self.resizeMotorNames(self.node_if.get_param('motor_names'),
                                                     self.motor_slot_count)
            self.rbx_enabled = bool(self.node_if.get_param('rbx_enabled'))
        if do_updates:
            if self.nt_instance is not None:
                nepi_wpilib.set_server_team(self.nt_instance, self.team_number)
            if self.rbx_enabled is False:
                self.teardownRbxIF()
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

    def setDemoBool(self, value):
        """Set the Example Controls demo_bool from heartbeat state."""

        if self.example_controls_if is None:
            return

        value = bool(value)

        # Avoid spamming the ControlsIF with the same value every timer tick.
        if self.demo_bool_value == value:
            return

        try:
            self.example_controls_if.set_control_value(
                "demo_bool",
                value
            )
            self.demo_bool_value = value

        except Exception as e:
            self.msg_if.pub_warn(
                "Failed to set demo_bool from heartbeat: " + str(e),
                throttle_s=10.0
            )

    # Fully-qualified namespace of the example ControlsIF.
    #
    # Built from self.node_namespace rather than read off
    # ControlsIF.get_namespace(). That method returns
    # create_namespace(node_NAME, controls_name) -- 'app_wpilib_if/example_controls',
    # with no leading slash. It resolves correctly where the IF itself uses it, since
    # rospy resolves a relative name against the node's parent namespace, but this
    # app publishes the value for the RUI, which appends '/status' and hands it to
    # rosbridge -- where a name with no leading slash resolves at the global root
    # instead of under /<prefix>/<device_id>.
    def getExampleControlsNamespace(self):
        return nepi_sdk.create_namespace(self.node_namespace, EXAMPLE_CONTROLS_NAME)

    def getExampleControlsReadyState(self):
        if self.example_controls_if is None:
            return False
        return self.example_controls_if.get_controls_ready_state()


    ###################
    ## NetworkTables Client
    #
    # This app node is the only NEPI process that holds a NetworkTables client,
    # and it holds exactly one. Everything NEPI sees of the robot is read here
    # and republished on standard NEPI interfaces; everything NEPI commands is
    # written from here.

    def startNetworkTables(self):
        # Non-blocking: nepi_wpilib.start_client returns as soon as the client is
        # started and NT connects on its own threads. Nothing in the ROS node's
        # main thread ever waits on the robot network.
        try:
            self.nt_instance = nepi_wpilib.start_client(team_number=self.team_number)
        except Exception as e:
            self.nt_instance = None
            self.msg_if.pub_warn("Failed to start NetworkTables client: " + str(e))
            return

        self.nt_heartbeat_entry = nepi_wpilib.get_heartbeat_entry(self.nt_instance)
        self.nt_conn_handle = nepi_wpilib.add_connection_callback(self.nt_instance,
                                                                  self.ntConnectionCb)
        self.msg_if.pub_info("NetworkTables client started for team " +
                             str(self.team_number) +
                             " (vendored ntcore: " +
                             str(nepi_wpilib.get_vendor_dir()) + ")")

    def stopNetworkTables(self):
        if self.nt_conn_handle is not None:
            nepi_wpilib.remove_connection_callback(self.nt_conn_handle)
            self.nt_conn_handle = None
        if self.nt_heartbeat_entry is not None:
            try:
                nepi_wpilib.write_boolean(self.nt_heartbeat_entry, False)
                self.nt_heartbeat_entry.close()
            except Exception:
                pass
            self.nt_heartbeat_entry = None
        if self.nt_instance is not None:
            nepi_wpilib.stop_client(self.nt_instance)
            self.nt_instance = None

    # Called from an NT listener thread, not from a ROS callback, so it only
    # stores state and republishes status.

    def ntConnectionCb(self, connected):
        self.set_connected(connected)

        if connected is False:
            self.heartbeat_waiting_for_response = False
            self.setDemoBool(False)

        self.publish_status()
    

    def heartbeatPublishCb(self, timer):
        # NEPI's half of the handshake: set the heartbeat True once a second and
        # let the RoboRIO answer it with False. The responder half
        # (nepi_wpilib.respond_to_heartbeat) blocks for 500 ms by design and is
        # therefore never called from this node -- it is the RoboRIO's job, and
        # the loopback test's RoboRIO stand-in runs it on its own thread.

        if self.nt_instance is None or self.nt_heartbeat_entry is None:
            self.heartbeat_waiting_for_response = False
            self.setDemoBool(False)
            return

        if nepi_wpilib.is_connected(self.nt_instance) is False:
            self.heartbeat_waiting_for_response = False
            self.setDemoBool(False)
            return

        nepi_wpilib.publish_heartbeat(
            self.nt_instance,
            self.nt_heartbeat_entry
        )

        self.heartbeat_waiting_for_response = True
        self.heartbeat_last_assert_time = nepi_sdk.get_time()

        # Visual proof that NEPI sent the heartbeat.
        self.setDemoBool(True)

    def heartbeatResponseCb(self):
        """Check whether the RoboRIO answered the heartbeat by writing False."""

        if self.nt_instance is None or self.nt_heartbeat_entry is None:
            self.heartbeat_waiting_for_response = False
            self.setDemoBool(False)
            return

        if nepi_wpilib.is_connected(self.nt_instance) is False:
            self.heartbeat_waiting_for_response = False
            self.setDemoBool(False)
            return

        if self.heartbeat_waiting_for_response is False:
            return

        try:
            heartbeat_value = nepi_wpilib.read_boolean(self.nt_heartbeat_entry)

        except Exception as e:
            self.msg_if.pub_warn(
                "Failed to read heartbeat response: " + str(e),
                throttle_s=10.0
            )
            return

        if heartbeat_value is False:
            self.heartbeat_waiting_for_response = False
            self.heartbeat_last_response_time = nepi_sdk.get_time()

            # Visual proof that RoboRIO answered.
            self.setDemoBool(False)
        

    def ntPollCb(self, timer):
        # One read of every input group per tick into the in-memory caches. The
        # RBX device, the motors producer and the status publisher all read those
        # caches, so NetworkTables is touched at exactly this one rate no matter
        # how many consumers there are.
        if self.nt_instance is None:
            self.heartbeat_waiting_for_response = False
            self.setDemoBool(False)
            return

        # Check whether RoboRIO answered the heartbeat by writing False.
        # This runs at NT_POLL_RATE_HZ, currently 10 Hz, so it is fast enough to
        # catch the RoboRIO's ~500 ms heartbeat response.
        self.heartbeatResponseCb()

        try:
            self.motor_feedback_dict = nepi_wpilib.read_all_motor_feedback(self.nt_instance)
            self.nt_motor_ids = sorted(self.motor_feedback_dict.keys())
            self.position_dict = nepi_wpilib.read_robot_position(self.nt_instance)
            self.velocity_dict = nepi_wpilib.read_robot_velocity(self.nt_instance)
            self.orientation_dict = nepi_wpilib.read_robot_orientation(self.nt_instance)
            self.rbx_feedback_dict = nepi_wpilib.read_rbx_feedback(self.nt_instance)

        except Exception as e:
            self.msg_if.pub_warn(
                "NetworkTables poll failed: " + str(e),
                throttle_s=10.0
            )
            return

        self.logFirstGroups()

        # Build the RBX device once the operator has enabled it and the RoboRIO
        # has said what it can do.
        self.updateRbxIF()

    # First dict per input group is logged exactly once, matching what the
    # connect-IF callbacks above do for their data products.
    def logFirstGroups(self):
        if self.got_first_motor_feedback is False and len(self.motor_feedback_dict) > 0:
            self.got_first_motor_feedback = True
            self.msg_if.pub_info("Motor feedback first data: " + str(self.motor_feedback_dict))
        if self.got_first_position is False and self.isGroupLive(self.position_dict):
            self.got_first_position = True
            self.msg_if.pub_info("Robot position first data: " + str(self.position_dict))
        if self.got_first_velocity is False and self.isGroupLive(self.velocity_dict):
            self.got_first_velocity = True
            self.msg_if.pub_info("Robot velocity first data: " + str(self.velocity_dict))
        if self.got_first_orientation is False and self.isGroupLive(self.orientation_dict):
            self.got_first_orientation = True
            self.msg_if.pub_info("Robot orientation first data: " + str(self.orientation_dict))
        if self.got_first_rbx_feedback is False and self.hasRbxFeedback():
            self.got_first_rbx_feedback = True
            self.msg_if.pub_info("RBX feedback first data: " + str(self.rbx_feedback_dict))

    # A group counts as live when the robot network is up, the group's own valid
    # flag is set, and its last observed change is inside NAVPOSE_STALE_SEC.
    # All three are required: an absent group reads back as its defaults, so
    # neither the valid flag nor the age is sufficient on its own.
    def isGroupLive(self, group_dict, stale_sec=NAVPOSE_STALE_SEC):
        if group_dict is None:
            return False
        if nepi_wpilib.is_connected(self.nt_instance) is False:
            return False
        if bool(group_dict.get('valid', False)) is False:
            return False
        return float(group_dict.get('age_s', stale_sec + 1.0)) <= stale_sec

    def hasRbxFeedback(self):
        # The RBX Feedback group defines no valid flag, so liveness is the
        # connection plus a timestamp that has actually been stamped.
        if self.rbx_feedback_dict is None:
            return False
        if nepi_wpilib.is_connected(self.nt_instance) is False:
            return False
        return float(self.rbx_feedback_dict.get('timestamp', 0.0)) > 0.0


    ###################
    ## Motor Slots
    #
    # The operator-facing model is an ordered list of slots, each holding one
    # RoboRIO motor_id. Slot order is the NEPI motor index: it selects the
    # channel for MotorControl.motor_ind and it names motor_0, motor_1, ... in
    # MotorsStatus. See docs/WPILIB_IF_DESIGN.md, Decision 2.

    def getFactoryMotorIds(self):
        return [UNMAPPED_MOTOR_ID] * FACTORY_MOTOR_SLOT_COUNT

    def getFactoryMotorNames(self):
        return [''] * FACTORY_MOTOR_SLOT_COUNT

    def resizeMotorIds(self, motor_ids, slot_count):
        if not isinstance(motor_ids, list):
            motor_ids = []
        resized = [UNMAPPED_MOTOR_ID] * slot_count
        for slot in range(min(slot_count, len(motor_ids))):
            try:
                resized[slot] = int(motor_ids[slot])
            except Exception:
                resized[slot] = UNMAPPED_MOTOR_ID
        return resized

    def resizeMotorNames(self, motor_names, slot_count):
        if not isinstance(motor_names, list):
            motor_names = []
        resized = [''] * slot_count
        for slot in range(min(slot_count, len(motor_names))):
            resized[slot] = str(motor_names[slot])
        return resized

    def parseMotorIdList(self, text):
        # Returns None for a malformed list so the caller can reject the whole
        # message rather than silently accepting a partly parsed mapping.
        text = str(text).strip()
        if text == '':
            return []
        motor_ids = []
        for field in text.split(','):
            field = field.strip()
            if field == '':
                motor_ids.append(UNMAPPED_MOTOR_ID)
                continue
            try:
                motor_ids.append(int(field))
            except Exception:
                return None
        return motor_ids

    def getNepiMotorName(self, slot):
        return 'motor_' + str(slot)

    # The one place slot configuration and live feedback are joined. Everything
    # motor-shaped -- MotorsStatus, the app status readout, the RBX motor
    # channels -- reads slots from here.
    def get_motor_slots(self):
        """Return the ordered motor slot list, one dict per slot.

        Each entry carries the slot index, its configured RoboRIO motor_id and
        display name, the NEPI motor name (motor_0, motor_1, ...), whether the
        mapped motor has ever been seen on NetworkTables, whether its feedback
        is currently fresh, and the raw feedback dict when there is one.

        Returns:
            list: One dict per slot, in slot order.
        """
        slots = []
        for slot in range(self.motor_slot_count):
            motor_id = UNMAPPED_MOTOR_ID
            if slot < len(self.motor_ids):
                motor_id = int(self.motor_ids[slot])
            display_name = ''
            if slot < len(self.motor_names):
                display_name = str(self.motor_names[slot])

            feedback = None
            if motor_id >= 0:
                feedback = self.motor_feedback_dict.get(motor_id)

            seen = feedback is not None
            fresh = False
            if seen is True and nepi_wpilib.is_connected(self.nt_instance) is True:
                fresh = float(feedback.get('age_s', MOTOR_STALE_SEC + 1.0)) <= MOTOR_STALE_SEC

            slots.append(dict(slot=slot,
                              motor_id=motor_id,
                              display_name=display_name,
                              nepi_motor_name=self.getNepiMotorName(slot),
                              seen=seen,
                              fresh=fresh,
                              feedback=feedback))
        return slots

    # Resolve a MotorsStatus/MotorCommand motor name back to its slot.
    def getSlotByName(self, motor_name):
        for slot_dict in self.get_motor_slots():
            if slot_dict['nepi_motor_name'] == str(motor_name):
                return slot_dict
        return None


    ###################
    ## Motor Feedback Out
    #
    # Motor feedback goes out on the standard NEPI motor contract, not a bespoke
    # one: MotorsDeviceIF publishes nepi_interfaces/MotorsStatus on the dedicated
    # motor_status topic, which is exactly what the connect-side
    # ConnectMotorsDeviceIF discovers devices by.
    #
    # RoboRIO field -> MotorStatus field:
    #   measured_output -> motor_speed_ratio (magnitude) and motor_dir (sign)
    #   position        -> motor_position
    #   velocity        -> motor_speed
    #   (derived)       -> motor_enable, from mapped AND seen AND fresh
    #   (slot order)    -> motor_name, motor_0 / motor_1 / ...
    #   timestamp       -> MotorsStatus.timestamp is stamped by MotorsDeviceIF
    #                      with NEPI time; the RoboRIO's own per-motor stamp has
    #                      no MotorStatus home and rides in WpilibMotorFeedback.
    #
    # WITH NO MotorStatus HOME, carried in WpilibMotorFeedback[] on this app's
    # own status message instead of being dropped: control_mode,
    # commanded_output, current_amps, the RoboRIO's own motor_name, the RoboRIO's
    # per-motor timestamp, and the RoboRIO motor_id itself.
    #
    # motor_max_speed is reported 0.0: the RoboRIO contract has no maximum-speed
    # field, so there is no value that motor_speed_ratio = 1.0 maps to.
    #
    # UNITS. position and velocity are passed through exactly as the RoboRIO
    # reports them. MotorStatus documents its own fields as degrees and
    # degrees/second, but a RoboRIO motor controller reports whatever its
    # configured native or sensor units are (rotations, ticks, RPM), and this app
    # has no field in the contract telling it which. Converting on a guess would
    # produce confidently wrong numbers; passing them through leaves them
    # correct in the robot's own units.

    def setupMotorsIF(self):
        device_info = dict(device_name=ROBOT_DEVICE_NAME,
                           path='',
                           serial_number='',
                           hw_version='',
                           sw_version='')
        try:
            self.motors_if = MotorsDeviceIF(
                device_info=device_info,
                data_source_description='motor_controller',
                data_ref_description='roborio',
                getMotorsStatusFunction=self.getMotorsStatusList,
                setSpeedFunction=self.setMotorSpeed,
                # None both: MotorCommand's direction is a signed spin
                # direction, and the RoboRIO command contract carries a 0.0-1.0
                # speed magnitude with no direction field to write it into.
                setDirectionFunction=None,
                goDirectionFunction=None,
                stopMotorFunction=self.stopMotor,
                msg_if=self.msg_if)
        except Exception as e:
            self.motors_if = None
            self.msg_if.pub_warn("Failed to start motors interface: " + str(e))

    def getMotorsStatusList(self):
        # MotorsDeviceIF turns each dict into one MotorStatus, in this order.
        motors_list = []
        for slot_dict in self.get_motor_slots():
            feedback = slot_dict['feedback']
            enabled = slot_dict['seen'] and slot_dict['fresh']

            # A slot whose motor has never been seen on NetworkTables reports as
            # not enabled with zeroed telemetry, rather than presenting stale or
            # absent values as if they were real readings.
            measured_output = 0.0
            position = 0.0
            velocity = 0.0
            if feedback is not None and enabled is True:
                measured_output = float(feedback.get('measured_output', 0.0))
                position = float(feedback.get('position', 0.0))
                velocity = float(feedback.get('velocity', 0.0))

            motor_dir = 1
            if measured_output < 0.0:
                motor_dir = -1

            motors_list.append(dict(
                motor_name=slot_dict['nepi_motor_name'],
                motor_enable=enabled,
                motor_dir=motor_dir,
                motor_max_speed=0.0,
                motor_speed_ratio=min(1.0, abs(measured_output)),
                motor_speed=velocity,
                motor_position=position))
        return motors_list

    def writeMotorCommand(self, motor_id, speed_ratio):
        # The single writer for per-motor commands, shared by the standard motor
        # command topics and by the RBX device's manual motor control.
        if self.nt_instance is None:
            self.msg_if.pub_warn("Motor command dropped: no NetworkTables client",
                                 throttle_s=10.0)
            return False
        return nepi_wpilib.write_motor_command(self.nt_instance, motor_id, speed_ratio)

    def commandMotorByName(self, motor_name, speed_ratio):
        if str(motor_name) == 'all':
            for slot_dict in self.get_motor_slots():
                if slot_dict['motor_id'] >= 0:
                    self.writeMotorCommand(slot_dict['motor_id'], speed_ratio)
            return
        slot_dict = self.getSlotByName(motor_name)
        if slot_dict is None:
            self.msg_if.pub_warn("Motor command ignored: no slot named " + str(motor_name))
            return
        if slot_dict['motor_id'] < 0:
            self.msg_if.pub_warn("Motor command ignored: slot " + str(slot_dict['slot']) +
                                 " has no RoboRIO motor_id mapped")
            return
        self.writeMotorCommand(slot_dict['motor_id'], speed_ratio)

    def setMotorSpeed(self, motor_name, speed_ratio):
        self.commandMotorByName(motor_name, speed_ratio)

    def stopMotor(self, motor_name):
        self.commandMotorByName(motor_name, 0.0)

    def getMotorFeedbackMsgs(self):
        # The RoboRIO fields with no MotorStatus home, one entry per slot.
        feedback_msgs = []
        for slot_dict in self.get_motor_slots():
            feedback = slot_dict['feedback']
            msg = WpilibMotorFeedback()
            msg.slot = slot_dict['slot']
            msg.motor_id = slot_dict['motor_id']
            msg.nepi_motor_name = slot_dict['nepi_motor_name']
            msg.display_name = slot_dict['display_name']
            msg.seen = slot_dict['seen']
            msg.fresh = slot_dict['fresh']
            if feedback is None:
                msg.roborio_motor_name = ''
                msg.control_mode = ''
                msg.commanded_output = 0.0
                msg.current_amps = 0.0
                msg.timestamp = 0.0
                msg.age_s = -1.0
            else:
                msg.roborio_motor_name = str(feedback.get('motor_name', ''))
                msg.control_mode = str(feedback.get('control_mode', ''))
                msg.commanded_output = float(feedback.get('commanded_output', 0.0))
                msg.current_amps = float(feedback.get('current_amps', 0.0))
                msg.timestamp = float(feedback.get('timestamp', 0.0))
                msg.age_s = float(feedback.get('age_s', -1.0))
            feedback_msgs.append(msg)
        return feedback_msgs


    ###################
    ## NavPose
    #
    # Robot Position, Robot Orientation and Robot Velocity are three
    # NetworkTables input groups and ONE NavPose. There is exactly one owner of
    # that pose, and it is the RBX device: RBXRobotIF publishes it through the
    # NPXDeviceIF/NavPoseIF it builds from getNavPoseCb, so this app creates no
    # NavPoseIF of its own and there is no second competing publisher. That also
    # means NavPose exists only while rbx_enabled is true -- see
    # docs/WPILIB_IF_DESIGN.md, Decision 4, for why getNavPoseCb is not optional
    # (RBXRobotIF's goto convergence math is the only consumer of the current_*
    # pose, and it is refreshed from nowhere else in this build).
    #
    # STALENESS. Each group contributes only when isGroupLive() passes for it:
    # robot network connected, the group's own valid flag set, and its last
    # observed change inside NAVPOSE_STALE_SEC. A group that fails leaves its
    # has_* flag False rather than contributing zeros, so a consumer sees the
    # component as absent instead of as a confident wrong reading. When neither
    # position nor orientation qualifies, this returns None and nothing is
    # published at all.

    def get_navpose_dict(self):
        """Return the fused NavPose dict, or None if nothing valid is available.

        Fuses the Robot Position, Robot Orientation and Robot Velocity input
        groups into one NEPI navpose dict, honouring each group's own valid flag
        and freshness. The dict is in NEPI standard frames already (ENU, WGS84),
        so no frame conversion is applied downstream.

        Returns:
            dict: A NEPI navpose dict, or None when no input group is currently
                valid and fresh.
        """
        position_live = self.isGroupLive(self.position_dict)
        orientation_live = self.isGroupLive(self.orientation_dict)
        velocity_live = self.isGroupLive(self.velocity_dict)

        if position_live is False and orientation_live is False:
            return None

        navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        # WPILib field coordinates are right-handed with z up and yaw positive
        # counter-clockwise, which is the convention NEPI calls ENU. The robot
        # has no global reference in this contract, so location, altitude and
        # depth stay absent.
        navpose_dict['frame_nav'] = 'ENU'
        navpose_dict['frame_altitude'] = 'WGS84'
        navpose_dict['frame_depth'] = 'MSL'

        if position_live is True:
            navpose_dict['has_position'] = True
            navpose_dict['time_position'] = self.getGroupTime(self.position_dict)
            navpose_dict['x_m'] = float(self.position_dict['x_m'])
            navpose_dict['y_m'] = float(self.position_dict['y_m'])
            navpose_dict['z_m'] = float(self.position_dict['z_m'])
            # heading_rad rides in the Robot Position group, and is a compass
            # heading rather than the ENU yaw the orientation group reports, so
            # it fills the navpose heading component rather than yaw.
            navpose_dict['has_heading'] = True
            navpose_dict['time_heading'] = navpose_dict['time_position']
            navpose_dict['heading_deg'] = math.degrees(float(self.position_dict['heading_rad']))

        if velocity_live is True:
            navpose_dict['x_m_per_sec'] = float(self.velocity_dict['velocity_x_mps'])
            navpose_dict['y_m_per_sec'] = float(self.velocity_dict['velocity_y_mps'])
            navpose_dict['z_m_per_sec'] = float(self.velocity_dict['velocity_z_mps'])
            navpose_dict['altitude_m_per_sec'] = 0.0
            # Ground speed along the heading, from the planar velocity.
            navpose_dict['heading_m_per_sec'] = math.sqrt(
                float(self.velocity_dict['velocity_x_mps']) ** 2 +
                float(self.velocity_dict['velocity_y_mps']) ** 2)
            navpose_dict['location_m_per_sec'] = navpose_dict['heading_m_per_sec']
            # Yaw rate appears in BOTH input groups (angular_velocity_radps here,
            # yaw_rate_radps in Robot Orientation). The orientation group is the
            # authority on orientation rates, so this only fills the field when
            # that group is not contributing.
            if orientation_live is False:
                navpose_dict['yaw_deg_per_sec'] = math.degrees(
                    float(self.velocity_dict['angular_velocity_radps']))

        if orientation_live is True:
            navpose_dict['has_orientation'] = True
            navpose_dict['time_orientation'] = self.getGroupTime(self.orientation_dict)
            navpose_dict['roll_deg'] = math.degrees(float(self.orientation_dict['roll_rad']))
            navpose_dict['pitch_deg'] = math.degrees(float(self.orientation_dict['pitch_rad']))
            navpose_dict['yaw_deg'] = math.degrees(float(self.orientation_dict['yaw_rad']))
            navpose_dict['roll_deg_per_sec'] = math.degrees(
                float(self.orientation_dict['roll_rate_radps']))
            navpose_dict['pitch_deg_per_sec'] = math.degrees(
                float(self.orientation_dict['pitch_rate_radps']))
            navpose_dict['yaw_deg_per_sec'] = math.degrees(
                float(self.orientation_dict['yaw_rate_radps']))

        return navpose_dict

    # Which clock a group's component is stamped with. The RoboRIO's own group
    # timestamp is the sample time and is preferred, but only when it is
    # plausibly in this device's clock domain -- NEPI NTP-syncs to the roboRIO on
    # an FRC network, so normally it is. A zero or wildly-off stamp falls back to
    # the local time the value was observed to change, which is always sane.
    def getGroupTime(self, group_dict):
        local_time = float(group_dict.get('time_local', nepi_sdk.get_time()))
        robot_time = float(group_dict.get('timestamp', 0.0))
        if robot_time > 0.0 and abs(local_time - robot_time) < NAVPOSE_CLOCK_SKEW_SEC:
            return robot_time
        return local_time

    def getNavPoseTopic(self):
        # The RBX device's own navpose path: RBXRobotIF hands getNavPoseCb to an
        # NPXDeviceIF built on this node's namespace, and that publishes through
        # a NavPoseIF at <npx namespace>/navpose.
        if self.rbx_if is None:
            return NONE_NAMESPACE
        return nepi_sdk.create_namespace(self.node_namespace, 'npx/navpose')


    ###################
    ## RBX Device
    #
    # One robot is one RBX device, hosted here and built entirely inside
    # wpilib_rbx_if.py so relocating it to a driver package is a file move plus a
    # transport swap. This node's whole job is to construct it when rbx_enabled
    # is true, hand it the injected callables, and tear it down when rbx_enabled
    # goes false. See docs/WPILIB_IF_DESIGN.md, Decision 3.

    # Serializes build against teardown. updateRbxIF runs on the NT poll timer
    # and setRbxEnabledCb runs on a subscriber thread, so without this a
    # disable arriving mid-construction could tear down a half-built device.
    def updateRbxIF(self):
        if self.rbx_enabled is False:
            return

        capabilities = self.getSupportedCapabilities()

        with self.rbx_lock:
            # Re-checked inside the lock: rbx_enabled may have gone false while
            # this call was waiting for it.
            if self.rbx_enabled is False:
                return

            if self.rbx_if is None:
                # Built only once the RoboRIO has said what it supports.
                # RBXRobotIF derives and CACHES its capability flags from which
                # callbacks are non-None at construction, so building before
                # supported_capabilities arrives would permanently advertise a
                # robot with no controls.
                #
                # Construction is not quick (RBXRobotIF brings up its own
                # NodeClassIF, settings, save-data, image and NPX interfaces), and
                # it runs inside this timer callback, so the NT poll pauses for
                # the duration. That is a one-time cost at enable, and nothing
                # consumes the paused telemetry until the device exists.
                if len(capabilities) == 0:
                    return
                self.buildRbxIF(capabilities)
                return

            # A changed capability list means a different robot control surface.
            # RBXRobotIF has no in-place equivalent of SimDeviceIF's
            # apply_capability_profile (the 2026-08 sanctioned exception is
            # specific to that class), so the only honest way to report new
            # capabilities is to tear down and rebuild. Compared as sets, so
            # merely reordering the advertised list does not churn the device.
            built_capabilities = self.rbx_if.get_supported_capabilities()
            if sorted(capabilities) != sorted(built_capabilities):
                self.msg_if.pub_warn("RoboRIO capabilities changed from " +
                                     str(built_capabilities) +
                                     " to " + str(capabilities) + ": rebuilding RBX device")
                self.tearDownRbx()
                self.buildRbxIF(capabilities)

    def buildRbxIF(self, capabilities):
        try:
            self.rbx_if = WpilibRbxIF(
                device_name=ROBOT_DEVICE_NAME,
                getMotorSlotsFunction=self.get_motor_slots,
                setMotorCommandFunction=self.writeMotorCommand,
                getNavPoseFunction=self.get_navpose_dict,
                getRbxFeedbackFunction=self.getRbxFeedbackDict,
                getConnectedFunction=self.get_connected,
                writeCommandRequestFunction=self.writeRbxCommandRequest,
                command_types=RBX_COMMAND_TYPES,
                supported_capabilities=capabilities,
                log_name='rbx',
                msg_if=self.msg_if)
        except Exception as e:
            self.rbx_if = None
            self.msg_if.pub_warn("Failed to build RBX device: " + str(e))
            return
        self.msg_if.pub_info("RBX device enabled at " + str(self.rbx_if.get_namespace()))
        self.publish_status()

    def teardownRbxIF(self):
        with self.rbx_lock:
            self.tearDownRbx()

    # Caller must hold rbx_lock.
    def tearDownRbx(self):
        if self.rbx_if is None:
            return
        self.rbx_if.shutdown()
        self.rbx_if = None
        self.msg_if.pub_warn("RBX device disabled")

    def getSupportedCapabilities(self):
        if self.hasRbxFeedback() is False:
            return []
        return [str(c) for c in self.rbx_feedback_dict.get('supported_capabilities', [])]

    def getRbxFeedbackDict(self):
        if self.hasRbxFeedback() is False:
            return None
        return self.rbx_feedback_dict

    def writeRbxCommandRequest(self, request_id, command_type, chassis_speeds,
                               target_pose, named_action):
        if self.nt_instance is None:
            self.msg_if.pub_warn("RBX command request dropped: no NetworkTables client")
            return False
        return nepi_wpilib.write_rbx_command_request(self.nt_instance,
                                                    request_id,
                                                    command_type,
                                                    chassis_speeds=chassis_speeds,
                                                    target_pose=target_pose,
                                                    named_action=named_action)

    def getRbxNamespace(self):
        if self.rbx_if is None:
            return NONE_NAMESPACE
        return self.rbx_if.get_namespace()

    def getRbxReadyState(self):
        if self.rbx_if is None:
            return False
        return self.rbx_if.get_device_ready_state()


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
                        dataCB = self.detectionsConnectCb,
                        msg_if = self.msg_if)

        self.navpose_if = ConnectNavPoseIF(
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.navposeConnectCb,
                        msg_if = self.msg_if)

    # Point the obstacles connect path at an obstacles app namespace, or tear it
    # down. Called from setObstaclesNamespaceCb with whatever the RUI selector
    # sent. The previous IF is always unregistered first, so a selection change
    # does not leave the old instance's publishers and subscribers registered.
    # An empty or 'None' selection is a disconnect: nothing is constructed and
    # the node is left in a valid state with no obstacles connection.
    #
    # ConnectObstaclesIF only advertises against the namespace -- it does not
    # require the app to be live -- but construction is guarded anyway so a stale
    # or mistyped namespace cannot take the node down.
    def connectObstacles(self, namespace):
        if self.obstacles_if is not None:
            self.obstacles_if.unregister()
            self.obstacles_if = None
        self.obstacles_dict = None
        self.obstacles_status = None
        self.got_first_obstacles = False

        if namespace is None or namespace == "" or namespace == NONE_NAMESPACE:
            self.obstacles_namespace = NONE_NAMESPACE
            self.msg_if.pub_info("Obstacles connection cleared")
            return

        self.obstacles_namespace = namespace
        try:
            self.obstacles_if = ConnectObstaclesIF(
                            namespace = namespace,
                            dataCB = self.obstaclesConnectCb)
        except Exception as e:
            self.obstacles_if = None
            self.msg_if.pub_warn("Failed to connect obstacles app at " +
                                 str(namespace) + ": " + str(e))


    ###################
    ## Connect IF First-Connection Callbacks
    #
    # Each connect IF invokes its dataCB with a single data dict. The
    # callback stores that dict and the IF's current status message (via
    # get_status_msg()) on every invocation. On the FIRST invocation per IF it
    # also logs both, then sets the got_first flag so it logs only once. The
    # Obstacles app publishes no data product this consumer subscribes to, so
    # ConnectObstaclesIF fires its dataCB with the status dict instead.

    def detectionsConnectCb(self, data_dict):
        self.detections_dict = data_dict
        self.detections_status = self.detections_if.get_status_msg()
        if self.got_first_detections is True:
            return
        self.got_first_detections = True
        self.msg_if.pub_info("Detections first-connection data dict: " + str(self.detections_dict))
        self.msg_if.pub_info("Detections first-connection status message: " + str(self.detections_status))

    def obstaclesConnectCb(self, data_dict):
        self.obstacles_dict = data_dict
        if self.obstacles_if is not None:
            self.obstacles_status = self.obstacles_if.get_status_msg()
        if self.got_first_obstacles is True:
            return
        self.got_first_obstacles = True
        self.msg_if.pub_info("Obstacles first-connection data dict: " + str(self.obstacles_dict))
        self.msg_if.pub_info("Obstacles first-connection status message: " + str(self.obstacles_status))

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
        # The connection state is read from the live NetworkTables client here
        # as well as being pushed by the NT connection listener (ntConnectionCb).
        # Both, deliberately: the listener makes a state change visible
        # immediately, and this poll makes the reported value correct even if a
        # listener event is ever missed or the listener could not be registered.
        #
        # No separate timer for it: STATUS_PUBLISH_RATE_HZ is 1.0, so this
        # callback already runs at exactly one hertz. A second timer would
        # either double-publish the status message or duplicate this schedule.
        if self.nt_instance is None:
            self.set_connected(False)
        else:
            self.set_connected(nepi_wpilib.is_connected(self.nt_instance))

        self.publish_status()
        

    def set_connected(self, connected):
        """Set the app's reported robot network connection state.

        Driven from the real WPILib NetworkTables connection state: pushed by the
        NT connection listener on every change, and re-read once a second from
        the live client by statusPublishCb.

        Args:
            connected (bool): True if the app is connected to the robot network.
        """
        self.connected = connected

    def get_connected(self):
        """Return the app's reported robot network connection state.

        Returns:
            bool: True while the NetworkTables client has a live connection to
                the RoboRIO.
        """
        return self.connected

    def publish_status(self):
        status_msg = NepiAppWpilibIFStatus()
        status_msg.enabled = self.enabled
        status_msg.options = self.options
        status_msg.selected_option = self.selected_option
        status_msg.value = self.value
        status_msg.connected = self.connected
        status_msg.selected_obstacles_namespace = self.obstacles_namespace

        # Robot interface configuration and live state
        status_msg.team_number = self.team_number
        status_msg.motor_slot_count = self.motor_slot_count
        status_msg.motor_ids = self.motor_ids
        status_msg.motor_names = self.motor_names
        status_msg.nt_motor_ids = self.nt_motor_ids
        status_msg.motor_feedback = self.getMotorFeedbackMsgs()
        status_msg.motors_namespace = self.getMotorsNamespace()

        status_msg.rbx_enabled = self.rbx_enabled
        status_msg.rbx_namespace = self.getRbxNamespace()
        status_msg.rbx_ready = self.getRbxReadyState()

        status_msg.navpose_topic = self.getNavPoseTopic()
        status_msg.navpose_valid = self.get_navpose_dict() is not None

        # RBX Feedback group, as last read
        feedback = self.rbx_feedback_dict
        if feedback is None:
            feedback = dict()
        status_msg.supported_capabilities = [str(c) for c in
                                            feedback.get('supported_capabilities', [])]
        status_msg.active_request_id = str(feedback.get('active_request_id', ''))
        status_msg.active_request_type = str(feedback.get('active_request_type', ''))
        status_msg.request_status = str(feedback.get('request_status', ''))
        status_msg.status_message = str(feedback.get('status_message', ''))
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

    def getMotorsNamespace(self):
        if self.motors_if is None:
            return NONE_NAMESPACE
        return self.motors_if.get_namespace()

    def cleanup_actions(self):
        self.msg_if.pub_info("WPILIB: Shutting down: Executing script cleanup actions")
        # The RBX device first, so its command surface goes before the transport
        # it writes through, then the one NetworkTables client this process owns.
        self.teardownRbxIF()
        self.stopNetworkTables()


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiWpilibApp()
