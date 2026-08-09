#!/usr/bin/env python
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

import os
import time
import copy
import importlib

from std_msgs.msg import Bool, Empty, String, Float32

from nepi_app_obstacles.msg import NepiAppObstaclesStatus

from nepi_sdk import nepi_sdk

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import ControlsIF

from nepi_api.connect_data_if import ConnectDepthMapIF
from nepi_api.connect_data_if import ConnectColorImageIF
from nepi_api.connect_data_if import ConnectNavPoseIF

from nepi_api.connect_targets_if import ConnectTargetsIF

import nepi_obstacles


#########################################
# Factory Control Values
FACTORY_ENABLED = False
FACTORY_SELECTED_OPTION = "None"
FACTORY_VALUE = 0.0
FACTORY_OPTIONS = ["None", "Option_A", "Option_B", "Option_C"]

STATUS_PUBLISH_RATE_HZ = 1.0

# Connect namespace names, one per connect IF. Each name is the <app>/<name>
# connect namespace its IF owns and the matching Nepi_IF_Connect* component in
# NepiAppObstacles.js binds to, so all five are named here to keep that
# node-to-RUI pairing greppable from one place. Depth map, targets and NavPose
# match their class defaults; the two image IFs are bare ConnectBaseImageIF
# subclasses whose inherited default is the generic 'data_connect'
# (connect_data_if.CONNECT_NAME), which says nothing about what they carry, so
# they are named explicitly. The two image names belong to node-driven IFs with
# no selector of their own -- the RUI viewers read their topics off this app's
# status message, not off these connect namespaces.
DEPTH_MAP_CONNECT_NAME = "depth_map_connect"
COLOR_IMAGE_CONNECT_NAME = "color_image_connect"
TARGETS_CONNECT_NAME = "targets_connect"
TARGETS_IMAGE_CONNECT_NAME = "targets_image_connect"
NAVPOSE_CONNECT_NAME = "navpose_connect"

# Subtopic a depth map's own image is published on, one level under the depth map
# itself: DepthMapIF with pub_image=True creates a DepthMapImageIF on its own
# namespace, and that IF's data product is 'depth_map_image'. Same join
# nepi_app_stereo_cam makes with DEPTH_IMAGE_SUBTOPIC. Used by
# getDepthMapImageTopic() -- see the comment there for why the depth map's own
# DepthMapStatus.image_topic is not taken at face value.
DEPTH_MAP_IMAGE_SUBTOPIC = 'depth_map_image'

# Subtopic the targets overlay image is published on. The detector image pub node
# (nepi_ai_detector_img_pub_node.py) creates one ColorImageIF per image source for
# the 'detections_image' data product and a second for 'targets_image', both on the
# same <img_source_dir>, so the targets image is a SIBLING of the detections image
# the detector reports in ProcessStatus.imaging_pub_topics. Same shape as
# DEPTH_MAP_IMAGE_SUBTOPIC and for the same reason: the status message points at
# the right directory, and the subtopic names the product wanted inside it.
TARGETS_IMAGE_SUBTOPIC = 'targets_image'

# ControlsIF names. Each is the <node>/<controls_name> namespace one ControlsIF
# owns and one Nepi_IF_Controls component in NepiAppObstacles.js binds to, the
# same node-to-RUI pairing the connect names above keep greppable from one place.
#
# THREE instances, not one. nepi_obstacles.PROCESSES_DICT holds two named control
# sets and only one is active at a time, so each process gets its own ControlsIF
# named after it and the operator is shown just the active one. The alternative --
# a single union set with per-process key prefixes and the inactive half hidden --
# cannot work: hiding a control at runtime goes through
# nepi_controls.set_control_hidden(), which does `hidden = str(hidden)` and so
# writes 'True'/'False' into a field nepi_interfaces/Control declares bool. That
# string breaks the ControlsStatus publish and never satisfies
# Nepi_IF_Controls.js's `control_msg.hidden === true` test either. Per-control
# 'hidden' works only as authored in the init dict, which is a startup value, not
# a runtime one.
#
# The process SELECTOR is not a control. It is this app's own state on this app's
# own topics -- set_selected_process (String) and reload_processes (Empty), read
# back from the status message's selected_process / available_processes -- exactly
# as nepi_app_stereo_cam does it, and rendered by NepiAppObstacles.js as a native
# dropdown plus a Reload Processes button.
#
# It was a Selection control in an app-level ControlsIF before, which read well but
# does not survive contact with the RUI: a reload trigger is unreachable there
# (Nepi_IF_Controls.js sends UpdateString to a topic ControlsIF subscribes as
# UpdateTrigger, and Store.js has no sendUpdateTriggerMsg), and which process runs
# is app state that outlives any one control set rather than a control of either
# process. Keeping it on a plain topic also means the selector still works when a
# ControlsIF does not.
#
# EXAMPLE_CONTROLS_NAME below is one more instance and the exception to everything
# above: it belongs to no process and drives nothing in this app. It is a copy of
# nepi_app_controls_sandbox's demonstration control set, mounted at the bottom of
# this app's RUI column as a live example of the controls pipeline -- see
# setupExampleControlsIF(). Unlike the per-process sets it is always mounted,
# because there is no active/inactive question about it.
EXAMPLE_CONTROLS_NAME = "example_controls"

# Rate the selected process function is called at. Deliberately independent of
# STATUS_PUBLISH_RATE_HZ and of the connect IF data rates: the depth map may
# arrive at video rate, and a process cycle per frame would tie obstacle
# processing cost to the camera rather than to what the operator asked for.
PROCESS_UPDATE_RATE_HZ = 5.0

#########################################
# Node Class
#########################################

class NepiObstaclesApp(object):

    enabled = FACTORY_ENABLED
    selected_option = FACTORY_SELECTED_OPTION
    value = FACTORY_VALUE
    options = FACTORY_OPTIONS

    node_if = None

    depth_map_if = None
    color_image_if = None
    targets_if = None
    targets_image_if = None
    navpose_if = None

    # One ControlsIF per entry in nepi_obstacles.PROCESSES_DICT, keyed by process
    # name. There is no app-level ControlsIF -- see the selector comment above.
    process_controls_ifs = None

    # The example control set copied from nepi_app_controls_sandbox. Not a process
    # control set and not keyed by process. See setupExampleControlsIF().
    example_controls_if = None

    available_processes = list(nepi_obstacles.PROCESSES_DICT.keys())
    selected_process = nepi_obstacles.DEFAULT_PROCESS

    # False while reload_processes is swapping the module out, which is what
    # disables the RUI dropdown for the duration. Same flag stereo_cam publishes.
    process_ready = True

    # Runtime data dict handed to the selected process function each cycle and
    # handed back by it, so a process can carry state between cycles the way
    # nepi_stab_pt's do.
    process_data_dict = nepi_obstacles.get_blank_data_dict()

    # Set once the first time a process cycle had to be skipped because a
    # ControlsIF reported a None controls dict. See getControlsValues() for
    # what puts an IF in that state; logged once rather than at the cycle rate.
    logged_controls_dict_none = False

    # Per-IF first-connection flags. Each connect IF's callback logs once on its
    # first invocation, then sets its flag.
    got_first_depth_map = False
    got_first_color_image = False
    got_first_targets = False
    got_first_targets_image = False
    got_first_navpose = False

    # Latest data dict and status msg per connect IF. Each callback stores both
    # on every invocation, so the rest of the app reads the most recent values
    # from here rather than re-querying the IF.
    depth_map_dict = None
    depth_map_status = None
    color_image_dict = None
    color_image_status = None
    targets_dict = None
    targets_status = None
    targets_image_dict = None
    targets_image_status = None
    navpose_dict = None
    navpose_status = None

    # Set once the first time getTargetsImageTopic() has to fall back to
    # imaging_pub_topics[0] because no entry matched the derived color image. The
    # fallback is a real choice the operator may want to know about, but it recurs
    # on every derivation, so it is logged once rather than every cycle.
    logged_targets_image_fallback = False

    # Source topics as last published, keyed by status message field name. Each
    # connect callback compares the live set against this so a selection change
    # reaches the RUI at the callback rate instead of waiting out a full status
    # timer period.
    last_source_topics = None

    DEFAULT_NODE_NAME = "app_obstacles"

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
        self.available_processes = list(nepi_obstacles.PROCESSES_DICT.keys())
        self.selected_process = nepi_obstacles.DEFAULT_PROCESS
        self.process_data_dict = nepi_obstacles.get_blank_data_dict()
        self.process_controls_ifs = dict()

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
            'selected_process': {
                'namespace': self.node_namespace,
                'factory_val': self.selected_process
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': NepiAppObstaclesStatus,
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
            # Process selector, stereo_cam's two topics. String carries the process
            # NAME rather than a menu index, so a PROCESSES_DICT that gains or
            # reorders an entry cannot silently point the operator at a different
            # process than the one they picked.
            'set_selected_process': {
                'namespace': self.node_namespace,
                'topic': 'set_selected_process',
                'msg': String,
                'qsize': 10,
                'callback': self.setSelectedProcessCb,
                'callback_args': ()
            },
            'reload_processes': {
                'namespace': self.node_namespace,
                'topic': 'reload_processes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.reloadProcessesCb,
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
        # Controls. Built after the app's NodeClassIF and before initCb, so the
        # first status publish can already report every controls namespace.
        self.setupControlsIFs()

        ##############################
        # Surface the source selectors this app's RUI page renders. Each connect
        # IF owns the <app>/<connect_name> connect namespace the matching
        # Nepi_IF_Connect* component binds to. The three source IFs are built with
        # show_selector=True and the other show_* flags False -- this page selects
        # sources, it does not drive them. The two image IFs are built with every
        # show_* flag False and their selections driven from this node, because an
        # image belongs to the source it came from rather than to the operator.
        self.setupInterfaceIFs()

        ##############################
        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)
        nepi_sdk.start_timer_process(float(1) / PROCESS_UPDATE_RATE_HZ, self.processUpdateCb)

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


    ###################
    ## Process Selector

    def setSelectedProcess(self, process_name):
        if process_name not in nepi_obstacles.PROCESSES_DICT.keys():
            self.msg_if.pub_warn("Ignoring unknown process: " + str(process_name))
            return False
        if process_name == self.selected_process:
            return False
        self.selected_process = process_name
        # Which process's controls the operator SEES is decided by the RUI mounting
        # one Nepi_IF_Controls on active_controls_namespace, published below. This
        # call is the same statement on the wire; see applyActiveProcessVisibility().
        self.applyActiveProcessVisibility()
        self.msg_if.pub_info("Obstacle process changed to: " + str(self.selected_process))
        return True

    def setSelectedProcessCb(self, msg):
        if self.setSelectedProcess(msg.data):
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('selected_process', self.selected_process)
                self.node_if.save_config()

    # Re-import nepi_obstacles so edits to its process registry are picked up
    # without restarting the node (stereo_cam's reloadProcessesCb, which took the
    # pattern from pan_tilt_auto).
    #
    # A reload can ADD a process -- a new PROCESSES_DICT entry gets its own
    # ControlsIF here. It cannot change or remove the controls of a process that
    # already has one: releasing a ControlsIF means ControlsIF.unregister(), whose
    # first line calls a self.unsubscribe_topic() that nepi_api/system_if.py never
    # defines, so it raises AttributeError before releasing anything. Editing an
    # existing process's control set still needs a node restart, and a process
    # dropped from the module keeps its namespace until then -- it just stops being
    # offered in available_processes.
    def reloadProcessesCb(self, msg):
        self.process_ready = False
        self.publish_status()
        try:
            importlib.reload(nepi_obstacles)
            self.available_processes = list(nepi_obstacles.PROCESSES_DICT.keys())
            if len(self.available_processes) == 0:
                self.msg_if.pub_warn("Reloaded nepi_obstacles registers no processes")
            else:
                if self.selected_process not in self.available_processes:
                    self.selected_process = self.available_processes[0]
                self.setupProcessControlsIFs()
                self.applyActiveProcessVisibility()
                self.msg_if.pub_info("Obstacle processes reloaded: " + str(self.available_processes))
            self.process_ready = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to reload nepi_obstacles module: " + str(e))
            self.process_ready = True
        self.publish_status()


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        if self.node_if is not None:
            self.enabled = self.node_if.get_param('enabled')
            self.selected_option = self.node_if.get_param('selected_option')
            self.value = self.node_if.get_param('value')
            # The selector is this app's param, so a restart restores the process the
            # operator last picked. A param naming a process the module no longer
            # registers falls back to the module default rather than leaving the app
            # pointed at nothing.
            selected_process = self.node_if.get_param('selected_process')
            if selected_process in self.available_processes:
                self.selected_process = selected_process
            else:
                self.selected_process = nepi_obstacles.DEFAULT_PROCESS
        # No-op on the first pass: NodeClassIF fires this callback while it is being
        # constructed, before setupControlsIFs() has run.
        self.applyActiveProcessVisibility()
        if do_updates:
            pass
        self.publish_status()

    def resetCb(self, do_updates=True):
        self.msg_if.pub_warn("Resetting")
        self.resetControlsIFs(factory=False)
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        self.resetControlsIFs(factory=True)
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    # Route the app's reset paths into every ControlsIF it owns, as the sandbox app
    # does.
    #
    # Both routes fall through to ControlsIF.init(), which reloads each IF's controls
    # from its own param. That used to wipe the dict on every reset -- see
    # getControlsValues() for the defect and where it was fixed.
    def resetControlsIFs(self, factory=False):
        controls_ifs = []
        if self.process_controls_ifs is not None:
            for process_name in self.process_controls_ifs.keys():
                controls_ifs.append(self.process_controls_ifs[process_name])
        if self.example_controls_if is not None:
            controls_ifs.append(self.example_controls_if)
        for controls_if in controls_ifs:
            if controls_if is None:
                continue
            if factory == True:
                controls_if.factory_reset()
            else:
                controls_if.reset()


    ###################
    ## Controls IF Setup

    # One ControlsIF per process in nepi_obstacles.PROCESSES_DICT, named after that
    # process. The process selector is not among them -- it is this app's own topic
    # pair, for the reasons given where those topics are declared.
    #
    # node_if is left as None on every instance so each builds and owns its own
    # NodeClassIF -- the current device-IF convention, and what the sandbox app
    # does. Every registry and param key a ControlsIF creates is already prefixed
    # from its own controls_name (system_if.py builds node_if_prefix that way), so
    # several instances in one node cannot overwrite each other's entries, which is
    # the collision the 2026-07 DECISION LOG entry warns about.
    def setupControlsIFs(self):
        self.setupProcessControlsIFs()
        self.setupExampleControlsIF()
        self.applyActiveProcessVisibility()

    # One ControlsIF per process that does not have one yet. Called at startup and
    # again on reload_processes, so it must be idempotent: a process already holding
    # an IF keeps it, because releasing one is not possible (see reloadProcessesCb).
    def setupProcessControlsIFs(self):
        if self.process_controls_ifs is None:
            self.process_controls_ifs = dict()
        for process_name in self.available_processes:
            if process_name in self.process_controls_ifs.keys():
                continue
            controls_if = ControlsIF(
                        controls_name = process_name,
                        controls_display_name = process_name,
                        controls_description = 'Controls for obstacle process ' + str(process_name),
                        controls_init_dict = nepi_obstacles.PROCESSES_DICT[process_name]['default_controls_dict'],
                        controls_updated_callback = self.makeProcessControlsUpdatedCb(process_name),
                        show_controls = True,
                        has_show_control = False,
                        log_name = process_name,
                        msg_if = self.msg_if)
            controls_if.wait_for_controls_ready()
            self.process_controls_ifs[process_name] = controls_if

    # The example control set: a copy of nepi_app_controls_sandbox's demonstration
    # controls, rendered as the Example Controls box at the bottom of this app's RUI
    # column. It drives nothing here -- the values are logged when they change and
    # read by nothing else, so the widgets, persistence and update callbacks behave
    # exactly as they do on the sandbox page.
    #
    # Kept out of process_controls_ifs on purpose: it is not a process control set,
    # so it must not be reachable through getControlsNamespace(selected_process),
    # counted in getControlsReadyState(), or touched by
    # applyActiveProcessVisibility(). It joins the per-process IFs only in
    # resetControlsIFs(), where every IF the app owns is reset alike.
    #
    # Idempotent for the same reason setupProcessControlsIFs() is: setupControlsIFs()
    # runs again on reload_processes, and releasing a ControlsIF is not possible.
    def setupExampleControlsIF(self):
        if self.example_controls_if is not None:
            return
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
    # demo_floats_slider is carried for fidelity with the sandbox but does not reach
    # the RUI: nepi_controls' FloatSliders branch references an undefined name and
    # silently drops the control. It is dropped on the sandbox page too.
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

    def getExampleControlsReadyState(self):
        if self.example_controls_if is None:
            return False
        return self.example_controls_if.get_controls_ready_state()

    # Fully-qualified namespace of one of this app's ControlsIF instances.
    #
    # Built from self.node_namespace rather than read off
    # ControlsIF.get_namespace(). That method returns
    # create_namespace(node_NAME, controls_name) -- 'app_obstacles/controls', with
    # no leading slash, because nepi_sdk.get_full_namespace() normalizes a
    # namespace but never prepends the base. It resolves correctly where the IF
    # itself uses it, since rospy resolves a relative name against the node's
    # parent namespace, but this app publishes the value for the RUI, which
    # appends '/status' and hands it to rosbridge -- where a name with no leading
    # slash resolves at the global root instead of under /<prefix>/<device_id>.
    # What is built here is that same namespace written the way a subscriber can
    # use it.
    def getControlsNamespace(self, controls_name):
        return nepi_sdk.create_namespace(self.node_namespace, controls_name)

    def getControlsReadyState(self):
        if self.process_controls_ifs is None:
            return False
        for process_name in self.available_processes:
            controls_if = self.process_controls_ifs.get(process_name, None)
            if controls_if is None:
                return False
            if controls_if.get_controls_ready_state() is not True:
                return False
        return True

    # Flat {control_name: current_value} snapshot of one ControlsIF, or None when
    # that IF has no controls dict to read.
    #
    # The None case used to be routine: ControlsIF.init() read its persisted controls
    # with get_param('controls_dict') while the param is registered under
    # '/<controls_name>_controls_dict', and ParamsIF.get_param() returns None for a
    # name it does not know -- so every config init, reset() and factory_reset()
    # replaced that IF's controls dict with None, and its controls stopped appearing
    # in ControlsStatus at all. Fixed in nepi_api/system_if.py (prefixed key, plus a
    # guard so a missing param cannot overwrite a live dict), which is what makes
    # this app's controls render. The check stays because a None dict would make
    # nepi_controls.get_control_value() raise rather than return a default, and the
    # caller, not this helper, decides what a controls-less process cycle means.
    def getControlsValues(self, controls_if):
        if controls_if is None:
            return None
        controls_dict = controls_if.get_controls_dict()
        if controls_dict is None:
            return None
        values = dict()
        for control_name in controls_dict.keys():
            values[control_name] = controls_if.get_control_value(control_name)
        return values

    # Show the active process's controls and suppress every other process's.
    #
    # The suppression the OPERATOR sees is the RUI not mounting an inactive set's
    # Nepi_IF_Controls at all: NepiAppObstacles.js mounts one component on the
    # active_controls_namespace this app reports in its status message, the same
    # node-derives / status-names-it / RUI-binds rule this file already applies to
    # its three derived image topics. set_controls_hidden() is called as well
    # because it is the honest statement of intent on the wire and it stores a real
    # bool -- but Nepi_IF_Controls reads only each control's own hidden flag, never
    # ControlsStatus.hidden, so that call suppresses nothing on its own today.
    #
    # Per-control set_control_hidden() is NOT used, here or anywhere in this app:
    # nepi_controls.set_control_hidden() does `hidden = str(hidden)`, writing
    # 'True'/'False' into a field nepi_interfaces/Control declares bool, which
    # breaks the ControlsStatus publish and never satisfies the RUI's
    # `control_msg.hidden === true` test. A control's 'hidden' is usable only as
    # authored in its init dict.
    def applyActiveProcessVisibility(self):
        if self.process_controls_ifs is None:
            return
        for process_name in self.process_controls_ifs.keys():
            controls_if = self.process_controls_ifs[process_name]
            if controls_if is not None:
                controls_if.set_controls_hidden(process_name != self.selected_process)


    ###################
    ## Controls Callbacks

    # Per-process controls callback, bound to the process it belongs to.
    #
    # ControlsIF hands its updated-callback only a control name, with no
    # indication of which instance it came from, and both process sets
    # deliberately use the same natural key names (min_range_m is in both). So each
    # process IF gets its own bound callback carrying the process name rather than
    # one shared callback that would have to guess which set moved.
    def makeProcessControlsUpdatedCb(self, process_name):
        def updatedCb(control_name):
            self.processControlsUpdatedCb(process_name, control_name)
        return updatedCb

    def processControlsUpdatedCb(self, process_name, control_name):
        controls_if = None
        if self.process_controls_ifs is not None:
            controls_if = self.process_controls_ifs.get(process_name, None)
        value = None
        if controls_if is not None:
            values = self.getControlsValues(controls_if)
            if values is not None:
                value = values.get(control_name, None)
        self.msg_if.pub_info("Process '" + str(process_name) + "' control '" +
                             str(control_name) + "' updated to: " + str(value))


    ###################
    ## Process Cycle

    def processUpdateCb(self, timer):
        self.updateProcess()

    # One processing cycle: hand the selected process function the current data
    # and the current value of every control in ITS OWN control set.
    #
    # The inputs are the connect callbacks' cached data dicts, passed verbatim --
    # see nepi_obstacles.DATA_DICT for why they are not unpacked into named fields
    # here. process_data_dict is handed back by the process function and kept, so a
    # process can carry state between cycles.
    def updateProcess(self):
        if self.enabled is False:
            return
        process = nepi_obstacles.PROCESSES_DICT.get(self.selected_process, None)
        controls_if = None
        if self.process_controls_ifs is not None:
            controls_if = self.process_controls_ifs.get(self.selected_process, None)
        if process is None or controls_if is None:
            return
        process_controls_dict = self.getControlsValues(controls_if)
        if process_controls_dict is None:
            if self.logged_controls_dict_none is False:
                self.logged_controls_dict_none = True
                self.msg_if.pub_warn("Process cycle skipped: controls for '" +
                                     str(self.selected_process) +
                                     "' read back as None -- see getControlsValues()")
            return
        self.process_data_dict['depth_map_dict'] = self.depth_map_dict
        self.process_data_dict['targets_dict'] = self.targets_dict
        self.process_data_dict['navpose_dict'] = self.navpose_dict
        self.process_data_dict, _ = process['process_function'](
                        self.process_data_dict, process_controls_dict)


    ###################
    ## Interface IF Setup

    def setupInterfaceIFs(self):
        # Connect-side (consumer) IFs, in RUI display order: depth map, its color
        # image, targets, its targets image, NavPose. Each auto-discovers the
        # matching producer topics; each connect_name is passed explicitly to keep
        # the RUI binding greppable from here.
        #
        # One rule governs the whole set: the operator selects a SOURCE, and every
        # IMAGE that belongs to that source is derived node-side. Depth map,
        # targets and NavPose are the sources and get selectors
        # (show_selector=True). The two images do not: each source's own status
        # message already names the image that belongs to it -- DepthMapStatus
        # names the depth map image at <depth_map>/depth_map_image and the sibling
        # color image in image_topic, and TargetingStatus names the detections
        # image whose sibling is the targets image -- so an image selector could
        # only offer the operator a way to disagree with the source they just
        # picked. Both image IFs are therefore built with show_selector=False and
        # their selections driven from this node by
        # applyDerivedColorImageSelection() and
        # applyDerivedTargetsImageSelection(). The connections are kept so the RUI
        # can still show whether each derived image is actually connected.
        self.depth_map_if = ConnectDepthMapIF(
                        connect_name = DEPTH_MAP_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.depthMapConnectCb,
                        msg_if = self.msg_if)

        self.color_image_if = ConnectColorImageIF(
                        connect_name = COLOR_IMAGE_CONNECT_NAME,
                        show_selector = False,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.colorImageConnectCb,
                        msg_if = self.msg_if)

        self.targets_if = ConnectTargetsIF(
                        connect_name = TARGETS_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.targetsConnectCb,
                        msg_if = self.msg_if)

        self.targets_image_if = ConnectColorImageIF(
                        connect_name = TARGETS_IMAGE_CONNECT_NAME,
                        show_selector = False,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.targetsImageConnectCb,
                        msg_if = self.msg_if)

        self.navpose_if = ConnectNavPoseIF(
                        connect_name = NAVPOSE_CONNECT_NAME,
                        show_selector = True,
                        show_controls = False,
                        show_data = False,
                        dataCB = self.navposeConnectCb,
                        msg_if = self.msg_if)


    ###################
    ## Connect IF Callbacks
    #
    # Each connect IF invokes its dataCB with a single data dict. The
    # callback stores that dict and the IF's current status message (via
    # get_status_msg()) on EVERY invocation, so the rest of the app reads the most
    # recent values from the class attributes rather than re-querying the IF. On
    # the FIRST invocation per IF it also logs both, then sets the got_first flag
    # so it logs only once. Unlike the wpilib app's callbacks, these also
    # republish status when the selected source topics change: this app reports
    # those topics in its own status message, so the RUI must see a new selection
    # without waiting out a full status timer period, and the callbacks run at the
    # source's data rate rather than the 1 Hz status rate. The stored data itself
    # is not consumed here -- this app wires sources, obstacle detection
    # processing is separate work.
    #
    # NOTE: the cached status msgs are for this app's own use. publish_status()
    # deliberately does NOT read them -- see getDepthMapImageTopic().

    def depthMapConnectCb(self, data_dict):
        self.depth_map_dict = data_dict
        self.depth_map_status = self.depth_map_if.get_status_msg()
        if self.got_first_depth_map is False:
            self.got_first_depth_map = True
            self.msg_if.pub_info("Depth map first-connection data dict: " + str(self.depth_map_dict))
            self.msg_if.pub_info("Depth map first-connection status message: " + str(self.depth_map_status))
        self.applyDerivedColorImageSelection()
        self.publishStatusIfSourcesChanged()

    def colorImageConnectCb(self, data_dict):
        self.color_image_dict = data_dict
        self.color_image_status = self.color_image_if.get_status_msg()
        if self.got_first_color_image is False:
            self.got_first_color_image = True
            self.msg_if.pub_info("Color image first-connection data dict: " + str(self.color_image_dict))
            self.msg_if.pub_info("Color image first-connection status message: " + str(self.color_image_status))
        self.publishStatusIfSourcesChanged()

    def targetsConnectCb(self, data_dict):
        self.targets_dict = data_dict
        self.targets_status = self.targets_if.get_status_msg()
        if self.got_first_targets is False:
            self.got_first_targets = True
            self.msg_if.pub_info("Targets first-connection data dict: " + str(self.targets_dict))
            self.msg_if.pub_info("Targets first-connection status message: " + str(self.targets_status))
        self.applyDerivedTargetsImageSelection()
        self.publishStatusIfSourcesChanged()

    def targetsImageConnectCb(self, data_dict):
        self.targets_image_dict = data_dict
        self.targets_image_status = self.targets_image_if.get_status_msg()
        if self.got_first_targets_image is False:
            self.got_first_targets_image = True
            self.msg_if.pub_info("Targets image first-connection data dict: " + str(self.targets_image_dict))
            self.msg_if.pub_info("Targets image first-connection status message: " + str(self.targets_image_status))
        self.publishStatusIfSourcesChanged()

    def navposeConnectCb(self, data_dict):
        self.navpose_dict = data_dict
        self.navpose_status = self.navpose_if.get_status_msg()
        if self.got_first_navpose is False:
            self.got_first_navpose = True
            self.msg_if.pub_info("NavPose first-connection data dict: " + str(self.navpose_dict))
            self.msg_if.pub_info("NavPose first-connection status message: " + str(self.navpose_status))
        self.publishStatusIfSourcesChanged()


    ###################
    ## Status Publishers

    def statusPublishCb(self, timer):
        self.publish_status()

    # Depth map image topic belonging to the SELECTED depth map, or 'None'.
    #
    # The operator never selects a depth map image, so this is derived. It is read
    # LIVE off the connect IF rather than from the cached self.depth_map_status:
    # that cache only refreshes when a depth map frame arrives, so after a switch
    # to a depth map with no image -- or a deselect -- it would keep reporting the
    # previous selection's image topic. ConnectDepthMapIF clears its own status_msg
    # on resubscribe, on unsubscribe, and after its connection timeout, so the live
    # read is the one that tracks the selection.
    #
    # Ownership is checked before the status is trusted at all. ConnectDepthMapIF
    # resubscribes on its own 1 Hz tick, so for up to a second after a selection
    # change get_status_msg() still returns the PREVIOUS depth map's status;
    # DepthMapIF reports its own namespace in DepthMapStatus.depth_map_topic, so a
    # status whose depth_map_topic is not the selected topic is discarded.
    #
    # DepthMapStatus.image_topic is only this depth map's own image when it
    # resolves UNDER the selected depth map namespace. On every DepthMapIF producer
    # in nepi_api today it does not: DepthMapIF._updaterCb fills image_topic with
    # the SIBLING <device>/color_image, so taking it at face value would put the
    # color image in the depth map viewer. Anything not under the selection is
    # therefore rejected in favor of the topic DepthMapIF actually publishes its
    # depth map image on -- <depth_map>/depth_map_image, gated on the producer's
    # own DepthMapStatus.img_pub_enabled flag. A depth map that reports no image
    # yields 'None' and the RUI viewer stays unmounted; the color image is never
    # substituted.
    def getDepthMapImageTopic(self):
        if self.depth_map_if is None:
            return 'None'
        selected_topic = self.depth_map_if.get_selected_topic()
        if selected_topic is None or selected_topic == '' or selected_topic == 'None':
            return 'None'
        status_msg = self.depth_map_if.get_status_msg()
        if status_msg is None:
            return 'None'
        if status_msg.depth_map_topic != selected_topic:
            return 'None'
        image_topic = status_msg.image_topic
        if image_topic is not None and image_topic.startswith(selected_topic + '/'):
            return image_topic
        if status_msg.img_pub_enabled == True:
            return nepi_sdk.create_namespace(selected_topic, DEPTH_MAP_IMAGE_SUBTOPIC)
        return 'None'

    # Color image topic belonging to the SELECTED depth map, or 'None'.
    #
    # The structural twin of getDepthMapImageTopic(), applying the same guards in
    # the same order and for the same reasons: 'None' when the IF is not built or
    # nothing is selected, the status read LIVE off the connect IF rather than from
    # the cached self.depth_map_status (that cache only refreshes when a depth map
    # frame arrives, so it would keep reporting the previous selection after a
    # switch or a deselect), and a status whose depth_map_topic is not the selected
    # topic discarded outright -- ConnectDepthMapIF resubscribes on its own 1 Hz
    # tick, so for up to a second after a selection change get_status_msg() still
    # returns the PREVIOUS depth map's status.
    #
    # The two methods then diverge on exactly one point, in opposite directions.
    # DepthMapStatus.image_topic is the SIBLING <device>/color_image on every
    # DepthMapIF producer in nepi_api today -- DepthMapIF._updaterCb fills it that
    # way -- which is what this method wants and what getDepthMapImageTopic()
    # rejects. So it is taken only when it does NOT resolve under the selected
    # depth map namespace. A depth map that reported its own depth map image there
    # instead would be the one case where that field is not a color image, and
    # rejecting it keeps a depth map image out of the color image viewer.
    def getColorImageTopic(self):
        if self.depth_map_if is None:
            return 'None'
        selected_topic = self.depth_map_if.get_selected_topic()
        if selected_topic is None or selected_topic == '' or selected_topic == 'None':
            return 'None'
        status_msg = self.depth_map_if.get_status_msg()
        if status_msg is None:
            return 'None'
        if status_msg.depth_map_topic != selected_topic:
            return 'None'
        image_topic = status_msg.image_topic
        if image_topic is None or image_topic == '' or image_topic == 'None':
            return 'None'
        if image_topic.startswith(selected_topic + '/'):
            return 'None'
        return image_topic

    # Point the color image connect IF at the derived color image topic.
    #
    # ConnectNodeIF.set_selected_topic() only accepts a topic already present in
    # available_topics (or the literal 'None'), and its publish_status()
    # auto-select branch grabs available_topics[0] whenever the current selection
    # is absent from that list. Both facts mean the IF's own selection cannot be
    # trusted to hold: a derived topic not yet discovered is silently ignored, and
    # a derived 'None' is overwritten by the first available image on the very next
    # IF status cycle. So the node re-asserts the derivation every cycle rather
    # than setting it once, and status_msg.image_topic reports the DERIVED topic
    # rather than whatever the IF currently has selected -- the RUI viewer follows
    # the selected depth map, not the auto-select.
    def applyDerivedColorImageSelection(self):
        if self.color_image_if is None:
            return
        derived_topic = self.getColorImageTopic()
        if derived_topic != self.color_image_if.get_selected_topic():
            self.color_image_if.set_selected_topic(derived_topic)

    # Targets image topic belonging to the SELECTED targets source, or 'None'.
    #
    # The structural twin of getColorImageTopic(): same guards in the same order,
    # derived off the live ConnectTargetsIF status rather than the cached
    # self.targets_status, for the reason documented on getDepthMapImageTopic() --
    # the cache only refreshes when a Targets message arrives, so it would keep
    # reporting the previous detector's image after a switch or a deselect.
    # process_status.namespace is the ownership check here: AiDetectorIF overrides
    # it with its own <node>/targets namespace before handing the message to
    # TargetsIF.publish_status(), which is exactly the topic ConnectTargetsIF
    # selects, so a status whose namespace is not the selected topic belongs to a
    # detector the operator has already switched away from.
    #
    # Beyond the shared guards, the detector only publishes overlay images while
    # its imaging path is running, so has_imaging, imaging_enabled and a non-empty
    # imaging_pub_topics are all required before any topic can be named.
    #
    # imaging_pub_topics entries are DETECTIONS image topics, siblings of the
    # targets image under the same image source directory, so the targets image is
    # built as TARGETS_IMAGE_SUBTOPIC under the chosen entry's parent namespace --
    # the same sibling-directory move getColorImageTopic() relies on, inverted:
    # there a sibling was read out of a status field, here one is constructed from
    # a status field that points at the sibling's directory.
    #
    # A detector may run on several image sources at once, so imaging_pub_topics
    # can hold more than one entry. The entry chosen is the one whose index-paired
    # imaging_source_topics value is the color image derived from the selected
    # depth map, so the targets viewer and the color image viewer show the same
    # camera. With no match there is no better answer than the detector's first
    # source, and that choice is logged once rather than every cycle.
    #
    # Unlike the color image, the derived topic is finally checked against the
    # targets image IF's available topics. The color image is a topic the depth map
    # producer names outright, but this one is reconstructed from a sibling's path,
    # so a producer that publishes its images elsewhere would yield a plausible
    # name for a topic that does not exist. Reporting 'None' keeps the RUI viewer
    # unmounted instead of mounting a dead stream.
    def getTargetsImageTopic(self):
        if self.targets_if is None:
            return 'None'
        selected_topic = self.targets_if.get_selected_topic()
        if selected_topic is None or selected_topic == '' or selected_topic == 'None':
            return 'None'
        status_msg = self.targets_if.get_status_msg()
        if status_msg is None:
            return 'None'
        process_status = status_msg.process_status
        if process_status.namespace != selected_topic:
            return 'None'
        if process_status.has_imaging != True or process_status.imaging_enabled != True:
            return 'None'
        pub_topics = process_status.imaging_pub_topics
        if pub_topics is None or len(pub_topics) == 0:
            return 'None'

        source_topics = process_status.imaging_source_topics
        color_image_topic = self.getColorImageTopic()
        pub_index = None
        if color_image_topic != 'None' and source_topics is not None:
            for index in range(min(len(source_topics), len(pub_topics))):
                if source_topics[index] == color_image_topic:
                    pub_index = index
                    break
        if pub_index is None:
            pub_index = 0
            if self.logged_targets_image_fallback is False:
                self.logged_targets_image_fallback = True
                self.msg_if.pub_info("Targets image: no detector image source matched the derived color image (" +
                                     str(color_image_topic) + "), using first source: " + str(source_topics))

        pub_topic = pub_topics[pub_index]
        if pub_topic is None or pub_topic == '' or pub_topic == 'None':
            return 'None'
        targets_image_topic = nepi_sdk.create_namespace(os.path.dirname(pub_topic), TARGETS_IMAGE_SUBTOPIC)
        if self.targets_image_if is None:
            return 'None'
        if targets_image_topic not in self.targets_image_if.get_available_topics():
            return 'None'
        return targets_image_topic

    # Point the targets image connect IF at the derived targets image topic. Same
    # re-assert-every-cycle reasoning as applyDerivedColorImageSelection().
    def applyDerivedTargetsImageSelection(self):
        if self.targets_image_if is None:
            return
        derived_topic = self.getTargetsImageTopic()
        if derived_topic != self.targets_image_if.get_selected_topic():
            self.targets_image_if.set_selected_topic(derived_topic)

    # Source topics this app reports, keyed by status message field name. A
    # connect IF that is not built yet reports 'None', which is also what
    # get_selected_topic() returns for an unselected source. The image fields are
    # derivations, never get_selected_topic() -- see
    # applyDerivedColorImageSelection() for why the IF's own selection is not the
    # answer.
    def getSourceTopics(self):
        source_topics = {
            'depth_map_topic': 'None',
            'depth_map_image_topic': self.getDepthMapImageTopic(),
            'image_topic': self.getColorImageTopic(),
            'targets_topic': 'None',
            'targets_image_topic': self.getTargetsImageTopic(),
            'navpose_topic': 'None'
        }
        if self.depth_map_if is not None:
            source_topics['depth_map_topic'] = self.depth_map_if.get_selected_topic()
        if self.targets_if is not None:
            source_topics['targets_topic'] = self.targets_if.get_selected_topic()
        if self.navpose_if is not None:
            source_topics['navpose_topic'] = self.navpose_if.get_selected_topic()
        return source_topics

    def publishStatusIfSourcesChanged(self):
        if self.getSourceTopics() != self.last_source_topics:
            self.publish_status()

    def publish_status(self):
        # Re-assert the node-driven image selections before reading the source
        # topics, so the connect IFs track the current derivation even when no
        # source callback has fired -- see applyDerivedColorImageSelection().
        self.applyDerivedColorImageSelection()
        self.applyDerivedTargetsImageSelection()
        status_msg = NepiAppObstaclesStatus()
        status_msg.enabled = self.enabled
        status_msg.options = self.options
        status_msg.selected_option = self.selected_option
        status_msg.value = self.value
        source_topics = self.getSourceTopics()
        status_msg.depth_map_topic = source_topics['depth_map_topic']
        status_msg.depth_map_image_topic = source_topics['depth_map_image_topic']
        status_msg.image_topic = source_topics['image_topic']
        status_msg.targets_topic = source_topics['targets_topic']
        status_msg.targets_image_topic = source_topics['targets_image_topic']
        status_msg.navpose_topic = source_topics['navpose_topic']
        self.last_source_topics = source_topics
        # Controls namespaces, all fully qualified -- see getControlsNamespace()
        # for why they are not ControlsIF.get_namespace(). The RUI mounts exactly
        # one Nepi_IF_Controls, on active_controls_namespace, which is how a page
        # owning one controls namespace per process still shows the operator only
        # the active process's controls.
        status_msg.active_controls_namespace = 'None'
        if self.selected_process in self.available_processes:
            status_msg.active_controls_namespace = self.getControlsNamespace(self.selected_process)
        controls_namespaces = []
        for process_name in self.available_processes:
            controls_namespaces.append(self.getControlsNamespace(process_name))
        status_msg.controls_namespaces = controls_namespaces
        status_msg.controls_ready = self.getControlsReadyState()
        # The example control set is reported separately from the process sets: it is
        # not one of them, and the RUI mounts it alongside the active process's
        # controls rather than instead of them. Same fully-qualified form.
        status_msg.example_controls_namespace = self.getControlsNamespace(EXAMPLE_CONTROLS_NAME)
        status_msg.example_controls_ready = self.getExampleControlsReadyState()
        status_msg.selected_process = self.selected_process
        status_msg.available_processes = self.available_processes
        status_msg.process_ready = self.process_ready
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions
    #######################

    def cleanup_actions(self):
        self.msg_if.pub_info("OBSTACLES: Shutting down: Executing script cleanup actions")
        # The per-process ControlsIF instances are deliberately NOT unregistered here.
        # ControlsIF.unregister() opens with a call to self.unsubscribe_topic(),
        # which no method in nepi_api/system_if.py defines, so it raises
        # AttributeError before it can release anything -- and it would raise inside
        # the shutdown handler, ahead of the connect IF cleanup below. Their
        # pubs/subs go down with the node.
        if self.depth_map_if is not None:
            self.depth_map_if.unregister()
        if self.color_image_if is not None:
            self.color_image_if.unregister()
        if self.targets_if is not None:
            self.targets_if.unregister()
        if self.targets_image_if is not None:
            self.targets_image_if.unregister()
        if self.navpose_if is not None:
            self.navpose_if.unregister()


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiObstaclesApp()
