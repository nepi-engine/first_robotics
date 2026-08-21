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
# The whole RBX surface for the WPILib IF app, in one self-contained module.
#
# Decision 3 in docs/WPILIB_IF_DESIGN.md: one robot is one RBX device, so this
# builds ONE RBXRobotIF representing the whole robot, with the mapped RoboRIO
# motors as that device's motor channels. Precedent is nepi_app_sim_connector
# hosting SimDeviceIF.
#
# RELOCATION. This module is deliberately transport-blind: it never imports
# ntcore or nepi_wpilib, and its only inputs are six injected callables. Moving
# the RBX device into an rbx_wpilib driver package under nepi_drivers is
# therefore a file move (this file becomes rbx_wpilib_node.py) plus a transport
# swap (rebind the six callables to whatever transport the driver holds). The
# RBX callbacks below do not change.
#
# REGISTRY KEYS (2026-07 DECISION LOG). This module shares no node_if with
# anything. RBXRobotIF always builds its own NodeClassIF internally, and nothing
# here passes node_if= to any interface, so there is no keyed dict.update() that
# could overwrite a sibling's registry entry and no domain prefix is needed. If
# a future change ever hands this module the app's node_if, every key it
# registers must be prefixed 'rbx_' first.

import copy
import math
import threading

from std_msgs.msg import String

from nepi_sdk import nepi_sdk

from nepi_interfaces.msg import AxisControls

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_rbx import RBXRobotIF


#########################################
# RBX contract constants
#########################################

# Capability strings the RoboRIO advertises in the RBX Feedback group's
# supported_capabilities list. Which RBXRobotIF callbacks are non-None is
# derived from this list, exactly the way nepi_app_sim_connector derives its
# capability kwargs from a robot config -- so a robot that does not advertise a
# capability gets a None callback and the matching has_* flag falls out False.
CAPABILITY_GOTO_POSITION = 'GOTO_POSITION'
CAPABILITY_GOTO_POSE = 'GOTO_POSE'
CAPABILITY_GO_HOME = 'GO_HOME'
CAPABILITY_STOP = 'STOP'
CAPABILITY_MOTOR_CONTROL = 'MOTOR_CONTROL'

# request_status values the RBX Feedback group reports. Anything not in this set
# is passed through to the operator verbatim rather than reinterpreted.
REQUEST_STATUS_IDLE = 'IDLE'
REQUEST_STATUS_ACCEPTED = 'ACCEPTED'
REQUEST_STATUS_EXECUTING = 'EXECUTING'
REQUEST_STATUS_COMPLETE = 'COMPLETE'
REQUEST_STATUS_FAILED = 'FAILED'
REQUEST_STATUS_REJECTED = 'REJECTED'

REQUEST_STATUS_FAILURES = [REQUEST_STATUS_FAILED, REQUEST_STATUS_REJECTED]

# How often the RBX Feedback group is folded into the RBX device's reported
# state. Twice the device's own 2 Hz status cadence would add nothing.
FEEDBACK_UPDATE_RATE_HZ = 2.0

# First request id issued. Ids only ever increase, so the RoboRIO can trigger on
# request_id changing.
FIRST_REQUEST_ID = 1


#########################################
# WPILib RBX IF Class
#########################################

class WpilibRbxIF:

    ready = False

    rbx_if = None
    namespace = 'None'

    def __init__(self,
                 device_name,
                 getMotorSlotsFunction,
                 setMotorCommandFunction,
                 getNavPoseFunction,
                 getRbxFeedbackFunction,
                 getConnectedFunction,
                 writeCommandRequestFunction,
                 command_types,
                 supported_capabilities=None,
                 log_name=None,
                 msg_if=None):
        """Build and own the RBX device for the WPILib robot.

        Args:
            device_name (str): Device name reported in RBX info and status.
            getMotorSlotsFunction (callable): Returns the ordered motor slot
                list, one dict per slot with at least 'motor_id'.
            setMotorCommandFunction (callable): Called as
                (motor_id, speed_ratio) to write a per-motor command out.
            getNavPoseFunction (callable): Returns the fused navpose dict, or
                None when no input group is currently valid.
            getRbxFeedbackFunction (callable): Returns the RBX Feedback group
                dict, or None if it has never been seen.
            getConnectedFunction (callable): Returns the live robot network
                connection state.
            writeCommandRequestFunction (callable): Called as
                (request_id, command_type, chassis_speeds, target_pose,
                named_action) to write one RBX Command Request out.
            command_types (dict): The COMMAND_TYPE_* codes, keyed
                'chassis_speeds', 'target_pose', 'named_action', 'stop'.
            supported_capabilities (list): Capability strings from the RBX
                Feedback group. Decides which callbacks are non-None.
            log_name (str): Optional log name for this interface.
            msg_if (MsgIF): Shared message interface, or None to create one.
        """
        self.class_name = type(self).__name__
        self.device_name = device_name

        self.getMotorSlotsFunction = getMotorSlotsFunction
        self.setMotorCommandFunction = setMotorCommandFunction
        self.getNavPoseFunction = getNavPoseFunction
        self.getRbxFeedbackFunction = getRbxFeedbackFunction
        self.getConnectedFunction = getConnectedFunction
        self.writeCommandRequestFunction = writeCommandRequestFunction
        self.command_types = copy.deepcopy(command_types)

        if supported_capabilities is None:
            supported_capabilities = []
        self.supported_capabilities = [str(c) for c in supported_capabilities]

        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF(log_name=self.class_name)
        self.log_name = log_name

        ##############################
        # Command and state tracking
        self.request_lock = threading.Lock()
        self.next_request_id = FIRST_REQUEST_ID
        self.last_request_id = 0
        self.last_request_type = 'None'

        self.stop_triggered = False
        self.last_request_status = 'None'
        # Last process name actually pushed to RBXRobotIF. Deliberately seeded
        # to a value the device can never report, so the first real update
        # always goes through.
        self.last_process_name = None

        # Manual motor-control set points, in slot order. This is the index
        # space RBXRobotIF bounds-checks MotorControl.motor_ind against, so its
        # length is resynced from the slot list on every read.
        self.motor_ratios = []
        self.syncMotorRatios()

        ##############################
        # Build the RBX device
        self.rbx_if = RBXRobotIF(
            device_info=dict(device_name=self.device_name,
                             path='',
                             serial_number='',
                             hw_version='',
                             sw_version=''),
            # No settings: the RoboRIO contract exposes nothing an operator sets
            # on the device itself (motor mapping and team number are the app's
            # own params). SettingsIF substitutes its NONE_* defaults for each
            # None, so the settings panel renders empty rather than fake.
            capSettings=None,
            factorySettings=None,
            settingUpdateFunction=None,
            getSettingsFunction=None,
            axisControls=self.buildAxisControls(),
            # None: the RBX Feedback group carries no battery field, so there is
            # no battery state to report.
            getBatteryPercentFunction=None,
            # Empty: the RBX Feedback group carries no state or mode
            # enumeration. RBXRobotIF handles empty lists correctly (its bounds
            # checks reject every set_state/set_mode index and status reads "Not
            # Set"), but it calls the get*Ind functions unconditionally, so
            # those must still be real callables.
            states=[],
            getStateIndFunction=self.getStateInd,
            setStateIndFunction=self.setStateInd,
            modes=[],
            getModeIndFunction=self.getModeInd,
            setModeIndFunction=self.setModeInd,
            checkStopFunction=self.checkStopFunction,
            # Empty: the RoboRIO advertises one flat capability list with no
            # setup/go distinction, so every named action is offered as a go
            # action and there is nothing left for setup actions to hold.
            setup_actions=[],
            setSetupActionIndFunction=None,
            go_actions=self.getGoActions(),
            setGoActionIndFunction=(self.setGoActionInd
                                    if self.getGoActions() else None),
            data_source_description='roborio',
            data_ref_description='roborio',
            # None both: the RBX Feedback group reports no home pose and the RBX
            # Command Request group has no set-home field, so home cannot be
            # read or written over this contract. GO_HOME is still commandable
            # as a named action when the RoboRIO advertises it -- the RoboRIO
            # owns where home is.
            getHomeFunction=None,
            setHomeFunction=None,
            manualControlsReadyFunction=self.manualControlsReady,
            getMotorControlRatios=(self.getMotorControlRatios
                                   if self.hasCapability(CAPABILITY_MOTOR_CONTROL) else None),
            setMotorControlRatio=(self.setMotorControlRatio
                                  if self.hasCapability(CAPABILITY_MOTOR_CONTROL) else None),
            autonomousControlsReadyFunction=self.autonomousControlsReady,
            goHomeFunction=(self.goHome
                            if self.hasCapability(CAPABILITY_GO_HOME) else None),
            goStopFunction=(self.goStop
                            if self.hasCapability(CAPABILITY_STOP) else None),
            gotoPoseFunction=(self.gotoPose
                              if self.hasCapability(CAPABILITY_GOTO_POSE) else None),
            gotoPositionFunction=(self.gotoPosition
                                  if self.hasCapability(CAPABILITY_GOTO_POSITION) else None),
            # None: the RoboRIO has no global position reference in this
            # contract. Robot Position is a local field-frame x/y/heading, so
            # there is no lat/lon to command to.
            gotoLocationFunction=None,
            getNavPoseCb=self.getNavPoseCb,
            navpose_update_rate=10,
            log_name=self.log_name,
            msg_if=self.msg_if)

        self.namespace = self.rbx_if.namespace

        ##############################
        # Fold RBX Feedback into the device's reported state
        nepi_sdk.start_timer_process(float(1) / FEEDBACK_UPDATE_RATE_HZ,
                                     self.feedbackUpdateCb)

        self.ready = True
        self.msg_if.pub_info("WPILib RBX interface running at " + str(self.namespace) +
                             " with capabilities: " + str(self.supported_capabilities))


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        """Return the ready state of this interface.

        Returns:
            bool: True once the RBX device has been built.
        """
        return self.ready

    def get_namespace(self):
        """Return the namespace the RBX device is advertised at.

        Returns:
            str: The RBX device namespace, '<app node>/rbx'.
        """
        return self.namespace

    def get_supported_capabilities(self):
        """Return the capability list this interface was built from.

        Returns:
            list: Capability strings as read from the RBX Feedback group at
                construction time.
        """
        return list(self.supported_capabilities)

    def get_device_ready_state(self):
        """Return the RBX device's own ready (not busy) state.

        Returns:
            bool: True when the device is idle and can accept a new command.
        """
        if self.rbx_if is None:
            return False
        try:
            return bool(self.rbx_if.status_msg.ready)
        except Exception:
            return False

    def get_motor_ratios(self):
        """Return the current manual motor-control set points in slot order.

        Returns:
            list: One float ratio (0.0-1.0) per motor slot.
        """
        return list(self.motor_ratios)

    def shutdown(self):
        """Retract as much of the RBX device from the ROS graph as nepi_api allows.

        Unregisters the RBXRobotIF NodeClassIF (every rbx/* topic, service and
        param) and each child interface that exposes a public unregister. What
        CANNOT be retracted is logged rather than glossed over: RBXRobotIF has
        no teardown entry point of its own, its NPXDeviceIF exposes none, and
        nepi_sdk timers cannot be cancelled once started, so the npx status and
        navpose timers keep firing against unregistered publishers until the
        node restarts. A full retract needs an apps_mgr disable/enable cycle.

        Returns:
            bool: True if the RBX device's own NodeClassIF was unregistered.
        """
        if self.rbx_if is None:
            return False

        self.ready = False
        success = False

        # Child interfaces first, so their own topics go before the device's.
        for (attr_name, if_obj) in self.reachableChildIfs():
            try:
                if_obj.unregister()
            except Exception as e:
                self.msg_if.pub_warn("Could not unregister " + str(attr_name) +
                                     ": " + str(e))

        # The NPX navpose publisher is reachable through its own NavPoseIF.
        try:
            npx_if = getattr(self.rbx_if, 'npx_if', None)
            if npx_if is not None:
                navpose_if = getattr(npx_if, 'navpose_if', None)
                if navpose_if is not None:
                    navpose_if.unsubscribe()
                node_if = getattr(npx_if, 'node_if', None)
                if node_if is not None:
                    node_if.unregister_class()
        except Exception as e:
            self.msg_if.pub_warn("Could not unregister NPX interface: " + str(e))

        try:
            self.rbx_if.node_if.unregister_class()
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Could not unregister RBX node class: " + str(e))

        self.msg_if.pub_warn("RBX device at " + str(self.namespace) + " torn down. "
                             "NPX status and navpose timers cannot be cancelled by "
                             "nepi_sdk, so a full retract needs an app restart.")
        self.rbx_if = None
        return success


    ###############################
    # Class Private Methods
    ###############################

    def reachableChildIfs(self):
        child_ifs = []
        for attr_name in ['image_if', 'settings_if', 'save_data_if', 'transform_if']:
            if_obj = getattr(self.rbx_if, attr_name, None)
            if if_obj is not None and hasattr(if_obj, 'unregister'):
                child_ifs.append((attr_name, if_obj))
        return child_ifs

    def hasCapability(self, capability):
        return capability in self.supported_capabilities

    def getGoActions(self):
        # Every advertised capability that is not already a first-class
        # RBXRobotIF control becomes a named action, so a RoboRIO can offer
        # robot-specific actions without this module knowing their names.
        control_capabilities = [CAPABILITY_GOTO_POSITION, CAPABILITY_GOTO_POSE,
                                CAPABILITY_GO_HOME, CAPABILITY_STOP,
                                CAPABILITY_MOTOR_CONTROL]
        return [c for c in self.supported_capabilities if c not in control_capabilities]

    def buildAxisControls(self):
        axis_controls = AxisControls()
        # target_pose carries x_m, y_m and heading_rad only, so the commandable
        # set is the ground plane plus yaw. z is reported in Robot Position but
        # cannot be commanded.
        has_position = self.hasCapability(CAPABILITY_GOTO_POSITION)
        has_pose = self.hasCapability(CAPABILITY_GOTO_POSE)
        axis_controls.x = has_position
        axis_controls.y = has_position
        axis_controls.z = False
        axis_controls.roll = has_pose
        axis_controls.pitch = has_pose
        axis_controls.yaw = has_position or has_pose
        return axis_controls

    def getSlots(self):
        try:
            slots = self.getMotorSlotsFunction()
        except Exception as e:
            self.msg_if.pub_warn("Failed to read motor slots: " + str(e), throttle_s=10.0)
            return []
        if slots is None:
            return []
        return slots

    def syncMotorRatios(self):
        slot_count = len(self.getSlots())
        if len(self.motor_ratios) == slot_count:
            return
        ratios = [0.0] * slot_count
        for i in range(min(slot_count, len(self.motor_ratios))):
            ratios[i] = self.motor_ratios[i]
        self.motor_ratios = ratios

    def allocateRequestId(self):
        with self.request_lock:
            request_id = self.next_request_id
            self.next_request_id = self.next_request_id + 1
            self.last_request_id = request_id
        return request_id

    # TEST MODE -- added for it, but NOT dependent on it and safe to keep after
    # test mode is removed. This is plain command tracing: it is not gated on
    # test mode and cannot be, because this module never learns test mode
    # exists. Delete logCommandEntry and its call sites only if the tracing
    # itself is unwanted.
    #
    # Every RBX command entry point announces itself here, in one shape, before
    # it does anything. This module stays transport-blind: it does not know
    # whether the injected writer reaches a robot, a log line or nothing at all,
    # and it must not learn -- so it logs what it was ASKED to do and leaves what
    # was actually written to whoever owns the transport. Read together, the two
    # halves say what RBXRobotIF wanted and what went out.
    def logCommandEntry(self, command_name, detail):
        self.msg_if.pub_info("RBX command in: " + str(command_name) + "  " + str(detail))

    def sendCommandRequest(self, command_type, chassis_speeds=None,
                           target_pose=None, named_action='', request_type='None'):
        request_id = self.allocateRequestId()
        self.last_request_type = request_type
        self.msg_if.pub_info("RBX command request out: id " + str(request_id) +
                             "  " + str(request_type) +
                             "  command_type " + str(command_type) +
                             "  chassis_speeds " + str(chassis_speeds) +
                             "  target_pose " + str(target_pose) +
                             "  named_action '" + str(named_action) + "'")
        try:
            success = self.writeCommandRequestFunction(request_id, command_type,
                                                       chassis_speeds, target_pose,
                                                       named_action)
        except Exception as e:
            self.msg_if.pub_warn("Failed to write RBX command request: " + str(e))
            return False
        if success is False:
            self.msg_if.pub_warn("RBX command request " + str(request_id) +
                                 " (" + str(request_type) + ") was not written")
        return bool(success)

    ##########################
    # RBX Interface Functions

    def getStateInd(self):
        # No robot states. RBXRobotIF calls this unconditionally and displays
        # "Not Set" for the empty list.
        return 0

    def setStateInd(self, state_ind):
        # Unreachable with an empty states list (RBXRobotIF bounds-checks first)
        return False

    def getModeInd(self):
        return 0

    def setModeInd(self, mode_ind):
        # Unreachable with an empty modes list
        return False

    def checkStopFunction(self):
        triggered = self.stop_triggered
        self.stop_triggered = False
        return triggered

    def manualControlsReady(self):
        # Gates per-motor manual control on a live robot network connection.
        # Fresh pose is not required: a direct motor command does not depend on
        # knowing where the robot is. Must stay a real callable whenever
        # setMotorControlRatio is -- RBXRobotIF.setMotorControl calls it
        # unconditionally.
        try:
            return bool(self.getConnectedFunction())
        except Exception:
            return False

    def autonomousControlsReady(self):
        # Gates every goto: a live connection AND a currently valid fused
        # navpose, because RBXRobotIF's goto is a blocking convergence loop that
        # measures its own progress against that pose. Must stay a real callable
        # -- gotoPoseCb and gotoLocationCb call it unconditionally.
        try:
            if bool(self.getConnectedFunction()) is False:
                return False
        except Exception:
            return False
        return self.getNavPoseCb() is not None

    def getMotorControlRatios(self):
        self.syncMotorRatios()
        return self.motor_ratios

    def setMotorControlRatio(self, motor_ind, speed_ratio):
        # Maps a NEPI motor index (slot order) to a RoboRIO motor_id through the
        # mapping and writes the command out. A command aimed at a slot with no
        # mapped motor_id is rejected with a warning and nothing is written.
        #
        # Logged on the way in, before any of the rejections below, so what
        # RBXRobotIF asked for is on the record next to what the app went on to
        # write -- including the cases where it wrote nothing. Command entry
        # points are operator-driven, not periodic, so this is not a log flood.
        self.logCommandEntry("set_motor_control_ratio", "motor_ind " + str(motor_ind) +
                             "  speed_ratio " + str(speed_ratio))
        self.syncMotorRatios()
        slots = self.getSlots()
        if motor_ind < 0 or motor_ind >= len(slots):
            self.msg_if.pub_warn("Motor control ignored: slot " + str(motor_ind) +
                                 " is out of range 0-" + str(len(slots) - 1))
            return
        motor_id = slots[motor_ind].get('motor_id', -1)
        if motor_id is None or int(motor_id) < 0:
            self.msg_if.pub_warn("Motor control ignored: slot " + str(motor_ind) +
                                 " has no RoboRIO motor_id mapped")
            return

        speed_ratio = max(0.0, min(1.0, float(speed_ratio)))
        self.motor_ratios[motor_ind] = speed_ratio
        try:
            self.setMotorCommandFunction(int(motor_id), speed_ratio)
        except Exception as e:
            self.msg_if.pub_warn("Failed to write motor command for slot " +
                                 str(motor_ind) + ": " + str(e))

    def goStop(self):
        self.logCommandEntry("go_stop", "zeroing " + str(len(self.motor_ratios)) +
                             " motor ratios")
        self.stop_triggered = True
        for i in range(len(self.motor_ratios)):
            self.motor_ratios[i] = 0.0
        return self.sendCommandRequest(self.command_types['stop'],
                                       chassis_speeds=dict(velocity_x_mps=0.0,
                                                           velocity_y_mps=0.0,
                                                           angular_velocity_radps=0.0),
                                       request_type='Stop')

    def goHome(self):
        self.logCommandEntry("go_home", "named action " + CAPABILITY_GO_HOME)
        return self.sendCommandRequest(self.command_types['named_action'],
                                       named_action=CAPABILITY_GO_HOME,
                                       request_type='Go Home')

    def gotoPose(self, attitude_enu_degs):
        # RBXRobotIF passes an absolute target attitude ENU [roll, pitch, yaw]
        # in degrees. target_pose carries heading only, so yaw is what is sent
        # and the robot holds its current x/y -- taken from the fused navpose so
        # the RoboRIO is given an absolute pose, not an offset.
        self.logCommandEntry("goto_pose", "attitude ENU degs " + str(list(attitude_enu_degs)))
        navpose_dict = self.getNavPoseCb()
        if navpose_dict is None:
            self.msg_if.pub_warn("GoTo Pose ignored: no valid robot pose")
            return False
        target_pose = dict(x_m=navpose_dict['x_m'],
                           y_m=navpose_dict['y_m'],
                           heading_rad=math.radians(attitude_enu_degs[2]))
        return self.sendCommandRequest(self.command_types['target_pose'],
                                       target_pose=target_pose,
                                       request_type='GoTo Pose')

    def gotoPosition(self, point_enu_m, orientation_enu_deg):
        # RBXRobotIF passes the goal as an ENU OFFSET point from the current
        # position plus an absolute target orientation (its own convergence
        # check adds current + offset the same way), so the absolute target is
        # computed here from the same fused navpose it measures against. z is
        # dropped: target_pose has no vertical field.
        self.logCommandEntry("goto_position",
                             "offset ENU m [" + str(point_enu_m.x) + ", " +
                             str(point_enu_m.y) + ", " + str(point_enu_m.z) + "]" +
                             "  orientation ENU degs " + str(list(orientation_enu_deg)))
        navpose_dict = self.getNavPoseCb()
        if navpose_dict is None:
            self.msg_if.pub_warn("GoTo Position ignored: no valid robot pose")
            return False
        target_pose = dict(x_m=navpose_dict['x_m'] + point_enu_m.x,
                           y_m=navpose_dict['y_m'] + point_enu_m.y,
                           heading_rad=math.radians(orientation_enu_deg[2]))
        return self.sendCommandRequest(self.command_types['target_pose'],
                                       target_pose=target_pose,
                                       request_type='GoTo Position')

    def setGoActionInd(self, action_ind):
        # action_ind is already bounds-checked against go_actions by RBXRobotIF.
        go_actions = self.getGoActions()
        action = go_actions[action_ind]
        self.logCommandEntry("set_go_action", "action_ind " + str(action_ind) +
                             "  action " + str(action))
        return self.sendCommandRequest(self.command_types['named_action'],
                                       named_action=action,
                                       request_type=action)

    def getNavPoseCb(self):
        try:
            return self.getNavPoseFunction()
        except Exception as e:
            self.msg_if.pub_warn("Failed to read navpose: " + str(e), throttle_s=10.0)
            return None

    ##########################
    # RBX Feedback -> device state

    def feedbackUpdateCb(self, timer):
        if self.rbx_if is None:
            return

        feedback = None
        try:
            feedback = self.getRbxFeedbackFunction()
        except Exception as e:
            self.msg_if.pub_warn("Failed to read RBX feedback: " + str(e), throttle_s=10.0)
        if feedback is None:
            return

        request_status = str(feedback.get('request_status', ''))
        status_message = str(feedback.get('status_message', ''))
        active_request_type = str(feedback.get('active_request_type', ''))

        # A newly reported failure becomes the device's last error message.
        # Edge-triggered so one failed request does not overwrite the field on
        # every tick.
        if request_status != self.last_request_status:
            if request_status in REQUEST_STATUS_FAILURES:
                message = status_message
                if message == '':
                    message = 'RoboRIO reported ' + request_status
                self.rbx_if.update_error_msg(message)
            self.last_request_status = request_status

        # process_current is only written while the device is idle. During a
        # goto, RBXRobotIF's own blocking convergence loop owns that field, and
        # two writers would fight over it.
        if self.get_device_ready_state() is False:
            return

        process_name = 'None'
        if request_status != '' and request_status != REQUEST_STATUS_IDLE:
            process_name = request_status
            if active_request_type != '':
                process_name = active_request_type + ': ' + request_status

        # Edge-triggered. This runs on every status tick, and RBXRobotIF logs a
        # line each time setProcessNameCb is called, so re-sending an unchanged
        # name (almost always 'None', since IDLE is the resting state) floods
        # the log at the status rate and drowns out the command tracing this
        # module exists to produce.
        if process_name == self.last_process_name:
            return
        self.last_process_name = process_name

        try:
            self.rbx_if.setProcessNameCb(String(data=process_name))
        except Exception as e:
            self.msg_if.pub_warn("Failed to set RBX process name: " + str(e),
                                 throttle_s=10.0)
