#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
#
# WPILib IF App -- TEST MODE
#
# TEMPORARY SCAFFOLDING. This whole file exists so the RBX command path can be
# exercised before the RoboRIO connection is finished, and it is meant to be
# deleted once it is. Everything that can live outside the app node lives here
# for exactly that reason.
#
# WHAT TEST MODE DOES
#
# It synthesizes the five NetworkTables INPUT groups -- motor feedback, robot
# position, robot velocity, robot orientation, RBX feedback -- into the same
# in-memory caches the NT read helpers fill, with the same dict shapes. Nothing
# downstream can tell the difference or needs to know. That makes the robot
# COMMANDABLE with no RoboRIO attached, which is the only thing standing between
# an operator and a live RBX device.
#
# The RBX Feedback group is the load-bearing one. updateRbxIF() builds the RBX
# device only when supported_capabilities is non-empty, so a test mode that
# faked only motor feedback would return early forever and never produce a
# device to command.
#
# WHAT TEST MODE DELIBERATELY DOES NOT DO
#
# It does not touch the OUTBOUND path. Motor commands and RBX command requests
# go to the real nepi_wpilib writers and the real NetworkTables keys, exactly as
# they would with a robot attached -- test mode only adds a log line as they go
# past. Confirming those writes are correct is the entire point of the exercise,
# and a test mode that short-circuited them is the one thing that could not
# confirm it.
#
# Nothing here responds to a command either. The pose holds still, the motors
# report zero output, and request status stays IDLE no matter what is sent. This
# is not a simulator and must not become one: synthetic telemetry that moved in
# response to a command would be this app confirming its own commands, which
# proves nothing about the robot. The visible consequence is that an RBX goto
# never converges and runs to cmd_timeout reporting cmd_success false. That is
# expected. The command still went out and was still logged, which is what is
# under test.
#
# HOW TO REMOVE TEST MODE COMPLETELY
#
# 1. Delete this file.
# 2. grep -rn "TEST MODE" scripts/ rui/ msg/ CMakeLists.txt and delete every
#    marked block. That includes the write-trace block in nepi_wpilib.py and the
#    writeField() hook that feeds it.
#    Every touch point outside this file carries that token in a comment.
# 3. Remove the "bool test_mode" field from msg/NepiAppWpilibIFStatus.msg and
#    rebuild the messages with catkin.
# 4. Rebuild the RUI so the toggle disappears from the app page.
# 5. Delete the test_mode param from any saved config yaml under the app's
#    config directory, or it will linger in the param server as an unread key.
#

import copy

from nepi_sdk import nepi_sdk

# Imported, never restated. These strings must match what RBXRobotIF matches on
# exactly -- a capability name that is merely close produces a device with no
# controls and no error. The dependency runs test mode -> rbx module and never
# the other way, so deleting this file removes it.
from wpilib_rbx_if import (CAPABILITY_GOTO_POSITION,
                           CAPABILITY_GOTO_POSE,
                           CAPABILITY_GO_HOME,
                           CAPABILITY_STOP,
                           CAPABILITY_MOTOR_CONTROL,
                           REQUEST_STATUS_IDLE)


# Off unless somebody turns it on. Persisted like any other app config, so a
# node that starts with it true warns loudly at startup rather than quietly
# reporting a robot that is not there.
FACTORY_TEST_MODE = False

# What the synthetic RoboRIO claims it can do. RBXRobotIF derives and CACHES its
# capability flags from which callbacks are non-None at construction, so this
# list decides which controls the RBX device advertises and therefore what can
# be exercised. Narrow it to test a smaller control surface.
TEST_MODE_CAPABILITY_NAMES = [CAPABILITY_GOTO_POSITION,
                              CAPABILITY_GOTO_POSE,
                              CAPABILITY_GO_HOME,
                              CAPABILITY_STOP,
                              CAPABILITY_MOTOR_CONTROL]

# Where the synthetic robot sits, and stays. A couple of metres onto the field,
# level, facing 45 degrees. Units and frames are the NetworkTables groups' own
# -- metres and radians.
#
# roll and pitch are exactly zero, not merely small. RBXRobotIF's attitude
# convergence loop compares all three axes, and the RBX Command Request carries
# heading only, so a non-zero synthetic roll or pitch could never be commanded
# away.
TEST_MODE_START_POSE = dict(x_m=2.0,
                            y_m=1.0,
                            z_m=0.0,
                            heading_rad=0.785398,
                            roll_rad=0.0,
                            pitch_rad=0.0,
                            yaw_rad=0.785398)

# The RoboRIO's own display name per synthetic motor, so the readout is
# obviously not a real controller's name.
TEST_MODE_MOTOR_NAME_PREFIX = "test_motor_"
TEST_MODE_MOTOR_CONTROL_MODE = "PERCENT_OUTPUT"

# Carried in the synthetic RBX Feedback group, so anything displaying the
# RoboRIO's status message says where the value came from.
TEST_MODE_STATUS_MESSAGE = "synthetic: test mode, no robot connected"

# Nothing on the other end executes anything, so the request never leaves IDLE.
# Reporting COMPLETE would be a fabricated acknowledgement: an operator watching
# the RUI would read it as the robot having done the thing. IDLE is also not in
# REQUEST_STATUS_FAILURES, so holding here logs no spurious failures.
TEST_MODE_REQUEST_STATUS = REQUEST_STATUS_IDLE


class WpilibTestMode:
    """Synthetic NetworkTables input telemetry for the WPILib IF app.

    Produces the five NT input groups in the same dict shapes the real
    nepi_wpilib read helpers produce, so the app's caches can be filled from
    here instead of from a robot. Holds no reference to the app node, no
    NetworkTables client, and no RBX interface.

    Never touches the outbound command path -- see the removal notes at the top
    of this file.
    """

    pose = None
    request_id = ''
    request_type = ''

    def __init__(self, msg_if=None):
        self.msg_if = msg_if
        self.pose = copy.deepcopy(TEST_MODE_START_POSE)
        self.request_id = ''
        self.request_type = ''

    def start(self):
        """Reset synthetic state to its starting values.

        Called when test mode is switched on, and at node startup when the
        persisted param comes back true.
        """
        self.pose = copy.deepcopy(TEST_MODE_START_POSE)
        self.request_id = ''
        self.request_type = ''

    def note_command_request(self, request_id, command_type_name):
        """Record the last RBX Command Request the app sent.

        Echoed back in the synthetic RBX Feedback group so the active request is
        visible. Recording a request does NOT advance its status: the request
        stays IDLE because nothing executed it.

        Args:
            request_id: Request id the app assigned, any type; stored as str.
            command_type_name: Readable command type name.
        """
        self.request_id = str(request_id)
        self.request_type = str(command_type_name)

    def build_groups(self, motor_ids, slot_count, unmapped_motor_id):
        """Build all five synthetic NT input groups.

        Every group is stamped with the current time and age_s 0.0 on each call,
        so the caches stay inside the app's staleness bounds for as long as test
        mode is polling them.

        Args:
            motor_ids: Ordered per-slot RoboRIO motor_id list, as configured.
            slot_count: Number of motor slots configured.
            unmapped_motor_id: Sentinel meaning a slot is unmapped.

        Returns:
            dict: Keys motor_feedback, position, velocity, orientation and
            rbx_feedback, each holding one group dict shaped exactly as the
            matching nepi_wpilib read helper returns it.
        """
        if self.pose is None:
            self.pose = copy.deepcopy(TEST_MODE_START_POSE)
        now = nepi_sdk.get_time()
        return dict(motor_feedback=self.buildMotorFeedback(motor_ids, slot_count,
                                                           unmapped_motor_id, now),
                    position=self.buildPosition(now),
                    velocity=self.buildVelocity(now),
                    orientation=self.buildOrientation(now),
                    rbx_feedback=self.buildRbxFeedback(now))

    # One entry per MAPPED slot, keyed by RoboRIO motor_id exactly as
    # read_all_motor_feedback keys it. An unmapped slot gets nothing, so the
    # unmapped sentinel still turns a slot off in test mode and the discovered-id
    # readout still shows only the ids that were configured.
    def buildMotorFeedback(self, motor_ids, slot_count, unmapped_motor_id, now):
        feedback_dict = dict()
        for slot in range(slot_count):
            if slot >= len(motor_ids):
                continue
            motor_id = int(motor_ids[slot])
            if motor_id == unmapped_motor_id or motor_id < 0:
                continue
            # Zeros, never an echo of what was commanded. Feeding a commanded
            # ratio back as this motor's measured_output would make the readout
            # confirm a command the RoboRIO never received. The slot reports
            # present and enabled; it does not report having done anything.
            feedback_dict[motor_id] = dict(motor_id=motor_id,
                                           motor_name=TEST_MODE_MOTOR_NAME_PREFIX + str(motor_id),
                                           control_mode=TEST_MODE_MOTOR_CONTROL_MODE,
                                           commanded_output=0.0,
                                           measured_output=0.0,
                                           position=0.0,
                                           velocity=0.0,
                                           current_amps=0.0,
                                           timestamp=now,
                                           time_local=now,
                                           age_s=0.0)
        return feedback_dict

    def buildPosition(self, now):
        return dict(x_m=self.pose['x_m'],
                    y_m=self.pose['y_m'],
                    z_m=self.pose['z_m'],
                    heading_rad=self.pose['heading_rad'],
                    timestamp=now,
                    valid=True,
                    time_local=now,
                    age_s=0.0)

    # A robot that is not moving. Zero velocity is the only reading coherent
    # with a pose that holds still.
    def buildVelocity(self, now):
        return dict(velocity_x_mps=0.0,
                    velocity_y_mps=0.0,
                    velocity_z_mps=0.0,
                    angular_velocity_radps=0.0,
                    timestamp=now,
                    valid=True,
                    time_local=now,
                    age_s=0.0)

    def buildOrientation(self, now):
        return dict(roll_rad=self.pose['roll_rad'],
                    pitch_rad=self.pose['pitch_rad'],
                    yaw_rad=self.pose['yaw_rad'],
                    roll_rate_radps=0.0,
                    pitch_rate_radps=0.0,
                    yaw_rate_radps=0.0,
                    timestamp=now,
                    valid=True,
                    time_local=now,
                    age_s=0.0)

    # The group updateRbxIF gates on. supported_capabilities MUST be non-empty
    # or no RBX device is ever built, which is the whole reason test mode
    # synthesizes this group and not just the motors.
    def buildRbxFeedback(self, now):
        return dict(supported_capabilities=list(TEST_MODE_CAPABILITY_NAMES),
                    active_request_id=self.request_id,
                    active_request_type=self.request_type,
                    request_status=TEST_MODE_REQUEST_STATUS,
                    status_message=TEST_MODE_STATUS_MESSAGE,
                    timestamp=now,
                    time_local=now,
                    age_s=0.0)
