#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# NetworkTables access layer for the NEPI WPILib IF app.
#
# This module is the ONLY place in the app that imports ntcore. The app node
# owns one client for the process and reads and writes the robot's telemetry
# groups through the typed helpers below; the RBX module never sees ntcore at
# all (it is handed injected callables instead -- see docs/WPILIB_IF_DESIGN.md,
# Decision 3).
#
# It also still runs standalone for bring-up testing:
#   nepi_wpilib.py --rio        real RoboRIO heartbeat test (needs a robot)
#   nepi_wpilib.py --loopback   two NT instances in this process, no hardware
#
# NT KEY PATHS ARE A CONTRACT. The project doc fixes the field names in each
# group and one concrete path -- the heartbeat at /NEPI/System/heartbeat. Every
# other group path follows that same /NEPI/<Group>/... shape and is declared
# once here. The RoboRIO code must use these same strings.

import os
import sys
import threading
import time


# Locate the NetworkTables packages stored inside the app folder.
#
# Two layouts have to work, so the candidates are tried in order rather than
# assumed. In a repo checkout this file sits in <repo>/scripts and vendor/ is a
# sibling of scripts/. Deployed -- by either deploy_app.sh (which syncs scripts/
# into /opt/nepi/nepi_engine/lib/<app>/) or a catkin install (which installs the
# scripts into the same lib/<app>/ directory) -- there is no scripts/ level left,
# so vendor/ sits beside this file. CMakeLists installs vendor/ into
# ${CATKIN_PACKAGE_BIN_DESTINATION}/vendor to make that second candidate real.
script_dir = os.path.dirname(os.path.abspath(__file__))
app_dir = os.path.dirname(script_dir)

VENDOR_SUBDIR = os.path.join("vendor", "py38_aarch64")

VENDOR_DIR_CANDIDATES = [
    os.path.join(script_dir, VENDOR_SUBDIR),
    os.path.join(app_dir, VENDOR_SUBDIR),
]


def setupVendorPath():
    for candidate in VENDOR_DIR_CANDIDATES:
        if os.path.isdir(candidate):
            if candidate not in sys.path:
                sys.path.insert(0, candidate)
            return candidate
    return None


vendor_dir = setupVendorPath()

import ntcore


TEAM_NUMBER = 9023
CLIENT_NAME = "nepi-wpilib-interface"

HEARTBEAT_TOPIC = "/NEPI/System/heartbeat"
HEARTBEAT_PERIOD_SECONDS = 1.0

# How long the responder waits after observing the heartbeat True before
# setting it back to False. The one delay constant for the heartbeat handshake:
# the RoboRIO test measures against it and the loopback responder honours it.
HEARTBEAT_RESPOND_DELAY_SECONDS = 0.5


#########################################
# NetworkTables group paths
#########################################

# RoboRIO -> NEPI. One subtable per motor, keyed by the RoboRIO motor_id, so a
# motor that is not on the bus has no subtable at all -- which is what lets a
# slot report "never seen" rather than reporting zeros as real telemetry.
MOTOR_FEEDBACK_TABLE = "/NEPI/Motors"

# NEPI -> RoboRIO. Per-motor manual control. NOT one of the groups in the
# project doc: the doc's only output group (RBX Command Request) is
# chassis-level, but RBXRobotIF exposes per-motor manual control through
# set_motor_control, so that command needs a per-motor key to land on. The
# RoboRIO implementing this group is a stated dependency, not an assumption --
# see docs/WPILIB_IF_DESIGN.md.
MOTOR_COMMAND_TABLE = "/NEPI/MotorCommand"

# RoboRIO -> NEPI
POSITION_TABLE = "/NEPI/Position"
VELOCITY_TABLE = "/NEPI/Velocity"
ORIENTATION_TABLE = "/NEPI/Orientation"
RBX_FEEDBACK_TABLE = "/NEPI/RBX/Feedback"

# NEPI -> RoboRIO
RBX_COMMAND_TABLE = "/NEPI/RBX/Command"

# Nested field prefixes inside the RBX Command Request group.
RBX_COMMAND_CHASSIS_PREFIX = "chassis_speeds"
RBX_COMMAND_POSE_PREFIX = "target_pose"

# command_type codes carried in an RBX Command Request. Also part of the
# contract this module declares.
COMMAND_TYPE_CHASSIS_SPEEDS = 1
COMMAND_TYPE_TARGET_POSE = 2
COMMAND_TYPE_NAMED_ACTION = 3
COMMAND_TYPE_STOP = 4

# Local loopback test ports. Deliberately not the NT defaults (1735 / 5810), so
# a loopback test can never be mistaken for, or interfere with, a real robot
# connection on the same host.
LOOPBACK_HOST = "127.0.0.1"
LOOPBACK_PORT_NT3 = 11735
LOOPBACK_PORT_NT4 = 15810


#########################################
# Group field specs
#########################################

# (field name, kind, default). Kind drives which typed NetworkTable getter is
# used. These lists are the single description of each group's shape: the read
# helpers, the freshness snapshots and the loopback test all walk them.

_POSITION_FIELDS = [
    ("x_m", "float", 0.0),
    ("y_m", "float", 0.0),
    ("z_m", "float", 0.0),
    ("heading_rad", "float", 0.0),
    ("timestamp", "float", 0.0),
    ("valid", "bool", False),
]

_VELOCITY_FIELDS = [
    ("velocity_x_mps", "float", 0.0),
    ("velocity_y_mps", "float", 0.0),
    ("velocity_z_mps", "float", 0.0),
    ("angular_velocity_radps", "float", 0.0),
    ("timestamp", "float", 0.0),
    ("valid", "bool", False),
]

_ORIENTATION_FIELDS = [
    ("roll_rad", "float", 0.0),
    ("pitch_rad", "float", 0.0),
    ("yaw_rad", "float", 0.0),
    ("roll_rate_radps", "float", 0.0),
    ("pitch_rate_radps", "float", 0.0),
    ("yaw_rate_radps", "float", 0.0),
    ("timestamp", "float", 0.0),
    ("valid", "bool", False),
]

# The RBX Feedback group defines no valid flag, so freshness is the only
# liveness signal it carries.
_RBX_FEEDBACK_FIELDS = [
    ("supported_capabilities", "str_list", []),
    ("active_request_id", "str", ""),
    ("active_request_type", "str", ""),
    ("request_status", "str", ""),
    ("status_message", "str", ""),
    ("timestamp", "float", 0.0),
]

# Per-motor feedback. Also carries no valid flag: the presence of the motor's
# subtable is what says the motor exists, and freshness says whether it is live.
_MOTOR_FEEDBACK_FIELDS = [
    ("motor_id", "int", -1),
    ("motor_name", "str", ""),
    ("control_mode", "str", ""),
    ("commanded_output", "float", 0.0),
    ("measured_output", "float", 0.0),
    ("position", "float", 0.0),
    ("velocity", "float", 0.0),
    ("current_amps", "float", 0.0),
    ("timestamp", "float", 0.0),
]


#########################################
# Freshness tracking
#########################################

# NT carries a server-side timestamp per value, but the RoboRIO's own group
# timestamp is the sample time the robot code reports and the two are not the
# same clock domain in every deployment. Freshness here is therefore measured
# locally and unambiguously: when a group's field set changes, the change is
# stamped with this host's clock, and age is measured from that stamp. Keyed on
# the instance so the two loopback instances never share a stamp.
_group_snapshots = dict()
_group_snapshot_lock = threading.Lock()

# One prefix subscriber per instance, kept alive for the life of the client. An
# NT4 client is only told about topics its subscriptions cover, so without this
# the motor subtables are invisible to getSubTables() and no motor could ever be
# discovered no matter how long the client stayed connected. Held at module level
# because dropping the reference unsubscribes.
_motor_discovery_subs = dict()


def nowSeconds():
    return time.time()


def groupCacheKey(nt_instance, group_key):
    return (id(nt_instance), group_key)


def markGroupChange(nt_instance, group_key, snapshot):
    key = groupCacheKey(nt_instance, group_key)
    now = nowSeconds()
    with _group_snapshot_lock:
        entry = _group_snapshots.get(key)
        if entry is None or entry[0] != snapshot:
            _group_snapshots[key] = (snapshot, now)
            return now
        return entry[1]


def forgetGroupChanges(nt_instance):
    instance_id = id(nt_instance)
    with _group_snapshot_lock:
        for key in list(_group_snapshots.keys()):
            if key[0] == instance_id:
                del _group_snapshots[key]
    if instance_id in _motor_discovery_subs:
        del _motor_discovery_subs[instance_id]


def ensureMotorDiscovery(nt_instance):
    instance_id = id(nt_instance)
    if instance_id in _motor_discovery_subs:
        return
    try:
        _motor_discovery_subs[instance_id] = ntcore.MultiSubscriber(
            nt_instance, [MOTOR_FEEDBACK_TABLE + "/"])
    except Exception:
        pass


#########################################
# Typed group access
#########################################

def groupTable(nt_instance, table_path):
    return nt_instance.getTable(table_path)


def readField(table, name, kind, default):
    try:
        if kind == "float":
            return float(table.getNumber(name, float(default)))
        if kind == "int":
            return int(table.getNumber(name, float(default)))
        if kind == "bool":
            return bool(table.getBoolean(name, bool(default)))
        if kind == "str":
            return str(table.getString(name, str(default)))
        if kind == "str_list":
            return [str(v) for v in table.getStringArray(name, list(default))]
    except Exception:
        pass
    return default


def readGroup(nt_instance, table_path, fields, group_key):
    # Reads one group into a plain dict, then stamps it with the local time of
    # the last observed change and the age derived from it. A group whose fields
    # are all absent reads as its defaults, which is why the caller must look at
    # 'valid' (where the group has one) or 'age_s' before trusting the values.
    #
    # SUBSCRIPTION PRIMING. NT4 delivers a value only to an existing subscriber,
    # and the NetworkTable entry behind each field is created on its first read,
    # so the FIRST read of a group returns defaults even when the RoboRIO has
    # already published it. NetworkTable caches its entries, so every read after
    # that sees real values. The app polls at NT_POLL_RATE_HZ, which makes the
    # cost one poll period at startup, and the valid/age gating means that first
    # read is correctly reported as invalid rather than as a set of zeros.
    table = groupTable(nt_instance, table_path)
    group_dict = dict()
    for (name, kind, default) in fields:
        group_dict[name] = readField(table, name, kind, default)

    snapshot = tuple(
        tuple(group_dict[name]) if isinstance(group_dict[name], list) else group_dict[name]
        for (name, _kind, _default) in fields
    )
    change_time = markGroupChange(nt_instance, group_key, snapshot)

    group_dict["time_local"] = change_time
    group_dict["age_s"] = max(0.0, nowSeconds() - change_time)
    return group_dict


def writeField(table, name, kind, value):
    try:
        if kind == "float":
            return bool(table.putNumber(name, float(value)))
        if kind == "int":
            return bool(table.putNumber(name, float(int(value))))
        if kind == "bool":
            return bool(table.putBoolean(name, bool(value)))
        if kind == "str":
            return bool(table.putString(name, str(value)))
        if kind == "str_list":
            return bool(table.putStringArray(name, [str(v) for v in value]))
    except Exception:
        return False
    return False


def parseMotorId(name):
    try:
        return int(str(name).strip())
    except Exception:
        return None


def motorTablePath(table_root, motor_id):
    return table_root + "/" + str(int(motor_id))


#########################################
# Public: module utility
#########################################

def log(message):
    """Print a standalone test message."""
    print(message)


def get_vendor_dir():
    """Return the vendored ntcore directory that was added to sys.path.

    Returns:
        str: The vendor directory in use, or None if ntcore was imported from
            somewhere else (a system or user install).
    """
    return vendor_dir


#########################################
# Public: connection lifecycle
#########################################

def start_client(team_number=TEAM_NUMBER, client_name=CLIENT_NAME, nt_instance=None):
    """Start an NT4 client and point it at a team's RoboRIO.

    Returns immediately: NT connects on its own threads, so this never blocks
    the caller. Use is_connected() or add_connection_callback() to learn when
    the link is up.

    Args:
        team_number (int): FRC team number, used to derive the robot address.
        client_name (str): NT client identity this process announces.
        nt_instance (ntcore.NetworkTableInstance): Instance to start. Defaults
            to the process-default instance.

    Returns:
        ntcore.NetworkTableInstance: The started instance.
    """
    if nt_instance is None:
        nt_instance = ntcore.NetworkTableInstance.getDefault()
    nt_instance.startClient4(str(client_name))
    nt_instance.setServerTeam(int(team_number))
    return nt_instance


def set_server_team(nt_instance, team_number):
    """Repoint a running client at a different team's RoboRIO.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started client instance.
        team_number (int): FRC team number to connect to.

    Returns:
        bool: True if the new server was accepted, False on failure.
    """
    try:
        nt_instance.setServerTeam(int(team_number))
        return True
    except Exception:
        return False


def stop_client(nt_instance):
    """Stop a client and drop its cached freshness state.

    Args:
        nt_instance (ntcore.NetworkTableInstance): The instance to stop.

    Returns:
        bool: True if the client stopped cleanly, False otherwise.
    """
    success = True
    try:
        nt_instance.stopClient()
    except Exception:
        success = False
    forgetGroupChanges(nt_instance)
    return success


def is_connected(nt_instance):
    """Return the live NetworkTables connection state.

    Args:
        nt_instance (ntcore.NetworkTableInstance): The instance to query.

    Returns:
        bool: True while the instance has at least one live connection.
    """
    if nt_instance is None:
        return False
    try:
        return bool(nt_instance.isConnected())
    except Exception:
        return False


def add_connection_callback(nt_instance, callback):
    """Register a callback invoked whenever the connection state changes.

    The callback is called with a single bool argument (True on connect, False
    on disconnect) from an NT listener thread, not from the caller's thread, so
    it must not block and must guard any shared state it touches.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.
        callback (callable): Called as callback(connected: bool).

    Returns:
        int: Listener handle for remove_connection_callback(), or None if the
            listener could not be registered.
    """
    def connectionEventCb(event):
        connected = is_connected(nt_instance)
        try:
            flags = int(event.flags)
            if flags & int(ntcore.EventFlags.kConnected):
                connected = True
            elif flags & int(ntcore.EventFlags.kDisconnected):
                connected = False
        except Exception:
            pass
        try:
            callback(connected)
        except Exception:
            pass

    try:
        return nt_instance.addConnectionListener(True, connectionEventCb)
    except Exception:
        return None


def remove_connection_callback(handle):
    """Unregister a connection callback.

    Args:
        handle (int): Handle returned by add_connection_callback().

    Returns:
        bool: True if the listener was removed.
    """
    if handle is None:
        return False
    try:
        ntcore.NetworkTableInstance.removeListener(handle)
        return True
    except Exception:
        return False


def flush(nt_instance):
    """Flush pending NetworkTables writes immediately.

    Args:
        nt_instance (ntcore.NetworkTableInstance): The instance to flush.

    Returns:
        bool: True if the flush was issued.
    """
    try:
        nt_instance.flush()
        return True
    except Exception:
        return False


def wait_for_connection(nt_instance, timeout_seconds=10.0):
    """Block until a NetworkTables connection is up or the timeout expires.

    BLOCKING. For standalone test use only -- a ROS node must use
    is_connected() or add_connection_callback() instead.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.
        timeout_seconds (float): Maximum seconds to wait.

    Returns:
        bool: True if connected within the timeout.
    """
    start_time = time.monotonic()

    while not is_connected(nt_instance):
        elapsed_time = time.monotonic() - start_time

        if elapsed_time >= timeout_seconds:
            return False

        time.sleep(0.1)

    return True


def connect_to_roborio(team_number=TEAM_NUMBER):
    """Start a client and wait for the RoboRIO to answer.

    BLOCKING. Used by the standalone RoboRIO test; the app node calls
    start_client() instead.

    Args:
        team_number (int): FRC team number to connect to.

    Returns:
        ntcore.NetworkTableInstance: The connected instance, or None if the
            RoboRIO did not answer within the timeout.
    """
    nt_instance = start_client(team_number=team_number)

    log(
        "Waiting for Team {} RoboRIO NetworkTables connection..."
        .format(team_number)
    )

    connected = wait_for_connection(
        nt_instance,
        timeout_seconds=10.0,
    )

    if not connected:
        log("Could not connect to the RoboRIO.")
        stop_client(nt_instance)
        return None

    log("Connected to RoboRIO NetworkTables.")

    return nt_instance


#########################################
# Public: generic entry helpers
#########################################

def get_boolean_entry(nt_instance, topic_name, default_value=False):
    """Create a Boolean entry that can be read and written.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.
        topic_name (str): Full NT topic path.
        default_value (bool): Value reported before any value is published.

    Returns:
        ntcore.BooleanEntry: The read/write entry.
    """
    return (
        nt_instance
        .getBooleanTopic(topic_name)
        .getEntry(bool(default_value))
    )


def write_boolean(boolean_entry, value):
    """Write a Boolean value to NetworkTables.

    Args:
        boolean_entry (ntcore.BooleanEntry): Target entry.
        value (bool): Value to write.
    """
    boolean_entry.set(bool(value))


def read_boolean(boolean_entry):
    """Read a Boolean value from NetworkTables.

    Args:
        boolean_entry (ntcore.BooleanEntry): Source entry.

    Returns:
        bool: The current value.
    """
    return bool(boolean_entry.get())


#########################################
# Public: heartbeat
#########################################

def get_heartbeat_entry(nt_instance):
    """Create the read/write entry for the NEPI system heartbeat.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.

    Returns:
        ntcore.BooleanEntry: Entry for HEARTBEAT_TOPIC.
    """
    return get_boolean_entry(nt_instance, HEARTBEAT_TOPIC, False)


def publish_heartbeat(nt_instance, heartbeat_entry):
    """Set the heartbeat True and flush it, the NEPI side of the handshake.

    Args:
        nt_instance (ntcore.NetworkTableInstance): Instance owning the entry.
        heartbeat_entry (ntcore.BooleanEntry): The heartbeat entry.

    Returns:
        bool: True if the write was issued.
    """
    try:
        write_boolean(heartbeat_entry, True)
    except Exception:
        return False
    flush(nt_instance)
    return True


def respond_to_heartbeat(nt_instance, heartbeat_entry):
    """Answer an observed heartbeat: wait 500 ms, then set it False.

    This is the responder half of the handshake -- what the RoboRIO does in
    production, and what the loopback test's RoboRIO stand-in does. Returns
    immediately without writing anything if the heartbeat is not currently True.

    BLOCKING for HEARTBEAT_RESPOND_DELAY_SECONDS while it holds the True value
    long enough for the other side to observe it. Call it from the standalone
    test, or from a dedicated thread -- never from the ROS node's main thread.

    Args:
        nt_instance (ntcore.NetworkTableInstance): Instance owning the entry.
        heartbeat_entry (ntcore.BooleanEntry): The heartbeat entry.

    Returns:
        bool: True if a heartbeat was observed and answered with False.
    """
    try:
        if read_boolean(heartbeat_entry) is not True:
            return False
    except Exception:
        return False

    time.sleep(HEARTBEAT_RESPOND_DELAY_SECONDS)

    try:
        write_boolean(heartbeat_entry, False)
    except Exception:
        return False
    flush(nt_instance)
    return True


#########################################
# Public: input group reads
#########################################

def read_robot_position(nt_instance):
    """Read the Robot Position input group.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.

    Returns:
        dict: x_m, y_m, z_m, heading_rad, timestamp, valid, plus time_local and
            age_s giving the local time of the last observed change and its age.
    """
    return readGroup(nt_instance, POSITION_TABLE, _POSITION_FIELDS, "position")


def read_robot_velocity(nt_instance):
    """Read the Robot Velocity input group.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.

    Returns:
        dict: velocity_x_mps, velocity_y_mps, velocity_z_mps,
            angular_velocity_radps, timestamp, valid, plus time_local and age_s.
    """
    return readGroup(nt_instance, VELOCITY_TABLE, _VELOCITY_FIELDS, "velocity")


def read_robot_orientation(nt_instance):
    """Read the Robot Orientation input group.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.

    Returns:
        dict: roll_rad, pitch_rad, yaw_rad, roll_rate_radps, pitch_rate_radps,
            yaw_rate_radps, timestamp, valid, plus time_local and age_s.
    """
    return readGroup(nt_instance, ORIENTATION_TABLE, _ORIENTATION_FIELDS, "orientation")


def read_rbx_feedback(nt_instance):
    """Read the RBX Feedback input group.

    This group defines no valid flag, so age_s is the only liveness signal it
    carries.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.

    Returns:
        dict: supported_capabilities, active_request_id, active_request_type,
            request_status, status_message, timestamp, plus time_local and age_s.
    """
    return readGroup(nt_instance, RBX_FEEDBACK_TABLE, _RBX_FEEDBACK_FIELDS, "rbx_feedback")


def read_motor_ids(nt_instance):
    """Return the RoboRIO motor_ids currently present on NetworkTables.

    A motor is present when it has a subtable under MOTOR_FEEDBACK_TABLE. Names
    that are not integers are ignored rather than treated as motors.

    The first call establishes the prefix subscription that makes those subtables
    visible to this client at all, so it returns an empty list until the next
    network round trip.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.

    Returns:
        list: Sorted list of int motor_ids, empty if none are published.
    """
    ensureMotorDiscovery(nt_instance)
    try:
        sub_tables = groupTable(nt_instance, MOTOR_FEEDBACK_TABLE).getSubTables()
    except Exception:
        return []
    # Deduplicated: getSubTables() returns one entry per key beneath the
    # subtable, not one per subtable, so a motor publishing nine fields appears
    # nine times.
    motor_ids = set()
    for name in sub_tables:
        motor_id = parseMotorId(name)
        if motor_id is not None:
            motor_ids.add(motor_id)
    return sorted(motor_ids)


def read_motor_feedback(nt_instance, motor_id):
    """Read the Motor-Control Feedback input group for one RoboRIO motor.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.
        motor_id (int): RoboRIO motor id to read.

    Returns:
        dict: motor_id, motor_name, control_mode, commanded_output,
            measured_output, position, velocity, current_amps, timestamp, plus
            time_local and age_s. None if this motor has never appeared on
            NetworkTables, which is what lets an unmapped or absent motor report
            as not enabled instead of reporting zeros as real telemetry.
    """
    try:
        motor_id = int(motor_id)
    except Exception:
        return None
    if motor_id not in read_motor_ids(nt_instance):
        return None

    table_path = motorTablePath(MOTOR_FEEDBACK_TABLE, motor_id)
    group_dict = readGroup(nt_instance, table_path, _MOTOR_FEEDBACK_FIELDS,
                           "motor_" + str(motor_id))
    # The RoboRIO echoes motor_id inside the group; the subtable name is
    # authoritative, so a mismatched or unset echo is corrected here.
    group_dict["motor_id"] = motor_id
    return group_dict


def read_all_motor_feedback(nt_instance):
    """Read the Motor-Control Feedback group for every motor present.

    One call per poll cycle, so the app node does not enumerate the motor
    subtables once per slot.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.

    Returns:
        dict: motor_id (int) -> feedback dict, as returned by
            read_motor_feedback().
    """
    feedback_dict = dict()
    for motor_id in read_motor_ids(nt_instance):
        table_path = motorTablePath(MOTOR_FEEDBACK_TABLE, motor_id)
        group_dict = readGroup(nt_instance, table_path, _MOTOR_FEEDBACK_FIELDS,
                               "motor_" + str(motor_id))
        group_dict["motor_id"] = motor_id
        feedback_dict[motor_id] = group_dict
    return feedback_dict


#########################################
# Public: output group writes
#########################################

def write_rbx_command_request(nt_instance, request_id, command_type,
                              chassis_speeds=None, target_pose=None,
                              named_action="", timestamp=None):
    """Write one RBX Command Request to NetworkTables.

    Every field in the group is written on every request, with zeros and empty
    strings for the parts this command does not use, so the RoboRIO can never
    read a new command_type against a previous command's payload. request_id is
    written LAST and then flushed, so a RoboRIO that triggers on request_id
    changing always sees a complete request.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.
        request_id (int): Monotonically increasing id for this request.
        command_type (int): One of the COMMAND_TYPE_* constants.
        chassis_speeds (dict): velocity_x_mps, velocity_y_mps,
            angular_velocity_radps. Missing keys are written as 0.0.
        target_pose (dict): x_m, y_m, heading_rad. Missing keys are written
            as 0.0.
        named_action (str): Action name for COMMAND_TYPE_NAMED_ACTION.
        timestamp (float): Request time in seconds. Defaults to now.

    Returns:
        bool: True if the whole request was written.
    """
    if chassis_speeds is None:
        chassis_speeds = dict()
    if target_pose is None:
        target_pose = dict()
    if timestamp is None:
        timestamp = nowSeconds()

    table = groupTable(nt_instance, RBX_COMMAND_TABLE)
    success = True

    success = writeField(table, "command_type", "int", command_type) and success
    success = writeField(table, "named_action", "str", named_action) and success

    for name in ["velocity_x_mps", "velocity_y_mps", "angular_velocity_radps"]:
        field = RBX_COMMAND_CHASSIS_PREFIX + "/" + name
        success = writeField(table, field, "float",
                             chassis_speeds.get(name, 0.0)) and success

    for name in ["x_m", "y_m", "heading_rad"]:
        field = RBX_COMMAND_POSE_PREFIX + "/" + name
        success = writeField(table, field, "float",
                             target_pose.get(name, 0.0)) and success

    success = writeField(table, "timestamp", "float", timestamp) and success
    success = writeField(table, "request_id", "int", request_id) and success

    flush(nt_instance)
    return success


def write_motor_command(nt_instance, motor_id, speed_ratio, timestamp=None):
    """Write a per-motor speed command to NetworkTables.

    The Motor Command group is a NEPI-side addition to the project doc's group
    list -- see MOTOR_COMMAND_TABLE above for why it exists.

    Args:
        nt_instance (ntcore.NetworkTableInstance): A started instance.
        motor_id (int): RoboRIO motor id to command.
        speed_ratio (float): Speed magnitude, clamped to 0.0-1.0 to match
            nepi_interfaces/MotorControl.
        timestamp (float): Command time in seconds. Defaults to now.

    Returns:
        bool: True if the command was written.
    """
    try:
        motor_id = int(motor_id)
    except Exception:
        return False
    if timestamp is None:
        timestamp = nowSeconds()

    speed_ratio = max(0.0, min(1.0, float(speed_ratio)))

    table = groupTable(nt_instance, motorTablePath(MOTOR_COMMAND_TABLE, motor_id))
    success = writeField(table, "speed_ratio", "float", speed_ratio)
    success = writeField(table, "timestamp", "float", timestamp) and success
    flush(nt_instance)
    return success


#########################################
# Standalone test: real RoboRIO
#########################################

def run_roborio_heartbeat_test():
    """Run the real RoboRIO heartbeat test.

    NEPI writes True every second. The RoboRIO should detect True, wait about
    500 ms, and write the same topic back to False.

    Returns:
        int: 0 if the test ran, 1 if the RoboRIO could not be reached.
    """
    nt_instance = connect_to_roborio()

    if nt_instance is None:
        return 1

    heartbeat_entry = get_heartbeat_entry(nt_instance)

    next_heartbeat_time = time.monotonic()
    waiting_for_roborio = False
    sent_time = None

    log("Starting RoboRIO heartbeat test.")
    log("Topic: {}".format(HEARTBEAT_TOPIC))

    try:
        while True:
            current_time = time.monotonic()

            if current_time >= next_heartbeat_time:
                publish_heartbeat(nt_instance, heartbeat_entry)

                log("NEPI set heartbeat to True.")

                waiting_for_roborio = True
                sent_time = current_time

                next_heartbeat_time = (
                    current_time
                    + HEARTBEAT_PERIOD_SECONDS
                )

            heartbeat_value = read_boolean(heartbeat_entry)

            if waiting_for_roborio and heartbeat_value is False:
                delay = current_time - sent_time
                log("RoboRIO set heartbeat to False after {:.3f} s (expected ~{:.3f} s)."
                    .format(delay, HEARTBEAT_RESPOND_DELAY_SECONDS))
                waiting_for_roborio = False

            time.sleep(0.01)

    except KeyboardInterrupt:
        log("\nRoboRIO heartbeat test stopped.")

    finally:
        write_boolean(heartbeat_entry, False)

        flush(nt_instance)
        heartbeat_entry.close()
        stop_client(nt_instance)

        log("NetworkTables client stopped.")

    return 0


#########################################
# Standalone test: local two-instance loopback
#########################################

# What the loopback's RoboRIO stand-in publishes, so the typed read helpers are
# exercised against known values with no hardware.
LOOPBACK_POSITION = dict(x_m=1.5, y_m=-2.25, z_m=0.0, heading_rad=0.75,
                         timestamp=0.0, valid=True)
LOOPBACK_VELOCITY = dict(velocity_x_mps=0.5, velocity_y_mps=0.0,
                         velocity_z_mps=0.0, angular_velocity_radps=0.1,
                         timestamp=0.0, valid=True)
LOOPBACK_ORIENTATION = dict(roll_rad=0.01, pitch_rad=-0.02, yaw_rad=0.75,
                            roll_rate_radps=0.0, pitch_rate_radps=0.0,
                            yaw_rate_radps=0.1, timestamp=0.0, valid=True)
LOOPBACK_RBX_FEEDBACK = dict(supported_capabilities=["GOTO_POSE", "GO_HOME"],
                             active_request_id="0", active_request_type="NONE",
                             request_status="IDLE", status_message="idle",
                             timestamp=0.0)
LOOPBACK_MOTOR_IDS = [1, 2, 3, 4]

LOOPBACK_HEARTBEAT_CYCLES = 3
LOOPBACK_RESPOND_TIMEOUT_SEC = 3.0
LOOPBACK_RESPOND_SLACK_SEC = 0.5

# How long the test waits after priming a side's subscriptions before asserting
# on what it reads back. See the subscription-priming note in readGroup: the
# first read of a group creates the subscription and returns defaults, and the
# value arrives on the next network round trip. Generously sized so the test is
# not timing-fragile on a slow or emulated host.
LOOPBACK_PRIME_WAIT_SEC = 2.0


def loopbackResponderLoop(nt_instance, heartbeat_entry, stop_event):
    while not stop_event.is_set():
        respond_to_heartbeat(nt_instance, heartbeat_entry)
        time.sleep(0.01)


def primeGroupReads(nt_instance):
    # One read of every input group, which is what creates the NT4 subscriptions
    # behind them. Values arrive on the next round trip; see readGroup.
    read_robot_position(nt_instance)
    read_robot_velocity(nt_instance)
    read_robot_orientation(nt_instance)
    read_rbx_feedback(nt_instance)
    for motor_id in read_motor_ids(nt_instance):
        read_motor_feedback(nt_instance, motor_id)


def loopbackPublishGroups(nt_instance):
    now = nowSeconds()

    for (table_path, fields, values) in [
        (POSITION_TABLE, _POSITION_FIELDS, LOOPBACK_POSITION),
        (VELOCITY_TABLE, _VELOCITY_FIELDS, LOOPBACK_VELOCITY),
        (ORIENTATION_TABLE, _ORIENTATION_FIELDS, LOOPBACK_ORIENTATION),
        (RBX_FEEDBACK_TABLE, _RBX_FEEDBACK_FIELDS, LOOPBACK_RBX_FEEDBACK),
    ]:
        table = groupTable(nt_instance, table_path)
        for (name, kind, default) in fields:
            value = values.get(name, default)
            if name == "timestamp":
                value = now
            writeField(table, name, kind, value)

    for motor_id in LOOPBACK_MOTOR_IDS:
        table = groupTable(nt_instance, motorTablePath(MOTOR_FEEDBACK_TABLE, motor_id))
        for (name, kind, default) in _MOTOR_FEEDBACK_FIELDS:
            value = default
            if name == "motor_id":
                value = motor_id
            elif name == "motor_name":
                value = "roborio_motor_" + str(motor_id)
            elif name == "control_mode":
                value = "PERCENT_OUTPUT"
            elif name == "commanded_output":
                value = 0.25 * motor_id
            elif name == "measured_output":
                value = 0.20 * motor_id
            elif name == "position":
                value = 10.0 * motor_id
            elif name == "velocity":
                value = 1.0 * motor_id
            elif name == "current_amps":
                value = 2.5 * motor_id
            elif name == "timestamp":
                value = now
            writeField(table, name, kind, value)

    flush(nt_instance)


def run_local_loopback_test():
    """Run the two-instance NetworkTables loopback test, no hardware needed.

    Starts an NT server in this process as the RoboRIO stand-in and an NT4
    client as NEPI, then checks two things: that the heartbeat handshake on
    HEARTBEAT_TOPIC is answered with False after HEARTBEAT_RESPOND_DELAY_SECONDS,
    and that every typed input-group read helper returns the values the stand-in
    published.

    Returns:
        int: 0 if every check passed, 1 otherwise.
    """
    failures = []

    log("Starting local NetworkTables loopback test.")
    log("Topic: {}".format(HEARTBEAT_TOPIC))
    log("Vendored ntcore dir: {}".format(str(get_vendor_dir())))

    server = ntcore.NetworkTableInstance.create()
    client = ntcore.NetworkTableInstance.create()
    stop_event = threading.Event()
    responder_thread = None
    server_heartbeat = None
    client_heartbeat = None

    try:
        server.startServer("", LOOPBACK_HOST, LOOPBACK_PORT_NT3, LOOPBACK_PORT_NT4)
        client.startClient4(CLIENT_NAME + "-loopback")
        client.setServer(LOOPBACK_HOST, LOOPBACK_PORT_NT4)

        if wait_for_connection(client, timeout_seconds=10.0) is False:
            log("FAIL: loopback client never connected to the local server.")
            return 1
        log("PASS: loopback client connected to the local server.")

        # The RoboRIO stand-in: publishes every input group, and answers the
        # heartbeat exactly the way the real robot code must.
        loopbackPublishGroups(server)
        server_heartbeat = get_heartbeat_entry(server)
        responder_thread = threading.Thread(
            target=loopbackResponderLoop,
            args=(server, server_heartbeat, stop_event))
        responder_thread.daemon = True
        responder_thread.start()

        # Heartbeat handshake, measured from the NEPI side.
        client_heartbeat = get_heartbeat_entry(client)
        for cycle in range(LOOPBACK_HEARTBEAT_CYCLES):
            write_boolean(client_heartbeat, False)
            flush(client)
            time.sleep(0.1)

            sent_time = time.monotonic()
            publish_heartbeat(client, client_heartbeat)
            log("NEPI set heartbeat to True (cycle {}).".format(cycle + 1))

            answered = False
            while (time.monotonic() - sent_time) < LOOPBACK_RESPOND_TIMEOUT_SEC:
                if read_boolean(client_heartbeat) is False:
                    answered = True
                    break
                time.sleep(0.005)

            delay = time.monotonic() - sent_time
            if answered is False:
                failures.append("heartbeat cycle {} was never answered".format(cycle + 1))
                log("FAIL: heartbeat cycle {} not answered within {:.1f} s."
                    .format(cycle + 1, LOOPBACK_RESPOND_TIMEOUT_SEC))
                continue

            low = HEARTBEAT_RESPOND_DELAY_SECONDS
            high = HEARTBEAT_RESPOND_DELAY_SECONDS + LOOPBACK_RESPOND_SLACK_SEC
            if delay < low or delay > high:
                failures.append(
                    "heartbeat cycle {} answered in {:.3f} s, outside {:.3f}-{:.3f} s"
                    .format(cycle + 1, delay, low, high))
                log("FAIL: responder set heartbeat False after {:.3f} s, expected {:.3f}-{:.3f} s."
                    .format(delay, low, high))
            else:
                log("PASS: responder set heartbeat False after {:.3f} s (expected ~{:.3f} s)."
                    .format(delay, HEARTBEAT_RESPOND_DELAY_SECONDS))

        # Typed input-group reads, from the NEPI side. Primed first -- the app
        # node gets this for free from its 10 Hz poll, but a test that reads each
        # group exactly once has to do it explicitly.
        primeGroupReads(client)
        time.sleep(LOOPBACK_PRIME_WAIT_SEC)
        primeGroupReads(client)
        time.sleep(LOOPBACK_PRIME_WAIT_SEC)

        checks = [
            ("Robot Position", read_robot_position(client), LOOPBACK_POSITION),
            ("Robot Velocity", read_robot_velocity(client), LOOPBACK_VELOCITY),
            ("Robot Orientation", read_robot_orientation(client), LOOPBACK_ORIENTATION),
            ("RBX Feedback", read_rbx_feedback(client), LOOPBACK_RBX_FEEDBACK),
        ]
        for (group_name, read_dict, expected) in checks:
            log("{}: {}".format(group_name, read_dict))
            bad = []
            for key in expected.keys():
                if key == "timestamp":
                    continue
                if read_dict.get(key) != expected[key]:
                    bad.append("{}={} expected {}".format(key, read_dict.get(key), expected[key]))
            if bad:
                failures.append("{} mismatch: {}".format(group_name, "; ".join(bad)))
                log("FAIL: {} read back wrong: {}".format(group_name, "; ".join(bad)))
            else:
                log("PASS: {} read back correctly.".format(group_name))

        motor_ids = read_motor_ids(client)
        log("Motor ids: {}".format(motor_ids))
        if motor_ids != LOOPBACK_MOTOR_IDS:
            failures.append("motor ids {} expected {}".format(motor_ids, LOOPBACK_MOTOR_IDS))
            log("FAIL: motor ids read back wrong.")
        else:
            log("PASS: motor ids read back correctly.")

        feedback_dict = read_all_motor_feedback(client)
        for motor_id in LOOPBACK_MOTOR_IDS:
            motor_feedback = feedback_dict.get(motor_id)
            log("Motor {} feedback: {}".format(motor_id, motor_feedback))
            if motor_feedback is None:
                failures.append("motor {} feedback missing".format(motor_id))
                log("FAIL: motor {} feedback missing.".format(motor_id))
                continue
            if motor_feedback.get("motor_name") != "roborio_motor_" + str(motor_id):
                failures.append("motor {} name wrong".format(motor_id))
                log("FAIL: motor {} name read back wrong.".format(motor_id))

        if read_motor_feedback(client, 99) is not None:
            failures.append("absent motor 99 did not read back as None")
            log("FAIL: absent motor 99 should read back as None.")
        else:
            log("PASS: absent motor reads back as None.")

        # Output group writes, read back on the stand-in side.
        write_rbx_command_request(client, 7, COMMAND_TYPE_TARGET_POSE,
                                  target_pose=dict(x_m=3.0, y_m=4.0, heading_rad=1.25),
                                  named_action="")
        write_motor_command(client, 2, 0.5)

        # Primed the same way as the input groups, from the stand-in's side.
        command_table = groupTable(server, RBX_COMMAND_TABLE)
        motor_command_table = groupTable(server, motorTablePath(MOTOR_COMMAND_TABLE, 2))
        command_table.getNumber("request_id", -1)
        command_table.getNumber(RBX_COMMAND_POSE_PREFIX + "/x_m", -1.0)
        motor_command_table.getNumber("speed_ratio", -1.0)
        time.sleep(LOOPBACK_PRIME_WAIT_SEC)

        request_id = int(command_table.getNumber("request_id", -1))
        pose_x = command_table.getNumber(RBX_COMMAND_POSE_PREFIX + "/x_m", -1.0)
        log("RBX Command Request read on stand-in: request_id={} target_pose.x_m={}"
            .format(request_id, pose_x))
        if request_id != 7 or abs(pose_x - 3.0) > 1e-6:
            failures.append("RBX Command Request did not arrive intact")
            log("FAIL: RBX Command Request did not arrive intact.")
        else:
            log("PASS: RBX Command Request arrived intact.")

        speed_ratio = motor_command_table.getNumber("speed_ratio", -1.0)
        log("Motor Command read on stand-in: motor 2 speed_ratio={}".format(speed_ratio))
        if abs(speed_ratio - 0.5) > 1e-6:
            failures.append("Motor Command did not arrive intact")
            log("FAIL: Motor Command did not arrive intact.")
        else:
            log("PASS: Motor Command arrived intact.")

    except KeyboardInterrupt:
        log("\nLoopback test stopped.")
        return 1

    finally:
        stop_event.set()
        if responder_thread is not None:
            responder_thread.join(timeout=2.0)
        for (entry, instance) in [(client_heartbeat, client), (server_heartbeat, server)]:
            if entry is not None:
                try:
                    entry.close()
                except Exception:
                    pass
        stop_client(client)
        try:
            server.stopServer()
        except Exception:
            pass
        forgetGroupChanges(server)
        log("Loopback instances stopped.")

    if failures:
        log("")
        log("LOOPBACK TEST FAILED ({} failure(s)):".format(len(failures)))
        for failure in failures:
            log("  - " + failure)
        return 1

    log("")
    log("LOOPBACK TEST PASSED")
    return 0


def print_usage():
    """Print the standalone test entry points."""
    log("Usage: nepi_wpilib.py [--rio | --loopback]")
    log("  --rio       heartbeat test against a real Team {} RoboRIO".format(TEAM_NUMBER))
    log("  --loopback  two local NT instances in this process, no hardware (default)")


#########################################
# Main
#########################################
if __name__ == "__main__":
    args = sys.argv[1:]

    if "--rio" in args:
        sys.exit(run_roborio_heartbeat_test())

    if "--help" in args or "-h" in args:
        print_usage()
        sys.exit(0)

    if not args:
        print_usage()
        log("")

    sys.exit(run_local_loopback_test())
