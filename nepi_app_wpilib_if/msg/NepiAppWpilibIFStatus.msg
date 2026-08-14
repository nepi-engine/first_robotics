# This message details the Wpilib IF App Status Msg

# App enabled state
bool enabled
# Available options (published so the RUI menu always matches the node)
string[] options
# Currently selected option
string selected_option
# Current value setting
float32 value

# App connection state to the robot network (WPILib NetworkTables)
bool connected

# Obstacles app namespace the node is currently connected to, or "None". The RUI
# Obstacles selector reads this back so the row shows the node's actual
# selection rather than only what the page last sent.
string selected_obstacles_namespace

#################
# Robot Interface Configuration
#################
# All persisted as params. Configuration lives in params; the telemetry it
# describes does not -- see docs/WPILIB_IF_DESIGN.md, Decision 1.

# FRC team number the NetworkTables client derives the RoboRIO address from
int32 team_number

# Number of motor slots. Slot order IS the NEPI motor index: it selects the
# channel for nepi_interfaces/MotorControl.motor_ind and it names motor_0,
# motor_1, ... in MotorsStatus. Four by default, any length allowed.
int32 motor_slot_count
# RoboRIO motor_id per slot, in slot order. -1 means the slot is unmapped, which
# is the one way to turn a slot off.
int32[] motor_ids
# Operator display name per slot, in slot order. Blank where none was set. NOT
# MotorStatus.motor_name, which stays positional.
string[] motor_names

#################
# Robot Interface Live State
#################

# motor_ids currently present on NetworkTables. The RUI shows these read-only
# beside the slot editor, so an operator can see which ids are real without
# configuration depending on the robot being alive.
int32[] nt_motor_ids

# Per-slot RoboRIO feedback fields that have no MotorStatus home. The mappable
# fields go out as nepi_interfaces/MotorsStatus on motors_namespace/motor_status.
WpilibMotorFeedback[] motor_feedback

# Namespace of the standard NEPI multi-motor producer this app hosts, or "None".
# MotorsStatus is published at <motors_namespace>/motor_status.
string motors_namespace

# True when the operator has enabled the RBX device. Off by default: enabling it
# is what presents this robot to the rest of NEPI as a commandable RBX device.
bool rbx_enabled
# Namespace the RBX device is advertised at once enabled, or "None". Built only
# after the RoboRIO reports supported_capabilities, since RBXRobotIF caches its
# capability flags at construction.
string rbx_namespace
# RBX device's own ready (not busy) state
bool rbx_ready

# Where the robot's NavPose appears. There is exactly one owner of that pose and
# it is the RBX device, so this reads "None" while rbx_enabled is false -- see
# docs/WPILIB_IF_DESIGN.md, Decision 4.
string navpose_topic
# True when Robot Position and/or Robot Orientation are currently valid and
# fresh enough to fuse into a NavPose. Reported regardless of rbx_enabled, so
# pose health is visible even when nothing is publishing it.
bool navpose_valid

#################
# RBX Feedback
#################
# The RBX Feedback input group as last read from NetworkTables. This group
# defines no valid flag, so an empty request_status means it has not been seen.

# Capability strings the RoboRIO advertises. These decide which RBXRobotIF
# callbacks are non-None, and therefore which controls the RUI renders.
string[] supported_capabilities
# RBX Command Request the RoboRIO reports it is working on
string active_request_id
string active_request_type
# IDLE, ACCEPTED, EXECUTING, COMPLETE, FAILED or REJECTED
string request_status
# Human readable detail for the current request status
string status_message

#################
# Example Controls
#################
# Per-control data is NOT carried here. It travels in the
# nepi_interfaces/ControlsStatus message the app's example ControlsIF publishes at
# <example_controls_namespace>/status, exactly as in ControlsSandboxStatus.msg.
#
# The namespace is FULLY QUALIFIED. ControlsIF.get_namespace() returns a relative
# namespace ('app_wpilib_if/example_controls', no leading slash), which resolves
# where the IF itself uses it but not when a RUI component appends '/status' and
# hands it to rosbridge -- see wpilib_if_app_node.py getExampleControlsNamespace().

# Namespace of the example control set copied from nepi_app_controls_sandbox,
# where NepiAppWpilibIF.js mounts its Example Controls box
string example_controls_namespace
# True once that ControlsIF has completed initialization
bool example_controls_ready
