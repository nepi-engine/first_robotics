# Per-slot RoboRIO motor feedback that has NO nepi_interfaces/MotorStatus home.
#
# The parts of the Motor-Control Feedback group that map onto the standard NEPI
# motor contract go out as MotorsStatus on the app's motors/motor_status topic
# (measured_output -> motor_speed_ratio + motor_dir, position -> motor_position,
# velocity -> motor_speed). MotorStatus has no field for the rest, so rather than
# drop them they ride here, one entry per motor slot, inside
# NepiAppWpilibIFStatus. See docs/WPILIB_IF_DESIGN.md, Decision 1.

# Slot index. Slot order IS the NEPI motor index: it is what
# nepi_interfaces/MotorControl.motor_ind selects.
int32 slot
# RoboRIO motor_id this slot is mapped to, or -1 for an unmapped slot
int32 motor_id
# NEPI motor name for this slot, matching MotorStatus.motor_name
string nepi_motor_name
# Operator's display name for this slot, blank if none was set
string display_name

# True once this slot's motor_id has been seen on NetworkTables at all. A slot
# that has never been seen reports motor_enable false in MotorsStatus rather
# than reporting zeros as if they were real telemetry.
bool seen
# True while this motor's feedback is also recent enough to trust
bool fresh

# RoboRIO's own display name for the motor. Distinct from nepi_motor_name, which
# is positional (motor_0, motor_1, ...) and must stay that way.
string roborio_motor_name
# RoboRIO motor controller mode, e.g. PERCENT_OUTPUT
string control_mode
# Commanded normalized output as reported by the RoboRIO. Not the same thing as
# MotorStatus.motor_speed_ratio, which carries the MEASURED output magnitude.
float32 commanded_output
# Motor current draw, amps
float32 current_amps

# RoboRIO's own sample timestamp for this motor's feedback, seconds
float32 timestamp
# Seconds since this motor's feedback was last observed to change, -1.0 if it
# has never been seen
float32 age_s
