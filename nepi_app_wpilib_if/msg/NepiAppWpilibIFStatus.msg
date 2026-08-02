# This message details the Wpilib IF App Status Msg

# App enabled state
bool enabled
# Available options (published so the RUI menu always matches the node)
string[] options
# Currently selected option
string selected_option
# Current value setting
float32 value

# Liveness heartbeat: always published True on every status update
bool heartbeat
