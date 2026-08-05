# This message details the AutoMove App Status Msg

# App enabled state
bool enabled
# Available options (published so the RUI menu always matches the node)
string[] options
# Currently selected option
string selected_option
# Current value setting
float32 value

#################
# Example Controls
#################
# Per-control data is NOT carried here. It travels in the
# nepi_interfaces/ControlsStatus message the app's example ControlsIF publishes at
# <example_controls_namespace>/status, exactly as in ControlsSandboxStatus.msg.
#
# The namespace is FULLY QUALIFIED. ControlsIF.get_namespace() returns a relative
# namespace ('app_auto_move/example_controls', no leading slash), which resolves
# where the IF itself uses it but not when a RUI component appends '/status' and
# hands it to rosbridge -- see auto_move_app_node.py getExampleControlsNamespace().

# Namespace of the example control set copied from nepi_app_controls_sandbox,
# where NepiAppAutoMove.js mounts its Example Controls box
string example_controls_namespace
# True once that ControlsIF has completed initialization
bool example_controls_ready
