# Controls Sandbox App status msg
# Minimal app-level status. The per-control data is carried by the
# nepi_interfaces/ControlsStatus message published by the ControlsIF
# instance at <node>/controls/status, and the per-datum data by the
# nepi_interfaces/DataStatus message published by the DataIF instance
# at <node>/data/status.

string name
string controls_namespace
bool controls_ready
string data_namespace
bool data_ready
