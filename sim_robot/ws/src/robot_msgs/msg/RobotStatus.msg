# Robot state values are published as strings:
# DISCONNECTED, LOCALIZING, IDLE, MANUAL, EXECUTING_ROUTE, ARRIVED, ERROR

builtin_interfaces/Time stamp
string robot_id
string map_id
bool connected
bool localization_ok
float32 localization_age_sec
string state
string message
string target_lm
string nearest_lm
string current_edge_id
string route_id
float32 route_progress
float32 pose_x
float32 pose_y
float32 pose_yaw
float32 linear_velocity
float32 angular_velocity
float32 battery_level
float32 battery_voltage
float32 battery_current
float32 battery_temperature
bool battery_charging
