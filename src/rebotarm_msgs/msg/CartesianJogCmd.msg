std_msgs/Header header
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
bool deadman
bool soft_stop
float64 speed_scale
bool enable_orientation
# When true, cartesian_jog_core skips Cartesian IK and integrates joint1 velocity.
bool base_jog_active
float32 joint1_jog_velocity_rad_s
# Frame for linear jog: local_window_frame | base_link
string command_frame_kind
