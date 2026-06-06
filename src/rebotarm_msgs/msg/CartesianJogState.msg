std_msgs/Header header
string state
geometry_msgs/Pose current_pose
geometry_msgs/Pose target_pose
geometry_msgs/Twist commanded_twist
# Six arm joints in order joint1..joint6 (radians).
float64[] q_current
float64[] q_target
bool ik_success
string rejection_reason
string clamp_reason
bool dry_run
string output_mode
float64 command_age_s
