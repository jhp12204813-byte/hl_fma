std_msgs/Header header
float32 speed_mps
float32 steering_angle_rad
bool emergency_stop

# Optional manual traction duty; false preserves autonomous default 15%.
bool use_pwm_override
uint8 drive_pwm_percent
