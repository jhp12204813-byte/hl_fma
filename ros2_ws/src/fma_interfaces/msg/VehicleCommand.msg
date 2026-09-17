std_msgs/Header header

uint8 DRIVE_STOP=0
uint8 DRIVE_FORWARD=1
uint8 DRIVE_REVERSE=2

uint8 drive_state
uint16 steering_adc
bool emergency_stop

# Optional manual traction duty; false preserves autonomous default 15%.
bool use_pwm_override
uint8 drive_pwm_percent
