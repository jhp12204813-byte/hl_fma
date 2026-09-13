std_msgs/Header header
float32 speed_mps
float32 steering_angle_rad
bool emergency_stop

# Optional raw timer compare counts. False preserves legacy direction-only mode.
# ARR=799, period=800 counts; deliberately cap below continuous-high CCR=800.
uint16 DRIVE_PWM_MAX=799
bool pwm_control
uint16 drive_pwm
