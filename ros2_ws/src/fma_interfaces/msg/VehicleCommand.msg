std_msgs/Header header

uint8 DRIVE_STOP=0
uint8 DRIVE_FORWARD=1
uint8 DRIVE_REVERSE=2

uint8 drive_state
uint16 steering_adc
bool emergency_stop

# Optional raw timer compare counts. False preserves legacy direction-only mode.
# ARR=799, period=800 counts; deliberately cap below continuous-high CCR=800.
uint16 DRIVE_PWM_MAX=799
bool pwm_control
uint16 drive_pwm
