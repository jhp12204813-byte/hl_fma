std_msgs/Header header

# 실제 관측된 LEFT boundary
bool left_valid
float32 left_x_m
float32 left_confidence

# 실제 관측된 RIGHT boundary
bool right_valid
float32 right_x_m
float32 right_confidence

# 현재 PAIR에서 계산된 폭
bool selected_lane_width_valid
float32 selected_lane_width_m

# 이전 HIGH-confidence PAIR들로 학습한 폭
bool expected_lane_width_valid
float32 expected_lane_width_m

# 실제 controller가 사용하는 center
bool center_valid
float32 center_lateral_error_m
float32 center_heading_error_rad

float32 tracking_confidence
string tracking_source
string control_source
