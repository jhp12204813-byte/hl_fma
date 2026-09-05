uint8 START=0
uint8 NORMAL_DRIVE=1
uint8 STOP_LINE=2
uint8 S_CURVE=3
uint8 TRAFFIC_LIGHT=4
uint8 PERPENDICULAR_PARKING=5
uint8 EMERGENCY_ZONE=6
uint8 PARALLEL_PARKING=7
uint8 LANE_CONTROL=8
uint8 FINISH=9

std_msgs/Header header
uint8 current_mission
bool active
bool completed
