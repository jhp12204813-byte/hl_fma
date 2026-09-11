"""All field-tunable constants for traffic-light perception.

Lamp boxes are ``(x1, y1, x2, y2)`` fractions *inside* the YOLO traffic-light
bounding box.  The defaults describe a horizontal four-lamp Korean vehicle
signal ordered RED, YELLOW, GREEN_LEFT, GREEN.  Change only these four values
when the physical housing layout differs; no absolute image ROI is used.
"""

# YOLO / tracking -----------------------------------------------------------
# One detector performs one inference and returns all three classes.
DETECTION_CONF_THRESHOLD = 0.35
DETECTION_IOU_THRESHOLD = 0.45
DETECTION_IMAGE_SIZE = 640
YOLO_FRAME_SKIP = 1  # 1 means infer every frame, 2 means every second frame.
TRAFFIC_LIGHT_CLASS_NAME = "traffic_light"
SIGN_PANEL_CLASS_NAME = "sign_panel"
MESSAGE_BOARD_CLASS_NAME = "message_board"

BBOX_SMOOTH_ALPHA = 0.45
BBOX_HISTORY_SIZE = 5
MAX_MISSING_FRAMES = 4
PANEL_ASSOCIATION_MAX_DISTANCE = 4.0  # multiples of the previous panel width

# Do not attempt internal ROI analysis when a detected object is too small.
MIN_TRAFFIC_LIGHT_WIDTH = 32
MIN_TRAFFIC_LIGHT_HEIGHT = 16
MIN_SIGN_PANEL_WIDTH = 16
MIN_SIGN_PANEL_HEIGHT = 16
MIN_MESSAGE_BOARD_WIDTH = 32
MIN_MESSAGE_BOARD_HEIGHT = 20

# Relative lamp ROIs: x1, y1, x2, y2, each in [0.0, 1.0].
TRAFFIC_RED_ROI = (0.02, 0.08, 0.23, 0.92)
TRAFFIC_YELLOW_ROI = (0.26, 0.08, 0.47, 0.92)
TRAFFIC_GREEN_LEFT_ROI = (0.51, 0.08, 0.72, 0.92)
TRAFFIC_GREEN_ROI = (0.76, 0.08, 0.98, 0.92)

# Compatibility aliases for the preceding STEP 1-6 naming.
RED_LAMP_ROI = TRAFFIC_RED_ROI
YELLOW_LAMP_ROI = TRAFFIC_YELLOW_ROI
GREEN_LEFT_LAMP_ROI = TRAFFIC_GREEN_LEFT_ROI
GREEN_LAMP_ROI = TRAFFIC_GREEN_ROI

LAMP_ROIS = {
    "RED": TRAFFIC_RED_ROI,
    "YELLOW": TRAFFIC_YELLOW_ROI,
    "GREEN_LEFT": TRAFFIC_GREEN_LEFT_ROI,
    "GREEN": TRAFFIC_GREEN_ROI,
}

# OpenCV hue is 0..180.  Red wraps around the ends of the hue axis.
RED_LOW_1 = (0, 130, 100)
RED_HIGH_1 = (8, 255, 255)
RED_LOW_2 = (168, 130, 100)
RED_HIGH_2 = (180, 255, 255)
YELLOW_LOW = (9, 130, 100)
YELLOW_HIGH = (38, 255, 255)
GREEN_LOW = (40, 110, 90)
GREEN_HIGH = (95, 255, 255)

RED1_LOW, RED1_HIGH = RED_LOW_1, RED_HIGH_1
RED2_LOW, RED2_HIGH = RED_LOW_2, RED_HIGH_2

# A lamp is ON only when both cleaned-mask occupancy and contour area pass.
MIN_COLOR_RATIO = 0.025
MIN_PIXEL_RATIO = MIN_COLOR_RATIO  # Compatibility alias.
MIN_CONTOUR_AREA = 5.0
MIN_CONTOUR_AREA_RATIO = 0.010
MORPH_KERNEL_SIZE = 3
MORPH_OPEN_ITERATIONS = 1
MORPH_CLOSE_ITERATIONS = 1

# Final-state priority.  GREEN_AND_LEFT is evaluated from two independent
# green lamp booleans before either individual green state.
STATE_PRIORITY = ("RED", "YELLOW", "GREEN_AND_LEFT", "GREEN_LEFT", "GREEN")
TRAFFIC_CONFIRM_FRAMES = 3
TRAFFIC_LOST_FRAMES = 3

# Each sign_panel is now an independent YOLO box. SLOT means left-to-right
# track identity, not a relative crop inside a vehicle-wide box.
SLOT_LANE_MAPPING = {1: "LEFT", 2: "CENTER", 3: "RIGHT"}
SIGN_CLASSIFY_SIZE = 224
SIGN_CLASS_CONF_THRESHOLD = 0.70
SIGN_CONFIRM_FRAMES = 3
SIGN_LOST_FRAMES = 3

# message_board active/idle test. Bright pixels include white LEDs; saturated
# pixels cover red/green/yellow LEDs. Both ratios are measured inside its bbox.
MESSAGE_BRIGHT_PIXEL_VALUE = 125
MESSAGE_SATURATION_MIN = 70
MESSAGE_MIN_ACTIVE_PIXEL_RATIO = 0.012
MESSAGE_MIN_SATURATED_PIXEL_RATIO = 0.006
MESSAGE_MIN_MEAN_BRIGHTNESS = 18.0
MESSAGE_MORPH_KERNEL_SIZE = 3
MESSAGE_RECOGNITION_MODE = "classification"  # Reserved until message classes are known.

# Drawing colors in BGR order.
STATE_COLORS = {
    "RED": (0, 0, 255),
    "YELLOW": (0, 255, 255),
    "GREEN": (0, 255, 0),
    "GREEN_LEFT": (0, 200, 0),
    "GREEN_AND_LEFT": (0, 255, 120),
    "UNKNOWN": (180, 180, 180),
}
