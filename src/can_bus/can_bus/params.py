import curses


# ROS control
ROS_QOS_DEPTH = 10
LEFT_ARM_TOPIC: str = "/left_arm"
RIGHT_ARM_TOPIC: str = "/right_arm"
LEFT_HAND_TOPIC: str = "/left_hand"
RIGHT_HAND_TOPIC: str = "/right_hand"

# Left arm angle
DEFAULT_FOUR_WHEEL_SPEED = [0.0, 0.0, 0.0, 0.0]
MIN_FOUR_WHEEL_SPEED = [0.0, 0.0, 0.0, 0.0]
MAX_FOUR_WHEEL_SPEED = [5000.0, 5000.0, 5000.0, 5000.0]

DEFAULT_FOUR_WHEEL_POS = [0.0, 0.0, 0.0, 0.0]

ARM_BIAS: int = 0
HAND_BIAS: int = 6
DEFAULT_WHEEL_NUMBER = len(DEFAULT_FOUR_WHEEL_SPEED)
DEFAULT_WHEEL_MOVE_STEP_VEL = 100.0
MIN_JOINT_MOVE_STEP_DEG = 1
MAX_JOINT_MOVE_STEP_DEG = 50

# Use for monitor ESP32 states `arm_reader.py`
LEFT_JOINTS_STATE_TOPIC = "/left_arm_state"
RIGHT_JOINTS_STATE_TOPIC = "/right_arm_state"
READER_TIMER_PERIOD = 0.5  # seconds
READER_CALLBACK_TIMER_PERIOD = 0.1  # seconds

# Publish Joints with Speed
DEFAULT_SPEED_DEG_PER_SEC = 5.0
DEFAULT_DURATION_SEC = 0.1
DEFAULT_FPS = 10.0

# Key mapping
KEY_ESC: int = 27
KEY_BACKSPACE: int = 127
KEY_ENTER: int = curses.KEY_ENTER

# Curses settings
STATE_POS: int = 0
ERR_POS: int = 1
MENU_POS: int = 2
CONTROL_POS: int = 8
TEXT_INPUT_POS: int = 13

# CAN settings 
WHEEL_CAN_IDS = [0x01, 0x02, 0x03, 0x04]
CAN_BAUDRATE = 2000000
CAN_TIMEOUT = 0.1
CAN_SPEED = 500000

if DEFAULT_WHEEL_NUMBER != len(DEFAULT_FOUR_WHEEL_SPEED) or \
        DEFAULT_WHEEL_NUMBER != len(MIN_FOUR_WHEEL_SPEED) or \
        DEFAULT_WHEEL_NUMBER != len(MAX_FOUR_WHEEL_SPEED):
    raise ValueError(
        "Error, the number of joints is not consistent. Please check again.")