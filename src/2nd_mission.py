#-----------------------------------------------------------------------------------------------------------------------
# 2nd Mission WRO 2025 FE - VivaLaVida
# Final Version
#-----------------------------------------------------------------------------------------------------------------------

# v1.1: add state print, add turn/lap count and stop after 3 laps, variables cleanup
#       add ultrasonic for side checks during cruise
# v1.2: add backwards drive after turning, unpark in both directions
# v1.3: give priority to object that is closer [line or obstacle]
# v1.4-v1.5: did not work
# v1.6: restructuring with FSM, lock object logic, tbd unpark alternstive in open space

# =========================
# IMPORTS
# =========================

import os, sys

VENV_PY = "/home/stem/env/bin/python"  # exact path to Python in your venv

if sys.executable != VENV_PY and os.path.exists(VENV_PY):
    print(f"[INFO] Relaunching under virtual environment: {VENV_PY}", flush=True)
    os.execv(VENV_PY, [VENV_PY] + sys.argv)

# --- your normal robot code below ---
print(f"[INFO] Now running inside: {sys.executable}")

import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import smbus2
import threading
import board
import busio
import digitalio
import adafruit_vl53l0x
from gpiozero import Device, Button, DistanceSensor, LED
from gpiozero.pins.lgpio import LGPIOFactory   # Uses /dev/gpiochip*
Device.pin_factory = LGPIOFactory()

# =========================
# CONFIGURABLE PARAMETERS
# =========================

BLUE_BACK_SPEED       = 16   # reverse speed when box is "inside the car box"
BLUE_BACK_DURATION    = 0.7  # how long to reverse in blue-backward mode
POST_BACK_FOLLOW_S    = 0.7  # how long to bias steering toward box after back

# --- Blue-backward close obstacle escape ---
in_blue_backward      = False
blue_backward_start   = 0.0
back_follow_angle     = None
post_back_follow_until = 0.0


TOP_CROP_PCT    = 0.15  # less top crop so obstacles enter view earlier
BOTTOM_CROP_PCT = 0.05
LEFT_CROP_PCT   = 0.04  # small side crop to reduce outside-course detections
RIGHT_CROP_PCT  = 0.04

# ---- Speed settings ----
NORMAL_SPEED          = 14   # Base forward cruising speed during normal driving
AVOID_SPEED           = 13   # Reverse speed when doing obstacle-avoidance backup
TURN_MOTOR_SPEED      = 19   # Motor speed while performing an 80° line-based turn
UNPARK_STRAIGHT_SPEED = 18
   # Speed used during smart unpark phases
STOP_SPEED            = 0    # Zero-speed (motors off)
POST_TURN_BACK_SPEED  = 20     # Reverse speed used right after each turn                                           !!!15 to 20

# ---- Motor / servo basics ----
MOTOR_FWD       = 1         # PCA9685 motor channel for forward direction
MOTOR_REV       = 2         # PCA9685 motor channel for reverse direction
SERVO_CHANNEL   = 0         # PCA9685 channel used by steering servo
CENTER_ANGLE    = 90        # Servo angle for going straight
LEFT_FAR        = 100       # Steering angle for a far red box (mild left)
LEFT_NEAR       = 120       # Steering angle for a near red box (strong left)
RIGHT_FAR       = 80        # Steering angle for a far green box (mild right)
RIGHT_NEAR      = 60        # Steering angle for a near green box (strong right)
LEFT_COLLIDE_ANGLE = 105    # Steering angle on left side collision correctio
RIGHT_COLLIDE_ANGLE = 75    # Steering angle on right side collision correctio

# ---- Obstacle detection (vision) ----
MIN_AREA          = 6000    # Lowered so obstacles are detected earlier
MAX_AREA          = 20000   # Area at which box is considered "very close"
COLOR_HOLD_FRAMES = 2       # Frames the same color must persist to be “locked”
OBSTACLE_CLEAR_FRAMES = 10     # Frames without red/green to "forget" obstacle
# X-position aware steering (normalized [-1..1] offset from image center)
XPOS_GAIN_DEG   = 3.0   # how many degrees we add/subtract for full offset
XPOS_MAX_OFFSET = 0.9    # clamp |offset|, treat anything beyond as 0.7
SERVO_XPOS_SIGN = 1.0  #X-position steering sign 1, -1
#  +1 works for most setups; if in tests the robot turns TOWARDS the obstacle


# ---- Obstacle lock (vision only) ----
# Only start avoidance when the box is visually close enough
OBSTACLE_LOCK_AREA_MIN = 6500    # Lowered so avoidance can lock earlier
OBSTACLE_LOCK_Y_FRAC   = 0.30    # Lowered so obstacle can lock higher in the image


# ---- ToF thresholds (general) ----
SIDE_COLLIDE_CM       = 40.0  # If side < this, we steer away to avoid collision

# ---- Post-turn backward reposition ----
POST_TURN_BACK_CLEAR_CM  = 25.0   # Back ToF distance target after a normal line-based turn
POST_TURN_BACK_TIMEOUT_S = 2.0    # Safety timeout so we don't reverse forever
POST_TURN_LINE_IGNORE_S  = 2.0  # Time to ignore blue/orange lines after backing (sec)

# ---- Obstacle avoidance ----
AVOID_BACK_DURATION = 0.0  # Reverse duration in normal avoidance (seconds)

# ---- Emergency front escape ----
# If the robot is very close to the front wall and there is no visible obstacle
# or turn line, reverse straight for a short time, then resume with a quick
# side/yaw correction. This prevents driving into the wall when detection is late.
FRONT_EMERGENCY_CM = 10.0
FRONT_EMERGENCY_BACK_DURATION = 1.5
FRONT_EMERGENCY_BACK_SPEED = 16
FRONT_EMERGENCY_COOLDOWN_S = 1.0
FRONT_EMERGENCY_RECOVER_S = 0.65
FRONT_EMERGENCY_RECOVER_SPEED = NORMAL_SPEED

# ---- Line-turn (shape / Hough based) ----
TURN_LEFT_SERVO       = 55    # Servo angle for a hard left line-turn
TURN_COOLDOWN_S       = 0.7   # Minimal cooldown after a line detection
TURN_MIN_INTERVAL_S   = 5.0   # Minimum time between two turns
CANNY_LO, CANNY_HI    = 60, 160  # Canny edge thresholds for line band pre-processing
BLUR_KSIZE            = 5     # Gaussian blur kernel size for edge preprocessing
HOUGH_THRESHOLD       = 60    # HoughLinesP threshold
HOUGH_MIN_LENGTH      = 120   # Min line length to accept from HoughLinesP
HOUGH_MAX_GAP         = 20    # Max allowed gap in HoughLinesP segments
LINE_DETECT_CONSEC_FRAMES = 2 # Frames of consistent line detection to confirm                                      !!!1 to 2
LINE_ORIENT_MIN_DEG   = 25    # Minimum angle (deg) to accept a line as “diagonal”
LINE_ORIENT_MAX_DEG   = 65    # Maximum angle (deg) to accept a line as “diagonal”
LINE_MASK_THICKNESS   = 9     # Thickness of mask drawn over detected line band

# ---- Turn-related constants ----
LINE_CENTER_BLUE_Y_MIN     = 480  # Minimal Y for blue line to be valid for turn
LINE_CENTER_ORANGE_Y_MIN   = 480 # Minimal Y for orange line to be valid for turn
TURN_RIGHT_SERVO           = 130  # Servo angle for a hard right line-turn

# ---- Yaw-aware turn target ----
# The robot checks its yaw before starting a line turn.
# If it is already angled, the turn target is adjusted so the final heading
# is a fixed 80 degree turn.
TURN_TARGET_DEG            = 80.0
TURN_YAW_TARGET_MIN_DEG    = 80.0
TURN_YAW_TARGET_MAX_DEG    = 80.0
TURN_FAILSAFE_MAX_DEG      = 80.0

# ---- Turn overshoot protection ----
# If the robot is already angled in the same direction as the next turn
# after passing a single-side obstacle, reduce the measured turn target.
# Example: before a LEFT turn, yaw=+30 means the robot is already partly left,
# so turning another full 80 degrees can over-rotate.
TURN_PRE_YAW_REDUCE_START_DEG = 12.0
TURN_PRE_YAW_REDUCE_MAX_DEG   = 24.0
TURN_MIN_TARGET_DEG           = 56.0
TURN_STOP_EARLY_DEG           = 3.0

YAW_RESET_AFTER_LEFT  = 0.0     # Kept for compatibility; corrected turns reset yaw to 0 after finishing
YAW_RESET_AFTER_RIGHT = 0.0      # Yaw offset after finishing a right turn

TURN_COOLDOWN_SEC = 6.0           # Cooldown after each completed turn (no new turns)

# ---- Dynamic front trigger for line turns ----
# When a blue/orange turn line is detected, the required front distance
# depends on the next closest visible obstacle color.
# Green -> turn earlier at 85 cm
# Red   -> turn later at 70 cm
# None/unknown -> default 76 cm
FRONT_TRIGGER_GREEN   = 85.0
FRONT_TRIGGER_RED     = 70.0
FRONT_TRIGGER_DEFAULT = 85.0

# Kept for compatibility with old debug/logic naming.
FRONT_TRIGGER = FRONT_TRIGGER_DEFAULT

def get_turn_front_trigger(robot_direction, next_obstacle_color):
    # Same rule for left and right direction:
    # closest green -> 85 cm, closest red -> 70 cm, else -> 76 cm.
    if next_obstacle_color == "Green":
        return FRONT_TRIGGER_GREEN
    if next_obstacle_color == "Red":
        return FRONT_TRIGGER_RED
    return FRONT_TRIGGER_DEFAULT

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def compute_yaw_corrected_turn_target(turn_direction, pre_turn_yaw_deg):
    """Return the measured yaw target for the next line turn.

    Normal turns still use 80 degrees.
    The only time we reduce the target is when the robot is already angled
    in the SAME direction as the upcoming turn. This happens after avoiding
    a single obstacle on one side; doing another full 80 degrees can make
    the robot over-turn.

    In this robot/logs, LEFT turns increase yaw and RIGHT turns decrease yaw.
    """
    # LEFT turn = positive yaw while turning, RIGHT turn = negative yaw.
    turn_sign = 1.0 if turn_direction == "Left" else -1.0

    # Positive same_dir_yaw means: already rotated toward the turn direction.
    same_dir_yaw = turn_sign * pre_turn_yaw_deg

    if same_dir_yaw <= TURN_PRE_YAW_REDUCE_START_DEG:
        return TURN_TARGET_DEG

    reduction = same_dir_yaw - TURN_PRE_YAW_REDUCE_START_DEG
    reduction = clamp(reduction, 0.0, TURN_PRE_YAW_REDUCE_MAX_DEG)

    return clamp(
        TURN_TARGET_DEG - reduction,
        TURN_MIN_TARGET_DEG,
        TURN_TARGET_DEG
    )

# ---- Obstacle vs line priority ----
OBSTACLE_LINE_MARGIN_PX = 10000  # How many pixels "closer" something must be to win priority

# HSV thresholds (tweak for venue lighting)
RED1_LO = np.array([0, 120, 80], dtype=np.uint8)
RED1_HI = np.array([9, 255, 220], dtype=np.uint8)

RED2_LO = np.array([170, 120, 80], dtype=np.uint8)
RED2_HI = np.array([179, 255, 220], dtype=np.uint8)

# Wider red-ish mask ONLY used to remove red/reflections from orange line detection.
# This is not used as the real red obstacle mask.
RED_BLOCK1_LO = np.array([0, 90, 70], dtype=np.uint8)
RED_BLOCK1_HI = np.array([16, 255, 255], dtype=np.uint8)

RED_BLOCK2_LO = np.array([165, 90, 70], dtype=np.uint8)
RED_BLOCK2_HI = np.array([179, 255, 255], dtype=np.uint8)

GREEN_LO = np.array([50, 100, 120], dtype=np.uint8)
GREEN_HI = np.array([85, 255, 230], dtype=np.uint8)

ORANGE_LO = np.array([11, 170, 135], dtype=np.uint8)
ORANGE_HI = np.array([30, 255, 255], dtype=np.uint8)

BLUE_LO = np.array([100, 140, 110], dtype=np.uint8)
BLUE_HI = np.array([140, 255, 250], dtype=np.uint8)


# ---- Dynamic yaw / line trigger tuning ----
BLUE_MIN_LEN_PX     = 50    # Minimal Hough line length (blue/orange) to trigger turn

# ---- Post-reverse / settling behavior ----
SETTLE_DURATION      = 0.0  # Seconds to force CENTER_ANGLE after certain events
settle_until_ts      = 0.0  # Timestamp until which settle is active

# ---- Extra IMU gain near boxes ----
BOX_YAW_GAIN_MIN = 1.5  # Minimum yaw gain when box just appears
BOX_YAW_GAIN_MAX = 3.0  # Maximum yaw gain when box very close

# ---- Drift control ----
DRIFT_GZ_THRESH       = 0.8   # Max |Gz| to consider for drift/bias update
BIAS_ALPHA            = 0.002 # Smoothing factor for gyro bias update
STRAIGHT_SERVO_WINDOW = 8     # Servo must be within this of CENTER to update bias

# ---- Stability gates ----
YAW_CLAMP_DEG     = 120.0 # Limit |yaw| to avoid runaway integration
SOFT_DECAY_RATE   = 0.6   # Softer yaw decay factor used during bias update

# ---- IMU keep-straight gains ----
YAW_KP_BASE             = 1.6  # Base proportional gain for yaw correction
SERVO_CORR_LIMIT_BASE   = 25   # Max correction (deg) from base yaw controller
YAW_DEADBAND_DEG_BASE   = 4.0  # Deadband for base yaw correction (small error ignored)
YAW_DEADBAND_DEG_STRONG = 6.0  # Deadband when using stronger yaw correction
YAW_KP_STRONG           = 1.8  # Stronger proportional gain when no boxes
SERVO_CORR_LIMIT_STRONG = 24   # Max correction (deg) for strong yaw controller

# ---- Smart Unpark configuration ----
UNPARK_CENTER_ANGLE     = CENTER_ANGLE # Servo angle for going straight during unpark
UNPARK_LEFT_TURN_ANGLE  = 55           # Initial unpark steering angle if turning left
UNPARK_RIGHT_TURN_ANGLE = 130          # Initial unpark steering angle if turning right

# ---- Turn / lap counting ----
TURNS_PER_LAP = 4   # Number of line-based turns per lap
TOTAL_LAPS    = 3   # Number of laps to complete (1 full lap = 4 turns, then parking)
TOTAL_TURNS   = TURNS_PER_LAP * TOTAL_LAPS  # Convenience (total planned turns)

# ---- Line / ToF gates for turning ----
TURN_FRONT_MIN_CM = 5.0    # Min front distance to allow starting a line-based turn
TURN_FRONT_MAX_CM = 140.0  # Max front distance to allow starting a line-based turn

# =========================
# HARDWARE SETUP
# =========================

# ---- Status LEDs (gpiozero) ----
RED_LED_PIN   = 13
GREEN_LED_PIN = 19

red_led   = LED(RED_LED_PIN)
green_led = LED(GREEN_LED_PIN)

# Program start indication: red ON, green OFF
red_led.on()
green_led.off()

# ---- IMU SETUP ----
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47
bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def get_gyro_z_bias(samples=200):
    total = 0.0
    for _ in range(samples):
        raw = read_raw_data(GYRO_ZOUT_H)
        total += raw / 131.0
        time.sleep(0.005)
    return total / samples

mpu6050_init()
print("Measuring gyro bias, keep sensor still...")
gyro_z_bias = get_gyro_z_bias()
print(f"Gyro Z bias: {gyro_z_bias:.3f} deg/s")

# ---- YAW TRACKING -----
yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()

def reset_yaw_listener():
    global yaw
    while True:
        input("Press ENTER to reset yaw to 0°")
        with yaw_lock:
            yaw = 0.0
            print("Yaw reset to 0°")

threading.Thread(target=reset_yaw_listener, daemon=True).start()

def imu_center_servo(current_yaw_deg: float, deadband: float, kp: float, limit: float) -> int:
    if abs(current_yaw_deg) <= deadband:
        return CENTER_ANGLE
    corr = kp * current_yaw_deg
    corr = max(-limit, min(limit, corr))
    return int(CENTER_ANGLE + corr)

# ----- SERVO SETUP -----
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ---- CAMERA SETUP ----
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (1280, 800)})) #(main={"size": (640, 480)}))
picam2.set_controls({"FrameRate": 15})
picam2.start()
time.sleep(1.0)
#picam2.set_controls({"AwbEnable": False})

# ---- ToF SETUP ----
i2c = busio.I2C(board.SCL, board.SDA)
xshut_pins = {
    "left":    board.D16,
    "right":   board.D25,
    "front":   board.D26,
    "back":    board.D8,
    "front_l": board.D7,
    "front_r": board.D24
}
addresses = {
    "left":    0x30,
    "right":   0x31,
    "front":   0x32,
    "back":    0x33,
    "front_l": 0x34,
    "front_r": 0x35
}

xshuts = {}
for name, pin in xshut_pins.items():
    x = digitalio.DigitalInOut(pin)
    x.direction = digitalio.Direction.OUTPUT
    x.value = False
    xshuts[name] = x
time.sleep(0.1)

sensors = {}
for name in ["left", "right", "front", "back", "front_l", "front_r"]:
    xshuts[name].value = True
    time.sleep(0.05)
    s = adafruit_vl53l0x.VL53L0X(i2c)
    s.set_address(addresses[name])
    s.start_continuous()
    sensors[name] = s
    print(f"[TOF] {name.upper()} active at {hex(addresses[name])}")

def tof_cm(sensor):
    try:
        val = sensor.range / 10.0
        if val <= 0 or val > 150:
            return 999
        return val
    except:
        return 999

# ==== ULTRASONIC PINS (FRONT, LEFT, RIGHT) ====
TRIG_FRONT, ECHO_FRONT = 22, 23  # GPIO pins for front sensor
TRIG_LEFT,  ECHO_LEFT  = 27, 17  # GPIO pins for left sensor
TRIG_RIGHT, ECHO_RIGHT = 5,  6   # GPIO pins for right sensor

# ==== ULTRASONIC SENSORS (gpiozero DistanceSensor) ====
front_ultra = DistanceSensor(echo=ECHO_FRONT, trigger=TRIG_FRONT, max_distance=2.4, queue_len=3)
left_ultra  = DistanceSensor(echo=ECHO_LEFT,  trigger=TRIG_LEFT,  max_distance=1.2, queue_len=3)
right_ultra = DistanceSensor(echo=ECHO_RIGHT, trigger=TRIG_RIGHT, max_distance=1.2, queue_len=3)

def ultra_cm(sensor, max_cm=200.0):
    d_m = sensor.distance  # in meters, between 0 and max_distance
    if d_m is None:
        return 999
    d_cm = d_m * 100.0
    if d_cm <= 0 or d_cm > max_cm:
        return 999
    return d_cm

def get_front_ultra_cm():
    return ultra_cm(front_ultra)

def get_left_ultra_cm():
    return ultra_cm(left_ultra)

def get_right_ultra_cm():
    return ultra_cm(right_ultra)


# =========================
# PARKING INTEGRATION HELPERS
# =========================
# Integrated from the separate parking script.
# This reuses the already initialized MPU6050, ToF sensors, ultrasonic sensors,
# motor driver, and steering servo from the main 2nd mission script.

PARKING_ALIGN_SPEED = 13
PARKING_SPEED       = 13

# ---- Final-turn parking preparation ----
# Runs only after the final turn, before parking search/alignment.
# Minimum moving speed is 13 because the robot cannot reliably move slower.
FINAL_PARK_RIGHT_CLEAR_SPEED  = 13
FINAL_PARK_RIGHT_CLEAR_TIME_S = 1.0
FINAL_PARK_RIGHT_CLEAR_ANGLE  = 75    # mild left steer, away from right-side obstacle/wall
FINAL_PARK_RIGHT_CLEAR_CM     = 45.0  # if right side is closer than this, clear it first

PARKING_SIDE_TARGET_CM       = 26.0  # fallback wall-follow target if the unpark snapshot is invalid

# ---- Parking-space trigger: match the POST-UNPARK sensor snapshot ----
# The robot records LEFT, RIGHT, FRONT, and BACK immediately after unpark.
# After the 4th turn it searches until it sees those same 4 values again
# within PARKING_TRIGGER_TOL_CM for a few stable reads, then it parks.
PARKING_TRIGGER_TOL_CM       = 3.0
PARKING_MATCH_STABLE_COUNT   = 1

# Front distance must be stricter than the side/back distances.
# The robot moves toward the front wall, so allowing target-3 cm lets it
# park too far forward. With these values, front is valid only when it is
# almost at the target or slightly farther back from the wall.
PARKING_FRONT_TOO_CLOSE_TOL_CM = 0.7
PARKING_FRONT_TOO_FAR_TOL_CM   = 3.0

# Fallback targets used only if a post-unpark reading was invalid (999).
# In normal runs these are overwritten by post_unpark_sensor_snapshot.
PARKING_TARGET_FRONT_CM      = 74.0
PARKING_TARGET_BACK_CM       = 10.0
PARKING_TARGET_NEAR_SIDE_CM  = 25.0
PARKING_TARGET_OPPOSITE_SIDE_CM = 120.0

# Compatibility names used elsewhere in the parking code/debug.
PARKING_TRIGGER_SIDE_CM       = PARKING_TARGET_NEAR_SIDE_CM
PARKING_TRIGGER_SIDE_TOL_CM   = PARKING_TRIGGER_TOL_CM
PARKING_TRIGGER_FRONT_STOP_CM = PARKING_TARGET_FRONT_CM

# Safety only. Do not use this as the parking trigger. It only prevents crashing
# if the robot has already passed the target and is very close to the front wall.
PARKING_FRONT_FORCE_STOP_CM   = 18.0
PARKING_FRONT_MAX_VALID_CM    = 200.0
PARKING_FOLLOW_MAX_TIME_S     = 35.0  # no parking on timeout; only debug message and keep searching

# When left/right/back are already correct, the robot must not miss the
# post-unpark front distance by driving forward too fast. It creeps near
# the target, stops as soon as the front reaches/crosses the target window,
# and only then starts the 3-phase parking if all four distances are correct.
PARKING_FRONT_SLOW_MARGIN_CM      = 22.0
PARKING_FRONT_OVERSHOOT_TOL_CM    = 10.0
PARKING_FRONT_CREEP_SPEED         = 13
PARKING_FRONT_REVERSE_CREEP_SPEED = 13
PARKING_FRONT_SETTLE_S            = 0.12
# Stop this many cm BEFORE the post-unpark front distance.
# Bigger front distance = robot stops earlier / farther back.
PARKING_FRONT_STOP_BACK_OFFSET_CM = 3.0

# Pulse-search parking constants kept for compatibility with older parking logic.
PARKING_PULSE_MOVE_S   = 0.10
PARKING_PULSE_SETTLE_S = 0.08

# ---- Magenta pass-based parking trigger ----
# After the final turn the robot searches forward. Parking starts only after:
#   1) magenta parking boundary is seen for a few frames, then
#   2) magenta disappears for a few frames.
# This means the robot has passed the parking space and can reverse into it.
# Moving speed is 13 because the robot cannot reliably move slower.
PARKING_MAGENTA_SEARCH_SPEED = 13
PARKING_MAGENTA_MIN_AREA     = 900
PARKING_MAGENTA_SEEN_FRAMES  = 2
PARKING_MAGENTA_LOST_FRAMES  = 3
PARKING_AFTER_MAGENTA_LOST_ROLL_S = 0.00
PARKING_MAGENTA_SEARCH_TIMEOUT_S  = 40.0
PARKING_MAGENTA_FRONT_SAFETY_CM   = 28.0
PARKING_MAGENTA_DEBUG_EVERY_S     = 0.25
# If magenta becomes very large/low in the image, the robot is about to hit
# the parking wall. Start the parking maneuver before collision, but only if
# no red/green block is currently in front.
PARKING_MAGENTA_CLOSE_BOTTOM_Y    = 620
PARKING_MAGENTA_CLOSE_AREA        = 120000

# During magenta parking search, a close front reading may be a red/green
# obstacle, not the parking wall. Never start the 3-phase parking from this
# safety condition; clear/avoid first and continue searching.
PARKING_OBSTACLE_MIN_AREA         = 3500
PARKING_OBSTACLE_FRONT_Y_FRAC     = 0.18
# Stronger parking-search obstacle clear. Speed stays at 13 because slower
# movement is unreliable on this robot; control distance with time instead.
PARKING_SAFETY_REVERSE_TIME_S     = 0.42
PARKING_SAFETY_AVOID_TIME_S       = 0.65
PARKING_SAFETY_CLEAR_SPEED        = 13
PARKING_SAFETY_FRONT_CLEAR_CM     = 30.0
PARKING_SAFETY_HARD_CLOSE_CM      = 18.0
PARKING_SAFETY_MAX_ATTEMPTS       = 4

# HSV for the magenta / pink parking boundary. Tune only these if lighting changes.
MAGENTA_LO = np.array([140, 60, 120], dtype=np.uint8)
MAGENTA_HI = np.array([170, 255, 255], dtype=np.uint8)

PARKING_MIN_VALID_CM = 5.0
PARKING_MAX_VALID_CM = 150.0

PARKING_YAW_DEADBAND_DEG = 2.0
PARKING_YAW_KP           = 1.0
PARKING_SERVO_CORR_LIMIT = 15

PARKING_DIST_KP = 2.0

PARKING_YAW_PARALLEL_TOL      = 1.0
PARKING_PARALLEL_STABLE_COUNT = 5
PARKING_PARALLEL_MAX_TIME     = 3.0

parking_running = False
parking_wall_side = "right"
post_unpark_sensor_snapshot = {}

def valid_cm_for_parking(d):
    return d != 999 and PARKING_MIN_VALID_CM <= d <= PARKING_MAX_VALID_CM

def get_best_side_cm(side_name):
    """Return the nearest valid side distance from ToF + ultrasonic."""
    if side_name == "right":
        tof_side = tof_cm(sensors["right"])
        ultra_side = get_right_ultra_cm()
    else:
        tof_side = tof_cm(sensors["left"])
        ultra_side = get_left_ultra_cm()

    candidates = [d for d in (tof_side, ultra_side) if d != 999]
    return min(candidates) if candidates else 999

def get_parking_sensor_snapshot(label="SENSOR"):
    """Read and print the sensor values used by parking."""
    snap = {
        "left":  get_best_side_cm("left"),
        "right": get_best_side_cm("right"),
        "front": get_front_ultra_cm(),
        "back":  tof_cm(sensors["back"]),
    }
    print(
        f"[{label}] left={snap['left']:.1f}cm | right={snap['right']:.1f}cm | "
        f"front_us={snap['front']:.1f}cm | back_tof={snap['back']:.1f}cm",
        flush=True
    )
    return snap


def get_parking_front_cm():
    """Return the front distance used for parking matching.

    For the 4-distance parking trigger we prefer the SAME type of reading that
    is printed in the post-unpark snapshot: the front ultrasonic. If it is
    invalid, we fall back to the front ToF. The nearest valid front value is
    also returned as front_best for safety/crash prevention only.
    """
    front_u = get_front_ultra_cm()
    front_t = tof_cm(sensors["front"])

    valid_u = (front_u != 999 and PARKING_MIN_VALID_CM <= front_u <= PARKING_FRONT_MAX_VALID_CM)
    valid_t = (front_t != 999 and PARKING_MIN_VALID_CM <= front_t <= PARKING_FRONT_MAX_VALID_CM)

    if valid_u:
        front_match = front_u
    elif valid_t:
        front_match = front_t
    else:
        front_match = 999

    candidates = []
    if valid_u:
        candidates.append(front_u)
    if valid_t:
        candidates.append(front_t)
    front_best = min(candidates) if candidates else 999

    return front_match, front_u, front_t, front_best



def get_parking_all_distances():
    """Read the four distances used to confirm the parking space.

    Left/right use the best available side reading from ToF + ultrasonic.
    Front uses the best available reading from front ultrasonic + front ToF.
    Back uses the rear ToF.
    """
    front_match, front_u, front_t, front_best = get_parking_front_cm()
    return {
        "left": get_best_side_cm("left"),
        "right": get_best_side_cm("right"),
        "front": front_match,      # value used for matching post-unpark snapshot
        "front_best": front_best,  # safety only
        "front_us": front_u,
        "front_tof": front_t,
        "back": tof_cm(sensors["back"]),
    }


def get_parking_target_distances(side_name):
    """Return parking targets from the POST-UNPARK sensor snapshot.

    This is the important behavior: after the 4th turn, the robot should find
    the same physical position it had the second it finished unparking. So the
    targets are not fixed values like 120/25/74/10; they come from
    post_unpark_sensor_snapshot. Fallback constants are used only if a snapshot
    reading was invalid.
    """
    snap = post_unpark_sensor_snapshot if isinstance(post_unpark_sensor_snapshot, dict) else {}

    def snap_or(name, fallback):
        v = snap.get(name, 999)
        if v != 999 and PARKING_MIN_VALID_CM <= v <= PARKING_FRONT_MAX_VALID_CM:
            return float(v)
        return float(fallback)

    if side_name == "right":
        left_fallback = PARKING_TARGET_OPPOSITE_SIDE_CM
        right_fallback = PARKING_TARGET_NEAR_SIDE_CM
    else:
        left_fallback = PARKING_TARGET_NEAR_SIDE_CM
        right_fallback = PARKING_TARGET_OPPOSITE_SIDE_CM

    front_target = snap_or("front", PARKING_TARGET_FRONT_CM) + PARKING_FRONT_STOP_BACK_OFFSET_CM

    return {
        "left": snap_or("left", left_fallback),
        "right": snap_or("right", right_fallback),
        "front": front_target,
        "back": snap_or("back", PARKING_TARGET_BACK_CM),
    }


def distance_match_ok(value, target, tol=PARKING_TRIGGER_TOL_CM):
    return value != 999 and abs(value - target) <= tol


def front_distance_match_ok(value, target):
    """Strict/asymmetric front match for parking.

    Side and back can use ±3 cm. Front cannot, because accepting target-3 cm
    means the robot is already 3 cm too far forward.

    Good front window:
      target - PARKING_FRONT_TOO_CLOSE_TOL_CM <= front <= target + PARKING_FRONT_TOO_FAR_TOL_CM
    """
    return (
        value != 999
        and (target - PARKING_FRONT_TOO_CLOSE_TOL_CM) <= value <= (target + PARKING_FRONT_TOO_FAR_TOL_CM)
    )


def parking_space_match_status(distances, targets):
    """Check all four directions against their targets.

    Left/right/back use ±3 cm. Front uses a stricter asymmetric window so the
    robot does not start parking too far forward.
    """
    ok = {
        "left": distance_match_ok(distances["left"], targets["left"]),
        "right": distance_match_ok(distances["right"], targets["right"]),
        "front": front_distance_match_ok(distances["front"], targets["front"]),
        "back": distance_match_ok(distances["back"], targets["back"]),
    }
    return ok, all(ok.values())


def format_parking_match(distances, targets, ok):
    return (
        f"L={distances['left']:.1f}/{targets['left']:.1f}({ok['left']}) | "
        f"R={distances['right']:.1f}/{targets['right']:.1f}({ok['right']}) | "
        f"F={distances['front']:.1f}/{targets['front']:.1f}({ok['front']}) | "
        f"B={distances['back']:.1f}/{targets['back']:.1f}({ok['back']})"
    )

def choose_parking_side_from_snapshot(snap):
    """Choose which wall to use for parking from the values measured after unpark."""
    left = snap.get("left", 999)
    right = snap.get("right", 999)

    if left == 999 and right == 999:
        return "right"
    if right == 999:
        return "left"
    if left == 999:
        return "right"

    # Use the closer wall from the sensor values taken after unpark.
    return "right" if right <= left else "left"

def reset_yaw_for_parking():
    global yaw, last_time
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()

def parking_imu_update():
    """Integrate yaw during blocking parking/alignment routines."""
    global yaw, last_time
    now_local = time.time()
    dt_local = now_local - last_time
    last_time = now_local

    raw_gz_dps = read_raw_data(GYRO_ZOUT_H) / 131.0
    Gz_local = raw_gz_dps - gyro_z_bias

    with yaw_lock:
        yaw += Gz_local * dt_local
        if yaw > YAW_CLAMP_DEG:
            yaw = YAW_CLAMP_DEG
        elif yaw < -YAW_CLAMP_DEG:
            yaw = -YAW_CLAMP_DEG
        return yaw

def parking_imu_center_servo(current_yaw_deg):
    if abs(current_yaw_deg) <= PARKING_YAW_DEADBAND_DEG:
        return CENTER_ANGLE
    corr = PARKING_YAW_KP * current_yaw_deg
    corr = max(-PARKING_SERVO_CORR_LIMIT, min(PARKING_SERVO_CORR_LIMIT, corr))
    return int(CENTER_ANGLE + corr)

def mirror_servo_for_left_parking(angle_for_right):
    """Mirror a right-side parking servo angle for left-side parking."""
    return int(180 - angle_for_right)

def parking_phase_1(side_name):
    """
    Parking phase 1:
      - Right parking: reverse with servo 130 until |yaw| >= 45 deg.
      - Left parking: mirrored reverse with servo 50 until |yaw| >= 45 deg.
    """
    print(f"[PARK] Phase 1 ({side_name}): reverse until |yaw| >= 45°", flush=True)
    reset_yaw_for_parking()

    servo_angle = 130 if side_name == "right" else mirror_servo_for_left_parking(130)
    servo_angle = max(50, min(130, servo_angle))

    set_servo_angle(SERVO_CHANNEL, servo_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -PARKING_SPEED)

    while True:
        current_yaw_p = parking_imu_update()
        if abs(current_yaw_p) >= 45.0:
            break
        time.sleep(0.01)

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    print("[PARK] Phase 1 complete.", flush=True)

def parking_phase_2(side_name):
    """
    Parking phase 2:
      - Right parking: reverse with servo 50 until extra yaw >= 30 deg.
      - Left parking: mirrored reverse with servo 130 until extra yaw >= 30 deg.
    """
    print(f"[PARK] Phase 2 ({side_name}): reverse until Δyaw >= 30°", flush=True)

    servo_angle = 50 if side_name == "right" else mirror_servo_for_left_parking(50)
    servo_angle = max(50, min(130, servo_angle))

    set_servo_angle(SERVO_CHANNEL, servo_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -PARKING_SPEED)

    start_yaw_p = parking_imu_update()
    target_delta = 30.0

    while True:
        current_yaw_p = parking_imu_update()
        delta_yaw = abs(current_yaw_p - start_yaw_p)
        if delta_yaw >= target_delta:
            break
        time.sleep(0.01)

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    print("[PARK] Phase 2 complete.", flush=True)

def parking_phase_3(side_name):
    """
    Parking phase 3:
      - Right parking: forward with servo 125 until extra yaw >= 15 deg.
      - Left parking: mirrored forward with servo 55 until extra yaw >= 15 deg.
    """
    print(f"[PARK] Phase 3 ({side_name}): forward until Δyaw >= 15°", flush=True)

    servo_angle = 125 if side_name == "right" else mirror_servo_for_left_parking(125)
    servo_angle = max(50, min(130, servo_angle))

    set_servo_angle(SERVO_CHANNEL, servo_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, PARKING_SPEED)

    start_yaw_p = parking_imu_update()
    target_delta = 15.0

    while True:
        current_yaw_p = parking_imu_update()
        delta_yaw = abs(current_yaw_p - start_yaw_p)
        if delta_yaw >= target_delta:
            break
        time.sleep(0.01)

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    reset_yaw_for_parking()

    print("[PARK] Phase 3 complete. Parking finished.", flush=True)

def run_parking_3phases(side_name):
    """Run the 3-phase parking maneuver."""
    global parking_running
    parking_running = True
    print(f"[PARK] === Starting 3-phase parking on {side_name.upper()} side ===", flush=True)

    parking_phase_1(side_name)
    parking_phase_2(side_name)
    parking_phase_3(side_name)

    parking_running = False
    print("[PARK] === 3-phase parking finished ===", flush=True)

def side_distance_align_for_parking(side_name, duration=3.0):
    """
    Align to the selected wall before searching for the parking trigger.
    Based on the separate parking script's right-wall alignment, with
    mirrored support for left-wall parking.
    """
    print(f"[ALIGN] Starting side-distance alignment for {side_name.upper()} wall", flush=True)
    reset_yaw_for_parking()

    # Phase 1: read side distance for a short time.
    end_time = time.time() + duration
    while time.time() < end_time:
        current_yaw_p = parking_imu_update()
        side_cm = get_best_side_cm(side_name)
        if side_cm != 999:
            print(f"[ALIGN-P1] {side_name}={side_cm:.1f}cm, yaw={current_yaw_p:.2f}", flush=True)
        time.sleep(0.05)

    # Phase 2: straighten yaw while moving forward.
    print("[ALIGN-P2] Straightening yaw to 0 while moving forward.", flush=True)
    stable_count = 0
    phase2_start = time.time()

    while time.time() - phase2_start < PARKING_PARALLEL_MAX_TIME:
        current_yaw_p = parking_imu_update()

        servo_angle = parking_imu_center_servo(current_yaw_p)
        servo_angle = max(75, min(105, servo_angle))

        set_servo_angle(SERVO_CHANNEL, servo_angle)
        set_motor_speed(MOTOR_FWD, MOTOR_REV, PARKING_ALIGN_SPEED)

        print(f"[ALIGN-P2] yaw={current_yaw_p:.2f}, servo={servo_angle}", flush=True)

        if abs(current_yaw_p) < PARKING_YAW_PARALLEL_TOL:
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= PARKING_PARALLEL_STABLE_COUNT:
            print("[ALIGN-P2] Yaw stable near 0; parallel achieved.", flush=True)
            break

        time.sleep(0.05)

    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, PARKING_ALIGN_SPEED)
    reset_yaw_for_parking()
    print("[ALIGN] Done alignment. Servo centered, moving straight.", flush=True)

def detect_parking_magenta_marker():
    """Return (seen, area, bottom_y, cx) for the magenta parking boundary.

    This is used only after the final turn, during parking search.
    The main obstacle / line logic is unchanged.
    """
    try:
        img_rgb = picam2.capture_array()
    except Exception as e:
        print(f"[PARK-MAGENTA] camera read failed: {e}", flush=True)
        return False, 0.0, -1, None

    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    h, w = hsv.shape[:2]

    # Ignore only the very top of the image. The magenta wall can appear on the
    # side or low/front area depending on the robot angle.
    y0 = int(h * 0.10)
    mask = cv2.inRange(hsv[y0:h, :], MAGENTA_LO, MAGENTA_HI)

    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0.0, -1, None

    c = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area < PARKING_MAGENTA_MIN_AREA:
        return False, area, -1, None

    x, y, bw, bh = cv2.boundingRect(c)
    bottom_y = y0 + y + bh
    cx = x + bw / 2.0
    return True, area, bottom_y, cx


def detect_parking_front_obstacle_marker():
    """Return (color, area, box) for a red/green obstacle during parking search.

    This is only a parking safety check. It prevents the robot from starting
    the 3-phase parking when the front sensor is close because of a block.
    """
    try:
        img_rgb = picam2.capture_array()
    except Exception as e:
        print(f"[PARK-OBS] camera read failed: {e}", flush=True)
        return None, 0.0, None

    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    h, w = hsv.shape[:2]

    y0 = int(h * 0.12)
    roi = hsv[y0:h, :]

    mask_red1 = cv2.inRange(roi, RED1_LO, RED1_HI)
    mask_red2 = cv2.inRange(roi, RED2_LO, RED2_HI)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(roi, GREEN_LO, GREEN_HI)

    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((5, 5), np.uint8)

    def clean(mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5)
        return mask

    candidates = []
    for color_name, mask in (("Red", clean(mask_red)), ("Green", clean(mask_green))):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        c = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(c))
        if area < PARKING_OBSTACLE_MIN_AREA:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        y_full = y0 + y
        bottom_y = y_full + bh
        if bottom_y < int(h * PARKING_OBSTACLE_FRONT_Y_FRAC):
            continue
        candidates.append((color_name, area, (x, y_full, x + bw, bottom_y)))

    if not candidates:
        return None, 0.0, None

    candidates.sort(key=lambda item: item[1], reverse=True)
    return candidates[0]


def parking_clear_front_safety(obstacle_color, front_best):
    """Clear a dangerously close front reading during magenta search.

    Important: this never starts parking. It first backs up enough to stop
    pushing/carrying the block, then performs a stronger color-based steer
    around it. This is used only after the final turn during parking search.
    """
    def servo_for_color(color_name):
        if color_name == "Red":
            return LEFT_NEAR, "red obstacle"
        if color_name == "Green":
            return RIGHT_NEAR, "green obstacle"
        return CENTER_ANGLE, "front safety"

    clear_servo, reason = servo_for_color(obstacle_color)

    print(
        f"[PARK-SAFETY] Front close during magenta search: "
        f"front_best={front_best:.1f}cm, reason={reason}. "
        f"Strong clear/avoid, NOT parking.",
        flush=True
    )

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    time.sleep(0.08)

    for attempt in range(1, PARKING_SAFETY_MAX_ATTEMPTS + 1):
        distances_now = get_parking_all_distances()
        current_front = distances_now.get("front_best", 999)
        color_now, area_now, box_now = detect_parking_front_obstacle_marker()

        if color_now is not None:
            obstacle_color = color_now
            clear_servo, reason = servo_for_color(obstacle_color)

        if color_now is None and (current_front == 999 or current_front >= PARKING_SAFETY_FRONT_CLEAR_CM):
            print(
                f"[PARK-SAFETY] clear before attempt {attempt}: "
                f"front={current_front:.1f}, no red/green block visible.",
                flush=True
            )
            break

        print(
            f"[PARK-SAFETY] attempt {attempt}/{PARKING_SAFETY_MAX_ATTEMPTS}: "
            f"front={current_front:.1f}, obstacle={color_now}, area={area_now:.0f}, "
            f"box={box_now}. Reversing first so the robot does not carry the block.",
            flush=True
        )

        # Always reverse first. This is what prevents the bumper from carrying
        # a green/red block while the car is trying to steer around it.
        set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
        set_motor_speed(MOTOR_FWD, MOTOR_REV, -PARKING_SAFETY_CLEAR_SPEED)
        time.sleep(PARKING_SAFETY_REVERSE_TIME_S)
        set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
        time.sleep(0.08)

        distances_after_back = get_parking_all_distances()
        after_front = distances_after_back.get("front_best", 999)
        color_after, area_after, box_after = detect_parking_front_obstacle_marker()

        if color_after is not None:
            obstacle_color = color_after
            clear_servo, reason = servo_for_color(obstacle_color)

        if after_front != 999 and after_front < PARKING_SAFETY_HARD_CLOSE_CM:
            print(
                f"[PARK-SAFETY] still hard-close after reverse: "
                f"front={after_front:.1f}cm. Reversing again before any forward move.",
                flush=True
            )
            continue

        if color_after is None and (after_front == 999 or after_front >= PARKING_SAFETY_FRONT_CLEAR_CM):
            print(
                f"[PARK-SAFETY] obstacle cleared after reverse: "
                f"front={after_front:.1f}, no block visible.",
                flush=True
            )
            break

        # Only now drive forward with steering, after there is some space.
        # Red/green use the same steering tendency as the normal avoidance logic.
        print(
            f"[PARK-SAFETY] steer-clear: obstacle={obstacle_color}, "
            f"servo={clear_servo}, front={after_front:.1f}, area={area_after:.0f}",
            flush=True
        )
        set_servo_angle(SERVO_CHANNEL, clear_servo)
        set_motor_speed(MOTOR_FWD, MOTOR_REV, PARKING_SAFETY_CLEAR_SPEED)
        time.sleep(PARKING_SAFETY_AVOID_TIME_S)
        set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
        set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
        time.sleep(0.08)

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    time.sleep(0.10)

def follow_wall_until_parking_spot(side_name):
    """
    Parking trigger based on passing the magenta parking boundary.

    Logic:
      after the final turn -> drive forward slowly along the selected wall
      -> once magenta is seen, remember it
      -> keep driving until magenta disappears for a few frames
      -> stop and run the 3-phase parking maneuver.

    This avoids the previous distance-only trigger that oscillated between
    front/back/side corrections. Distances are used only for steering and
    emergency crash safety during the search.
    """
    print(f"=== ENTERING {side_name.upper()}-WALL MAGENTA-PASS SEARCH FOR PARKING ===", flush=True)
    print(
        "[PARK] Trigger rule: see MAGENTA once, then park after MAGENTA disappears. "
        "Distances are safety/steering only, not the parking-start trigger.",
        flush=True
    )

    targets = get_parking_target_distances(side_name)
    print(
        f"[PARK] Post-unpark snapshot kept for debug only: "
        f"L={targets['left']:.1f}, R={targets['right']:.1f}, "
        f"F={targets['front']:.1f}, B={targets['back']:.1f}",
        flush=True
    )

    magenta_seen_count = 0
    magenta_lost_count = 0
    magenta_was_seen = False
    search_start = time.time()
    last_debug = 0.0

    def choose_search_servo(current_yaw_p, distances):
        """Keep roughly parallel to the selected parking wall while searching."""
        servo_angle = parking_imu_center_servo(current_yaw_p)
        side_cm = distances.get(side_name, 999)
        side_target = targets.get(side_name, PARKING_SIDE_TARGET_CM)

        if valid_cm_for_parking(side_cm):
            dist_err = side_cm - side_target
            if side_name == "right":
                # Right wall: too far from wall -> steer right (>90)
                servo_angle += PARKING_DIST_KP * dist_err
            else:
                # Left wall: too far from wall -> steer left (<90)
                servo_angle -= PARKING_DIST_KP * dist_err

        return int(max(50, min(130, servo_angle)))

    while True:
        current_yaw_p = parking_imu_update()
        distances = get_parking_all_distances()
        mag_seen, mag_area, mag_bottom_y, mag_cx = detect_parking_magenta_marker()

        if mag_seen:
            magenta_seen_count += 1
            magenta_lost_count = 0
            if magenta_seen_count >= PARKING_MAGENTA_SEEN_FRAMES and not magenta_was_seen:
                magenta_was_seen = True
                print(
                    f"[PARK-MAGENTA] SEEN confirmed: area={mag_area:.0f}, "
                    f"bottom_y={mag_bottom_y}, cx={mag_cx}",
                    flush=True
                )
        else:
            if magenta_was_seen:
                magenta_lost_count += 1
            else:
                magenta_seen_count = 0

        now = time.time()
        if now - last_debug >= PARKING_MAGENTA_DEBUG_EVERY_S:
            print(
                f"[PARK-MAGENTA] seen={mag_seen} was_seen={magenta_was_seen} "
                f"seen_count={magenta_seen_count}/{PARKING_MAGENTA_SEEN_FRAMES} "
                f"lost_count={magenta_lost_count}/{PARKING_MAGENTA_LOST_FRAMES} "
                f"area={mag_area:.0f} bottom_y={mag_bottom_y} "
                f"L={distances['left']:.1f} R={distances['right']:.1f} "
                f"F={distances['front']:.1f} B={distances['back']:.1f} "
                f"yaw={current_yaw_p:.2f}",
                flush=True
            )
            last_debug = now

        # Crash safety while searching. If a front reading is dangerously close,
        # it may be a red/green block. Do NOT start parking from this condition.
        # Clear/avoid it, then continue looking for the magenta pass marker.
        front_best = distances.get("front_best", 999)
        if magenta_was_seen and front_best != 999 and front_best <= PARKING_MAGENTA_FRONT_SAFETY_CM:
            obstacle_color, obstacle_area, obstacle_box = detect_parking_front_obstacle_marker()
            if obstacle_color is not None:
                print(
                    f"[PARK-OBS] {obstacle_color} obstacle blocks parking search: "
                    f"area={obstacle_area:.0f}, box={obstacle_box}",
                    flush=True
                )
            parking_clear_front_safety(obstacle_color, front_best)
            magenta_lost_count = 0
            continue

        # If magenta is extremely low/large, the robot is about to touch the
        # parking wall. Start parking before collision, but never while a red/green
        # obstacle is still blocking the front.
        magenta_very_close = (
            magenta_was_seen
            and mag_seen
            and (
                mag_bottom_y >= PARKING_MAGENTA_CLOSE_BOTTOM_Y
                or mag_area >= PARKING_MAGENTA_CLOSE_AREA
            )
        )
        if magenta_very_close:
            front_best = distances.get("front_best", 999)
            obstacle_color, obstacle_area, obstacle_box = detect_parking_front_obstacle_marker()
            if obstacle_color is not None or (front_best != 999 and front_best <= PARKING_MAGENTA_FRONT_SAFETY_CM):
                print(
                    f"[PARK-BLOCKED] Magenta is very close, but parking is blocked: "
                    f"front_best={front_best:.1f}, obstacle={obstacle_color}, "
                    f"area={obstacle_area:.0f}, box={obstacle_box}. Clearing first.",
                    flush=True
                )
                parking_clear_front_safety(obstacle_color, front_best)
                magenta_lost_count = 0
                continue

            print(
                f"[TRIGGER] Magenta very close/low: area={mag_area:.0f}, "
                f"bottom_y={mag_bottom_y}. Starting parking before wall hit.",
                flush=True
            )
            set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
            set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
            time.sleep(0.15)
            run_parking_3phases(side_name)
            return

        # Main trigger: park only after magenta was seen and then disappeared.
        # Before starting, make one final red/green/front safety check.
        if magenta_was_seen and magenta_lost_count >= PARKING_MAGENTA_LOST_FRAMES:
            front_best = distances.get("front_best", 999)
            obstacle_color, obstacle_area, obstacle_box = detect_parking_front_obstacle_marker()
            if obstacle_color is not None or (front_best != 999 and front_best <= PARKING_MAGENTA_FRONT_SAFETY_CM):
                print(
                    f"[PARK-BLOCKED] Magenta passed, but parking start is blocked: "
                    f"front_best={front_best:.1f}, obstacle={obstacle_color}, "
                    f"area={obstacle_area:.0f}, box={obstacle_box}. Clearing first.",
                    flush=True
                )
                parking_clear_front_safety(obstacle_color, front_best)
                magenta_lost_count = 0
                continue

            print("[TRIGGER] Magenta was seen and then disappeared. Starting parking.", flush=True)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
            set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
            time.sleep(0.15)

            if PARKING_AFTER_MAGENTA_LOST_ROLL_S > 0:
                print(
                    f"[PARK-MAGENTA] Extra roll after magenta lost: "
                    f"{PARKING_AFTER_MAGENTA_LOST_ROLL_S:.2f}s",
                    flush=True
                )
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, PARKING_MAGENTA_SEARCH_SPEED)
                time.sleep(PARKING_AFTER_MAGENTA_LOST_ROLL_S)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
                time.sleep(0.10)

            run_parking_3phases(side_name)
            return

        # If no magenta was found for a long time, keep searching but make the
        # situation obvious in the log. Do not use distance-only parking fallback.
        if (now - search_start) >= PARKING_MAGENTA_SEARCH_TIMEOUT_S:
            print(
                "[PARK-WAIT] Magenta search timeout reached, but no distance-only "
                "fallback will start. Still searching for magenta pass marker.",
                flush=True
            )
            search_start = now

        servo_angle = choose_search_servo(current_yaw_p, distances)
        set_servo_angle(SERVO_CHANNEL, servo_angle)
        set_motor_speed(MOTOR_FWD, MOTOR_REV, PARKING_MAGENTA_SEARCH_SPEED)
        time.sleep(0.05)

def final_turn_right_clear_before_parking():
    """
    Run only after the final turn and before parking search/alignment.
    If the right side is close, move forward while steering left to create
    clearance from the right-side obstacle/wall. If the right side is already
    clear, do not waste distance; stop and continue to parking search.
    """
    print("[PARK-PREP] Final turn complete. Checking RIGHT side before parking.", flush=True)

    start_t = time.time()
    moved_to_clear = False

    while time.time() - start_t < FINAL_PARK_RIGHT_CLEAR_TIME_S:
        right_tof = tof_cm(sensors["right"])
        right_us = get_right_ultra_cm()

        right_candidates = [d for d in (right_tof, right_us) if d != 999]
        right_best = min(right_candidates) if right_candidates else 999

        if right_best < FINAL_PARK_RIGHT_CLEAR_CM:
            moved_to_clear = True
            set_servo_angle(SERVO_CHANNEL, FINAL_PARK_RIGHT_CLEAR_ANGLE)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, FINAL_PARK_RIGHT_CLEAR_SPEED)
            print(
                f"[PARK-PREP] right={right_best:.1f}cm < {FINAL_PARK_RIGHT_CLEAR_CM:.1f}; "
                f"clearing right side, servo={FINAL_PARK_RIGHT_CLEAR_ANGLE}, "
                f"speed={FINAL_PARK_RIGHT_CLEAR_SPEED}",
                flush=True
            )
            time.sleep(0.05)
        else:
            print(
                f"[PARK-PREP] right={right_best:.1f}cm clear; no extra right-clear move needed.",
                flush=True
            )
            break

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    time.sleep(0.15)

    if moved_to_clear:
        print("[PARK-PREP] Right-side clear step done. Starting parking search/alignment.", flush=True)
    else:
        print("[PARK-PREP] Right side already clear. Starting parking search/alignment.", flush=True)


def run_integrated_parking_after_12th_turn():
    """
    Called after the configured mission turns. It uses the wall side chosen from the sensor values
    measured right after unpark, then aligns, follows that wall, and parks.
    """
    global parking_wall_side

    print("[PARK] Mission turns complete. Starting integrated parking process.", flush=True)

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    time.sleep(0.2)

    # Print current values too, but keep the wall side selected after unpark.
    get_parking_sensor_snapshot("PRE-PARK")

    targets = get_parking_target_distances(parking_wall_side)
    print(f"[PARK] Using {parking_wall_side.upper()} wall selected after unpark.", flush=True)
    print(
        f"[PARK] Searching for POST-UNPARK snapshot again: "
        f"L={targets['left']:.1f}, R={targets['right']:.1f}, "
        f"F={targets['front']:.1f}, B={targets['back']:.1f} ±{PARKING_TRIGGER_TOL_CM:.1f}cm",
        flush=True
    )

    # Important: do NOT drive forward in a separate alignment phase here.
    # The last version moved forward for several seconds before checking the
    # 4-distance trigger, so it passed the post-unpark pose and got stuck
    # backing up. We now check the snapshot values continuously while moving.
    follow_wall_until_parking_spot(parking_wall_side)

    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    print("[PARK] Integrated parking process complete.", flush=True)


# ==== HELPERS ====

# ==== STATE PRINTING ====
run_state = None

def set_run_state(new_state: str):
    global run_state
    if new_state != run_state:
        run_state = new_state
        print(f"[STATE] {new_state}", flush=True)

def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest_contour)
    return largest_contour, (x, y), (x + w, y + h), w*h

def thin_shape_reject(candidate, min_extent=0.30, ar_lo=0.35, ar_hi=3.0):
    if not candidate:
        return None
    cnt, tl, br, area = candidate
    x1, y1 = tl; x2, y2 = br
    w, h = x2 - x1, y2 - y1
    if w <= 0 or h <= 0:
        return None
    ar = w / float(h)
    if not (ar_lo <= ar <= ar_hi):
        return None
    extent = cv2.contourArea(cnt) / float(w * h)
    if extent < min_extent:
        return None
    return candidate

def compute_servo_angle(color, area):
    norm_area = max(MIN_AREA, min(MAX_AREA, area))
    closeness = (norm_area - MIN_AREA) / (MAX_AREA - MIN_AREA)
    if color == "Red":
        return int(LEFT_FAR + closeness * (LEFT_NEAR - LEFT_FAR))
    else:
        return int(RIGHT_FAR - closeness * (RIGHT_FAR - RIGHT_NEAR))

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

def valid_orientation(x1,y1,x2,y2):
    dx = x2-x1; dy = y2-y1
    ang = abs(np.degrees(np.arctan2(dy, dx)))
    return LINE_ORIENT_MIN_DEG <= ang <= LINE_ORIENT_MAX_DEG

def max_line_len(lines):
    if lines is None:
        return 0
    m = 0
    for x1, y1, x2, y2 in lines[:, 0]:
        L = int(np.hypot(x2 - x1, y2 - y1))
        if L > m:
            m = L
    return m

def clean_mask(mask):
    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5)
    return mask


def get_line_mask_info(mask, min_area=700):
    """Return whether a blue/orange line mask is visible, its area, and bottom y.

    This is the same robust contour idea used in the manual controller masks:
    it does not depend only on Hough lines, so full-frame blue/orange line
    detections can still trigger the autonomous turn logic.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False, 0, -1

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return False, area, -1

    x, y, w, h = cv2.boundingRect(largest)
    bottom_y = y + h
    return True, area, bottom_y

def preprocess_edges(img_bgr):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    gray = cv2.GaussianBlur(gray, (BLUR_KSIZE, BLUR_KSIZE), 0)
    edges = cv2.Canny(gray, CANNY_LO, CANNY_HI)
    return edges

def detect_line_and_mask(edges, h, w):
    band_y1, band_y2 = int(h*0.45), int(h*0.90)
    roi = edges.copy()
    roi[:band_y1,:] = 0
    roi[band_y2:,:] = 0
    lines = cv2.HoughLinesP(
        roi, rho=1, theta=np.pi/180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LENGTH,
        maxLineGap=HOUGH_MAX_GAP
    )
    line_mask = np.zeros((h, w), dtype=np.uint8)
    seg = None
    if lines is not None:
        for ln in lines:
            x1,y1,x2,y2 = ln[0]
            if valid_orientation(x1,y1,x2,y2):
                if seg is None:
                    seg = (x1,y1,x2,y2)
                cv2.line(line_mask, (x1,y1), (x2,y2), 255, LINE_MASK_THICKNESS)
    return (seg is not None), seg, line_mask

def y_at_center(lines, center_x):
    if lines is None:
        return -1
    ys = []
    for x1,y1,x2,y2 in lines[:,0]:
        if x1 != x2:
            m = (y2 - y1) / float(x2 - x1)
            y = m * (center_x - x1) + y1
            ys.append(y)
        else:
            if x1 == center_x:
                ys.append(max(y1, y2))
    return max(ys) if ys else -1

# ===============================
# SMART UNPARK (two-phase; faster; left-case phase 2 = 65° + extra ~60° left if direction == "left")
# ===============================
start_button = Button(20)

print("\n=== SMART UNPARK START ===")
red_led.off()
green_led.blink(on_time=0.3, off_time=0.3, background=True)

start_button.wait_for_press()
print("Button pressed! tarting sequence...")
green_led.on()
time.sleep(1)

left_dist = tof_cm(sensors["left"])
right_dist = tof_cm(sensors["right"])
print(f"Left={left_dist:.1f}cm | Right={right_dist:.1f}cm")

def UnPark_L():
    # Phase 1: ~45�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    first_angle = UNPARK_LEFT_TURN_ANGLE
    set_servo_angle(SERVO_CHANNEL, first_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= 45.0:
            break
    
    # Phase 2: ~40�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    second_angle = UNPARK_RIGHT_TURN_ANGLE; target_abs_yaw = 40.0
    set_servo_angle(SERVO_CHANNEL, second_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= target_abs_yaw:
            break
    
    # Straighten & reset yaw
    set_servo_angle(SERVO_CHANNEL, UNPARK_CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -UNPARK_STRAIGHT_SPEED)
    time.sleep(0.5)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    with yaw_lock:
        yaw = 0.0
    print("[DONE] Unpark sequence complete. Entering vision/avoid loop...")
    set_run_state("cruise")

def UnPark_R():
        # Phase 1: ~45�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    first_angle = UNPARK_RIGHT_TURN_ANGLE
    set_servo_angle(SERVO_CHANNEL, first_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= 45.0:
            break
    
    # Phase 2: ~40�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    second_angle = UNPARK_LEFT_TURN_ANGLE;  target_abs_yaw = 40.0
    set_servo_angle(SERVO_CHANNEL, second_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= target_abs_yaw:
            break
    
    # Straighten & reset yaw
    set_servo_angle(SERVO_CHANNEL, UNPARK_CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -UNPARK_STRAIGHT_SPEED)
    time.sleep(0.8)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    with yaw_lock:
        yaw = 0.0
    print("[DONE] Unpark sequence complete. Entering vision/avoid loop...")
    set_run_state("cruise")

if left_dist > right_dist:
    direction = "left"
    UnPark_L()
    print("? Choosing LEFT (more open).")
else:
    direction = "right"
    UnPark_R()
    print("? Choosing RIGHT (more open).")

# Take sensor values immediately after unpark, as requested.
# These values decide which wall the integrated parking routine will use later,
# after the configured mission turns.
post_unpark_sensor_snapshot = get_parking_sensor_snapshot("POST-UNPARK")
parking_wall_side = choose_parking_side_from_snapshot(post_unpark_sensor_snapshot)
print(f"[POST-UNPARK] Parking wall selected: {parking_wall_side.upper()}", flush=True)

# ==== FSM STATES & RUNTIME VARIABLES ====

STATE_CRUISE    = "cruise"
STATE_TURN      = "turn"
STATE_POST_TURN = "post_turn"
STATE_AVOID     = "avoid_obstacle"
STATE_FRONT_EMERGENCY_BACK = "front_emergency_back"

fsm_state        = STATE_CRUISE
state_start_time = time.time()

# Obstacle lock & avoidance phases
obstacle_lock_color      = None   # "Red" or "Green" when in AVOID
obstacle_lock_last_area  = 0
obstacle_lock_last_box   = None   # (x1, y1, x2, y2)
obstacle_clear_streak    = 0      # consecutive frames without the locked obstacle
avoid_phase              = None   # "back_off" or "forward"
avoid_phase_start        = 0.0
avoid_direction          = None   # "left" or "right" during reverse

# Front emergency reverse runtime
front_emergency_start = 0.0
front_emergency_recover_until = 0.0
front_emergency_recover_angle = CENTER_ANGLE
last_front_emergency_time = -999.0
emergency_saw_line = False

# Turn / line detection & gating
blue_gate_streak         = 0
last_turn_end_time       = -1.0
next_turn_allowed_time   = 0.0
post_turn_line_ignore_until = 0.0

turn_dir                 = None   # "Left" or "Right"
turn_target_abs_yaw      = TURN_TARGET_DEG
turn_pre_yaw_error       = 0.0
post_turn_start_time     = 0.0

# Lap & turn counters
turn_count = 0
lap_count  = 1

# Color streak for obstacle locking
last_color         = None
color_hold_streak  = 0

# Servo state
current_servo_angle = CENTER_ANGLE
last_time           = time.time()
settle_until_ts     = 0.0  # already defined above, we just make sure it exists

# ---- Simple debug helper ----
def dbg(msg: str):
    print(f"[DBG {time.time():.2f}] {msg}", flush=True)

try:
    # initial “safe” motor state
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    time.sleep(0.2)

    dbg("FSM start -> CRUISE")

    while True:
        now = time.time()
        dt  = now - last_time
        last_time = now

        # ===== IMU / YAW UPDATE =====
        raw_gz_dps = read_raw_data(GYRO_ZOUT_H) / 131.0
        Gz         = raw_gz_dps - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            if yaw > YAW_CLAMP_DEG:
                yaw = YAW_CLAMP_DEG
            elif yaw < -YAW_CLAMP_DEG:
                yaw = -YAW_CLAMP_DEG
            current_yaw = yaw

        # Drift / bias update only in cruise
        if fsm_state == STATE_CRUISE:
            near_center = abs(current_servo_angle - CENTER_ANGLE) <= STRAIGHT_SERVO_WINDOW
            if near_center and abs(raw_gz_dps) < DRIFT_GZ_THRESH:
                gyro_z_bias = (1.0 - BIAS_ALPHA)*gyro_z_bias + BIAS_ALPHA*raw_gz_dps
                yaw *= (1.0 - min(1.0, SOFT_DECAY_RATE * dt))

        # Default target angle (will be overwritten by each state)
        target_angle = current_servo_angle

        # ===== VISION: IMAGE, LINES, BOXES =====
        #img     = picam2.capture_array()
        #img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        #imgHSV  = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        #h_img, w_img = img_bgr.shape[:2]
        #center_x = w_img // 2


        # ===== VISION: IMAGE, LINES, BOXES =====
        # Grab one RGB frame from the camera
        img_rgb = picam2.capture_array()

        # Full-frame conversions (for lines, masks, etc.)
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        imgHSV  = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
        h_img, w_img = img_bgr.shape[:2]
        center_x = w_img // 2
        obstacle_lock_y_min = int(h_img * OBSTACLE_LOCK_Y_FRAC)



                # --- Crop to active area using per-side percentages ---
        h_full, w_full = h_img, w_img  # from img_bgr

        x1 = int(w_full * LEFT_CROP_PCT)
        x2 = int(w_full * (1.0 - RIGHT_CROP_PCT))
        y1 = int(h_full * TOP_CROP_PCT)
        y2 = int(h_full * (1.0 - BOTTOM_CROP_PCT))

        # Safety clamp (just in case)
        x1 = max(0, min(x1, w_full - 1))
        x2 = max(x1 + 1, min(x2, w_full))
        y1 = max(0, min(y1, h_full - 1))
        y2 = max(y1 + 1, min(y2, h_full))

        # Crop ROI (in RGB space)
        roi_rgb = img_rgb[y1:y2, x1:x2]

        # Visualize active area on full frame (yellow rectangle)
        img_disp = img_bgr.copy()
        cv2.rectangle(img_disp, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 2)
        #cv2.imshow("Original with active area", img_disp)

        # From now on we work only on the cropped region for obstacles
        frame_bgr = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2BGR)
        hsv       = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)


        # --- Diagonal line mask (for excluding from red/green) ---
        edges_full = preprocess_edges(img_bgr)
        _, _, line_mask_diag = detect_line_and_mask(edges_full, h_img, w_img)

        # --- ORANGE & BLUE line detection (Hough) ---

        # Build a WIDER full-frame red-ish block mask, then remove it from orange.
        # This prevents red obstacles/reflections from being detected together with orange lines.
        mask_red_block1 = cv2.inRange(imgHSV, RED_BLOCK1_LO, RED_BLOCK1_HI)
        mask_red_block2 = cv2.inRange(imgHSV, RED_BLOCK2_LO, RED_BLOCK2_HI)
        mask_red_block  = cv2.bitwise_or(mask_red_block1, mask_red_block2)

        # Dilate so the orange mask also loses red edges/reflections.
        red_block_kernel = np.ones((15, 15), np.uint8)
        mask_red_block = cv2.dilate(mask_red_block, red_block_kernel, iterations=1)

        mask_orange = cv2.inRange(imgHSV, ORANGE_LO, ORANGE_HI)
        mask_orange = cv2.bitwise_and(mask_orange, cv2.bitwise_not(mask_red_block))

        mask_blue = cv2.inRange(imgHSV, BLUE_LO, BLUE_HI)

        # Same reliable idea as the manual controller: clean the full-frame
        # blue/orange masks and use the largest contour as an additional line trigger.
        mask_orange = clean_mask(mask_orange)
        mask_blue = clean_mask(mask_blue)

        # Contour-based line detection from the color masks.
        blue_seen_mask, blue_area_mask, blue_bottom_y = get_line_mask_info(mask_blue, min_area=700)
        orange_seen_mask, orange_area_mask, orange_bottom_y = get_line_mask_info(mask_orange, min_area=700)

        # Keep Hough as an extra check, but no longer depend only on it.
        edges_orange = cv2.Canny(mask_orange, 50, 150)
        edges_blue = cv2.Canny(mask_blue, 50, 150)

        lines_orange = cv2.HoughLinesP(
            edges_orange, 1, np.pi/180,
            threshold=50, minLineLength=50, maxLineGap=10
        )
        lines_blue = cv2.HoughLinesP(
            edges_blue, 1, np.pi/180,
            threshold=50, minLineLength=50, maxLineGap=10
        )

        blue_len_max = max_line_len(lines_blue)
        orange_len_max = max_line_len(lines_orange)

        orange_y = y_at_center(lines_orange, center_x)
        blue_y = y_at_center(lines_blue, center_x)

        # Old Hough trigger.
        blue_trigger_hough = (
            blue_y >= LINE_CENTER_BLUE_Y_MIN and
            blue_len_max >= BLUE_MIN_LEN_PX
        )
        orange_trigger_hough = (
            orange_y >= LINE_CENTER_ORANGE_Y_MIN and
            orange_len_max >= BLUE_MIN_LEN_PX
        )

        # New mask trigger, matching the manual-controller behavior.
        blue_trigger_mask = (
            blue_seen_mask and
            blue_bottom_y >= LINE_CENTER_BLUE_Y_MIN
        )
        orange_trigger_mask = (
            orange_seen_mask and
            orange_bottom_y >= LINE_CENTER_ORANGE_Y_MIN
        )

        # Final line trigger: Hough OR clean color-mask contour.
        blue_trigger = blue_trigger_hough or blue_trigger_mask
        orange_trigger = orange_trigger_hough or orange_trigger_mask
        line_trigger_raw = blue_trigger or orange_trigger

        # If the line appears while reversing in emergency, remember it so the
        # robot turns immediately after the backup instead of going straight again.
        if fsm_state == STATE_FRONT_EMERGENCY_BACK and line_trigger_raw:
            emergency_saw_line = True

        # Choose the “closest” line Y for comparison with obstacle.
        valid_ys = []
        if blue_trigger_hough and blue_y >= 0:
            valid_ys.append(blue_y)
        if orange_trigger_hough and orange_y >= 0:
            valid_ys.append(orange_y)
        if blue_trigger_mask and blue_bottom_y >= 0:
            valid_ys.append(blue_bottom_y)
        if orange_trigger_mask and orange_bottom_y >= 0:
            valid_ys.append(orange_bottom_y)
        line_y_for_turn = max(valid_ys) if valid_ys else -1

        # --- RED / GREEN obstacle masks ---
        # IMPORTANT:
        # Obstacles use the CROPPED ROI hsv, not full-frame imgHSV.
        # This reduces detections outside the useful track area, while the
        # smaller crop still lets obstacles enter view earlier.

        # Crop the full-frame orange/diagonal masks so they match the obstacle ROI size.
        mask_orange_roi = mask_orange[y1:y2, x1:x2]
        line_mask_diag_roi = None
        if line_mask_diag is not None:
            line_mask_diag_roi = line_mask_diag[y1:y2, x1:x2]

        # PINK to exclude from red, using cropped ROI
        mask_pink = cv2.inRange(
            hsv,
            np.array([140, 60, 120], dtype=np.uint8),
            np.array([170, 255, 255], dtype=np.uint8),
        )

        # Red obstacle mask from CROPPED hsv
        mask_red1 = cv2.inRange(hsv, RED1_LO, RED1_HI)
        mask_red2 = cv2.inRange(hsv, RED2_LO, RED2_HI)
        mask_red  = cv2.bitwise_or(mask_red1, mask_red2)

        # Remove orange/pink/diagonal-line pixels from red obstacle mask
        mask_red  = cv2.bitwise_and(mask_red, cv2.bitwise_not(mask_orange_roi))
        mask_red  = cv2.bitwise_and(mask_red, cv2.bitwise_not(mask_pink))
        if line_mask_diag_roi is not None:
            mask_red = cv2.bitwise_and(mask_red, cv2.bitwise_not(line_mask_diag_roi))

        # Green obstacle mask from CROPPED hsv
        mask_green = cv2.inRange(hsv, GREEN_LO, GREEN_HI)
        if line_mask_diag_roi is not None:
            mask_green = cv2.bitwise_and(mask_green, cv2.bitwise_not(line_mask_diag_roi))

        # Morphological cleanup
        k3 = np.ones((3,3), np.uint8)
        k5 = np.ones((5,5), np.uint8)
        def morph(m):
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k3)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k5)
            return m

        mask_red   = morph(mask_red)
        mask_green = morph(mask_green)

        boxes = []

        # Since obstacle masks are cropped, convert detected boxes back to FULL image coordinates.
        # The rest of the code compares boxes with full-frame line/car-box coordinates.
        def candidate_box_to_full(candidate):
            _, tl, br, area = candidate
            return (tl[0] + x1, tl[1] + y1, br[0] + x1, br[1] + y1), area

        red_box_full = None
        green_box_full = None

        red_data = thin_shape_reject(get_largest_contour(mask_red, min_area=MIN_AREA))
        if red_data:
            red_box_full, area = candidate_box_to_full(red_data)
            boxes.append(("Red", area, red_box_full))

        green_data = thin_shape_reject(get_largest_contour(mask_green, min_area=MIN_AREA))
        if green_data:
            green_box_full, area = candidate_box_to_full(green_data)
            boxes.append(("Green", area, green_box_full))

        # For existing obstacle avoidance logic, keep the original behavior:
        # choose the largest detected obstacle by area.
        if boxes:
            boxes.sort(key=lambda b: b[1], reverse=True)
            chosen_color, chosen_area, chosen_box = boxes[0]
        else:
            chosen_color = None
            chosen_area  = 0
            chosen_box   = None

        # For turn timing only, use the closest visible obstacle.
        # In the camera image, larger bottom-y means visually closer to the robot.
        if boxes:
            closest_turn_obstacle_color, closest_turn_obstacle_area, closest_turn_obstacle_box = max(
                boxes,
                key=lambda b: b[2][3]
            )
        else:
            closest_turn_obstacle_color = None
            closest_turn_obstacle_area  = 0
            closest_turn_obstacle_box   = None

        # --- "Car box" region in front of the robot (in image coordinates) ---
        center_x_img = w_img // 2
        car_width    = 500    # tune these two values to match your camera FOV
        car_height   = 150

        bottom_y = h_img - 10
        car_box = (
            center_x_img - car_width // 2,
            bottom_y - car_height,
            center_x_img + car_width // 2,
            bottom_y
        )

        # ==== BLUE-BACKWARD LOGIC (immediate if obstacle intersects car_box) ====
        if not in_blue_backward and fsm_state not in (STATE_TURN, STATE_POST_TURN, STATE_FRONT_EMERGENCY_BACK):
            now_ts = time.time()
            # red_box_full / green_box_full hold full-frame coordinates
            if red_box_full is not None and boxes_intersect(car_box, red_box_full):
                # immediate escape when red box is inside car_box
                in_blue_backward    = True
                blue_backward_start = now_ts
                back_follow_angle   = LEFT_NEAR    # after back, follow red to the left
                set_run_state("red obstacle – close escape backward")
                # reverse straight first
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)

            elif green_box_full is not None and boxes_intersect(car_box, green_box_full):
                in_blue_backward    = True
                blue_backward_start = now_ts
                back_follow_angle   = RIGHT_NEAR   # after back, follow green to the right
                set_run_state("green obstacle – close escape backward")
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)

        if in_blue_backward:
            # hold wheels straight while reversing
            set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)

            if (time.time() - blue_backward_start) >= BLUE_BACK_DURATION:
                # done reversing: reset yaw & go forward again
                with yaw_lock:
                    yaw = 0.0
                in_blue_backward = False
                settle_until_ts  = time.time() + SETTLE_DURATION

                # start forward cruise
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                set_run_state("cruise")

                # briefly bias steering toward the side of the obstacle we avoided
                if back_follow_angle is not None:
                    post_back_follow_until = time.time() + POST_BACK_FOLLOW_S

            # while in this mode, skip the rest of the loop (no FSM updates)
            if cv2.waitKey(1) in [27, ord('q')]:
                dbg("ESC pressed in blue-backward, exiting")
                break
            continue



        # Horizontal position of chosen box (for x-aware steering)
        if chosen_box is not None:
            x1, y1, x2, y2 = chosen_box
            box_center_x   = 0.5 * (x1 + x2)
            # offset in [-1..1]; >0 means box is to the RIGHT of image center
            box_offset_norm = (box_center_x - center_x) / float(center_x)
        else:
            box_center_x   = None
            box_offset_norm = 0.0

        # Update color streak used for locking obstacle (decay instead of hard reset)
        if chosen_color is not None:
            if chosen_color == last_color:
                # seeing the same color again -> build up confidence
                color_hold_streak = min(color_hold_streak + 1, 12)
            else:
                # switched color (Red <-> Green) -> don't nuke, but reduce
                last_color = chosen_color
                color_hold_streak = max(1, color_hold_streak - 1)
        else:
            # nothing seen this frame -> slowly decay
            color_hold_streak = max(0, color_hold_streak - 1)

        # Which is “closer”: line or obstacle?
        closest_obstacle_y = chosen_box[3] if chosen_box is not None else -1
        obstacle_closer = False
        line_closer     = False

        if chosen_area > 6000:
            obstacle_closer = True
            line_closer     = False
        else:
            if line_trigger_raw and line_y_for_turn >= 0 and closest_obstacle_y >= 0:
                if closest_obstacle_y > line_y_for_turn + OBSTACLE_LINE_MARGIN_PX:
                    obstacle_closer = True
                elif line_y_for_turn + OBSTACLE_LINE_MARGIN_PX > closest_obstacle_y:
                    line_closer = True
                else:
                    line_closer = True
            elif line_trigger_raw and closest_obstacle_y < 0:
                line_closer = True
            elif (not line_trigger_raw) and closest_obstacle_y >= 0:
                obstacle_closer = True
            # --- DEBUG: what do we see & who wins, line or obstacle? ---
            if chosen_box is not None or line_trigger_raw:
                if chosen_box is not None:
                    x1, y1, x2, y2 = chosen_box
                    cx = 0.5 * (x1 + x2)
                    off_norm = (cx - center_x) / float(center_x)  # [-1..1], >0 = box on right
                else:
                    cx = None
                    off_norm = 0.0
        
                dbg(
                    f"DETECT: color={chosen_color} area={chosen_area} "
                    f"streak={color_hold_streak} cx={cx} off={off_norm:.2f} "
                    f"line_trig={line_trigger_raw} line_y={line_y_for_turn} "
                    f"line_closer={line_closer} obs_closer={obstacle_closer}"
                )

        # ===== DISTANCES (ToF + ultrasonic) =====
        f_cm = tof_cm(sensors["front"])
        b_cm = tof_cm(sensors["back"])
        l_cm = tof_cm(sensors["left"])
        r_cm = tof_cm(sensors["right"])

        f_ultra = get_front_ultra_cm()
        l_ultra = get_left_ultra_cm()
        r_ultra = get_right_ultra_cm()

        # Gate for starting a turn (based on front ultrasonic)
        tof_line_turn_gate = (TURN_FRONT_MIN_CM <= f_ultra <= TURN_FRONT_MAX_CM)

        # Dynamic turn trigger:
        # closest green obstacle -> turn when front < 85 cm
        # closest red obstacle   -> turn when front < 70 cm
        # no visible obstacle    -> turn when front < 76 cm
        turn_front_trigger = get_turn_front_trigger(direction, closest_turn_obstacle_color)

        # Turn cooldown logic
        time_since_last_turn = now - last_turn_end_time if last_turn_end_time > 0 else 1e9
        out_of_cooldown     = time_since_last_turn >= TURN_COOLDOWN_S
        out_of_min_interval = time_since_last_turn >= TURN_MIN_INTERVAL_S

        # Consecutive frames condition for line confirmation
        if (fsm_state == STATE_CRUISE and
            line_trigger_raw and
            tof_line_turn_gate and
            now >= post_turn_line_ignore_until and
            out_of_cooldown and out_of_min_interval and
            not obstacle_closer):

            blue_gate_streak += 1
        else:
            blue_gate_streak = 0

        line_confirmed = (blue_gate_streak >= LINE_DETECT_CONSEC_FRAMES)

        # ===== EMERGENCY FRONT ESCAPE =====
        # Trigger only when the front is dangerously close AND there is no obstacle
        # or turn line currently detected. This is for the case where the robot
        # is heading into the wall after losing an obstacle/line.
        front_candidates = [d for d in (f_ultra, f_cm) if d != 999]
        front_best = min(front_candidates) if front_candidates else 999
        no_obstacle_seen = (chosen_color is None and chosen_box is None)
        no_line_seen = (not line_trigger_raw)

        if (fsm_state == STATE_CRUISE and
            not in_blue_backward and
            no_obstacle_seen and
            no_line_seen and
            front_best != 999 and
            front_best < FRONT_EMERGENCY_CM and
            (now - last_front_emergency_time) >= FRONT_EMERGENCY_COOLDOWN_S):

            # Choose the recovery angle from side distances.
            # Use the same side-collision steering constants that cruise already uses.
            l_side_emg = l_ultra if l_ultra < 999 else l_cm
            r_side_emg = r_ultra if r_ultra < 999 else r_cm

            if l_side_emg < SIDE_COLLIDE_CM and l_side_emg < r_side_emg:
                front_emergency_recover_angle = LEFT_COLLIDE_ANGLE
            elif r_side_emg < SIDE_COLLIDE_CM and r_side_emg < l_side_emg:
                front_emergency_recover_angle = RIGHT_COLLIDE_ANGLE
            else:
                # If side data is unclear, use yaw correction after the reverse.
                front_emergency_recover_angle = CENTER_ANGLE

            front_emergency_start = now
            emergency_saw_line = False
            fsm_state = STATE_FRONT_EMERGENCY_BACK
            state_start_time = now
            blue_gate_streak = 0

            dbg(
                f"FSM CRUISE -> FRONT_EMERGENCY_BACK: front_best={front_best:.1f}cm, "
                f"f_ultra={f_ultra:.1f}, f_tof={f_cm:.1f}, "
                f"l_side={l_side_emg:.1f}, r_side={r_side_emg:.1f}, "
                f"recover_angle={front_emergency_recover_angle}"
            )
            set_run_state("front emergency reverse")

        # ====== FSM ======
        if fsm_state == STATE_FRONT_EMERGENCY_BACK:
            # ---- FRONT EMERGENCY: reverse straight for 1.5 seconds ----
            target_angle = CENTER_ANGLE
            set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, -FRONT_EMERGENCY_BACK_SPEED)

            if (now - front_emergency_start) >= FRONT_EMERGENCY_BACK_DURATION:
                last_front_emergency_time = now

                if emergency_saw_line:
                    # Stop reverse before starting the turn.
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
                    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)

                    # Start the real line turn immediately.
                    turn_dir = "Left" if direction == "left" else "Right"
                    turn_target_abs_yaw = TURN_TARGET_DEG

                    # Reset yaw ONLY for measuring this turn.
                    # This is not a cruise/emergency yaw reset.
                    with yaw_lock:
                        yaw = 0.0
                        current_yaw = 0.0

                    fsm_state = STATE_TURN
                    state_start_time = now
                    blue_gate_streak = 0
                    emergency_saw_line = False

                    dbg(
                        f"FRONT_EMERGENCY_BACK -> TURN because line was seen during backup. "
                        f"dir={turn_dir}, target_yaw={turn_target_abs_yaw:.1f}"
                    )
                    set_run_state(f"emergency line – turn {turn_dir.lower()}")

                else:
                    front_emergency_recover_until = now + FRONT_EMERGENCY_RECOVER_S

                    fsm_state = STATE_CRUISE
                    state_start_time = now
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, FRONT_EMERGENCY_RECOVER_SPEED)
                    dbg(
                        f"FRONT_EMERGENCY_BACK done after {now - front_emergency_start:.2f}s; "
                        f"recover_until={front_emergency_recover_until:.2f}, "
                        f"recover_angle={front_emergency_recover_angle}"
                    )
                    set_run_state("cruise")

        elif fsm_state == STATE_CRUISE:
            # ---- CRUISE: go straight with wall safety ----
            set_run_state("cruise")
            set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)

            # Base IMU keep-straight (strong gains – no obstacle locked)
            target_angle = imu_center_servo(
                current_yaw,
                YAW_DEADBAND_DEG_STRONG,
                YAW_KP_STRONG,
                SERVO_CORR_LIMIT_STRONG
            )

            # Wall collision correction using ultrasonic (fallback to ToF)
            l_side = l_ultra if l_ultra < 999 else l_cm
            r_side = r_ultra if r_ultra < 999 else r_cm

            if l_side < SIDE_COLLIDE_CM and r_side >= SIDE_COLLIDE_CM:
                target_angle = LEFT_COLLIDE_ANGLE
            elif r_side < SIDE_COLLIDE_CM and l_side >= SIDE_COLLIDE_CM:
                target_angle = RIGHT_COLLIDE_ANGLE

            # --- DEBUG: why are we staying in CRUISE? ---
            dbg(
                f"CRUISE: color={chosen_color} streak={color_hold_streak} "
                f"line_closer={line_closer} obs_closer={obstacle_closer} "
                f"next_obs={closest_turn_obstacle_color} turn_thr={turn_front_trigger:.1f} "
                f"f_ultra={f_ultra:.1f} yaw={current_yaw:.1f} state={fsm_state}"
            )
            
            # ---- Priority: lock obstacle -> AVOID state (VISION ONLY) ----
            if (chosen_color is not None and
                chosen_box is not None and
                color_hold_streak >= COLOR_HOLD_FRAMES and
                not line_closer):

                x1, y1, x2, y2 = chosen_box

                # Vision-only "closeness":
                # 1) box is large enough
                # 2) bottom is low in the image (close to robot)
                area_ok = (chosen_area >= OBSTACLE_LOCK_AREA_MIN)
                y_ok    = (y2 >= obstacle_lock_y_min)

                if area_ok and y_ok:
                    obstacle_lock_color     = chosen_color
                    obstacle_lock_last_area = chosen_area
                    obstacle_lock_last_box  = chosen_box
                    obstacle_clear_streak   = 0
                    avoid_phase             = "back_off"
                    avoid_phase_start       = now

                    # For red we want to pass on the right;
                    # for green on the left -> reverse opposite first.
                    avoid_direction = "left" if obstacle_lock_color == "Red" else "right"

                    dbg(
                        f"FSM CRUISE -> AVOID (lock {obstacle_lock_color}, "
                        f"area={chosen_area}, box={chosen_box}, dir={avoid_direction}, "
                        f"y2={y2}, y_thr={obstacle_lock_y_min})"
                    )
                    fsm_state        = STATE_AVOID
                    state_start_time = now
                    set_run_state(f"avoid lock – {obstacle_lock_color.lower()}")
                else:
                    dbg(
                        f"NO AVOID (vision gate): color={chosen_color} "
                        f"streak={color_hold_streak} area={chosen_area} "
                        f"area_ok={area_ok} y2={y2} y_ok={y_ok} "
                        f"y_thr={obstacle_lock_y_min}"
                    )


            # ---- Otherwise: line-based turn (if allowed) ----
            elif (line_confirmed and
                  (turn_count < TOTAL_TURNS) and
                  not obstacle_closer) and f_ultra < turn_front_trigger:

                fsm_state        = STATE_TURN
                state_start_time = now
                turn_dir = "Left" if direction == "left" else "Right"

                trig_color = "blue" if blue_trigger and not orange_trigger else \
                             "orange" if orange_trigger and not blue_trigger else \
                             "both"
                dbg(
                    f"FSM CRUISE -> TURN (dir={turn_dir}, trig={trig_color}, "
                    f"next_obs={closest_turn_obstacle_color}, "
                    f"front_trigger={turn_front_trigger:.1f}, f_ultra={f_ultra:.1f})"
                )

                if blue_trigger and not orange_trigger:
                    set_run_state(f"blue line – turn {turn_dir.lower()}")
                elif orange_trigger and not blue_trigger:
                    set_run_state(f"orange line – turn {turn_dir.lower()}")
                else:
                    set_run_state(f"line – turn {turn_dir.lower()}")

                # Yaw-aware target:
                # Check how angled the robot is before turning, then add/subtract
                # that error so the turn finishes closer to exactly 90 degrees.
                turn_pre_yaw_error = current_yaw
                turn_target_abs_yaw = compute_yaw_corrected_turn_target(turn_dir, turn_pre_yaw_error)

                dbg(
                    f"YAW-CORRECTED TURN TARGET: pre_yaw={turn_pre_yaw_error:.1f} "
                    f"dir={turn_dir} target_yaw={turn_target_abs_yaw:.1f}"
                )

                # Reset yaw for measuring only the turn rotation.
                with yaw_lock:
                    yaw = 0.0

        elif fsm_state == STATE_TURN:
            # ---- TURN: 80° line-based turn ----
            if turn_dir == "Left":
                target_angle = TURN_LEFT_SERVO
            else:
                target_angle = TURN_RIGHT_SERVO

            set_motor_speed(MOTOR_FWD, MOTOR_REV, TURN_MOTOR_SPEED)
            effective_turn_stop_yaw = max(0.0, turn_target_abs_yaw - TURN_STOP_EARLY_DEG)
            dbg(
                f"TURNING: dir={turn_dir}, yaw={current_yaw:.1f}, "
                f"target={turn_target_abs_yaw:.1f}, stop_at={effective_turn_stop_yaw:.1f}, "
                f"servo={target_angle}, speed={TURN_MOTOR_SPEED}"
            )

            # Stop rule: yaw-based, with a small early-stop margin to reduce
            # physical overshoot caused by motor/servo inertia.
            if abs(current_yaw) >= effective_turn_stop_yaw:
                # Corrected turns should finish straight, so reset yaw error to zero.
                # The old YAW_RESET_AFTER_LEFT/RIGHT constants are kept above for compatibility,
                # but not used here because the target was already corrected before the turn.
                with yaw_lock:
                    yaw = 0.0
                    current_yaw = yaw

                # Turn / lap counting
                turn_count += 1
                turn_in_lap = ((turn_count - 1) % TURNS_PER_LAP) + 1
                lap_count   = (turn_count - 1) // TURNS_PER_LAP + 1
                dbg(f"TURN end: dir={turn_dir}, yaw={current_yaw:.1f}, turn_count={turn_count}, lap={lap_count}")
                print(f"[LAP] turn {turn_in_lap} / lap {lap_count}", flush=True)

                # If all laps done -> start integrated parking instead of final straight stop
                if turn_count >= TOTAL_TURNS:
                    print("[LAP] Mission turns completed. Preparing for parking.", flush=True)
                    final_turn_right_clear_before_parking()
                    print("[LAP] Starting parking process.", flush=True)
                    run_integrated_parking_after_12th_turn()
                    break

                # Otherwise go into post-turn reverse state
                fsm_state              = STATE_POST_TURN
                state_start_time       = now
                post_turn_start_time   = now
                last_turn_end_time     = now
                next_turn_allowed_time = now + TURN_COOLDOWN_SEC
                post_turn_line_ignore_until = now + POST_TURN_LINE_IGNORE_S
                settle_until_ts        = time.time() + SETTLE_DURATION
                dbg("FSM TURN -> POST_TURN (start reverse)")
                set_run_state("post-turn reverse")

                target_angle = CENTER_ANGLE
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -POST_TURN_BACK_SPEED)

        elif fsm_state == STATE_POST_TURN:
            # ---- POST_TURN: back up straight a bit ----
            target_angle = CENTER_ANGLE
            set_motor_speed(MOTOR_FWD, MOTOR_REV, -POST_TURN_BACK_SPEED)

            back_cm      = tof_cm(sensors["back"])
            elapsed_back = now - post_turn_start_time

            if back_cm >= POST_TURN_BACK_CLEAR_CM or elapsed_back >= POST_TURN_BACK_TIMEOUT_S:
                dbg(f"POST_TURN done: back_cm={back_cm:.1f}, elapsed={elapsed_back:.2f}")
                # Done backing up – resume cruise
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                fsm_state        = STATE_CRUISE
                state_start_time = now
                settle_until_ts  = time.time() + SETTLE_DURATION
                dbg("FSM POST_TURN -> CRUISE")
                set_run_state("cruise")

        elif fsm_state == STATE_AVOID:
            # ---- AVOID: obstacle avoidance has priority ----
            locked_seen = (chosen_color == obstacle_lock_color)

            if avoid_phase == "back_off":
                # Short reverse with steering away, to create clearance
                elapsed = now - avoid_phase_start
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
                target_angle = LEFT_NEAR if avoid_direction == "left" else RIGHT_NEAR

                if elapsed >= AVOID_BACK_DURATION:
                    avoid_phase       = "forward"
                    avoid_phase_start = now
                    dbg(f"AVOID phase switch: back_off -> forward (color={obstacle_lock_color})")
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                    obstacle_clear_streak = 0

            elif avoid_phase == "forward":
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
    
                if locked_seen and chosen_box is not None:
                    # Obstacle still visible – follow around it
                    area       = chosen_area
                    base_angle = compute_servo_angle(obstacle_lock_color, area)
    
                    # --- X-position of the box in the image ---
                    x1, y1, x2, y2 = chosen_box
                    cx = 0.5 * (x1 + x2)
                    offset_norm = (cx - center_x) / float(center_x)   # [-1..1], >0 = box on RIGHT
                    offset_norm = max(-XPOS_MAX_OFFSET, min(XPOS_MAX_OFFSET, offset_norm))
    
                    # --- Desired side of the camera for each color ---
                    # "Pass red from right"  -> keep red on RIGHT side of the image
                    # "Pass green from left" -> keep green on LEFT side of the image
                    if obstacle_lock_color == "Red":
                        target_offset = +0.45   # red box should sit on right half
                    else:
                        target_offset = -0.45   # green box should sit on left half
    
                    # Position error in normalized units
                    err = target_offset - offset_norm
    
                    # Use XPOS_GAIN_DEG as a proportional gain on this error.
                    # SERVO_XPOS_SIGN lets you flip direction if needed.
                    correction_deg = SERVO_XPOS_SIGN * XPOS_GAIN_DEG * err
    
                    # Combine: base color-based curve + x-position correction
                    target_angle = int(base_angle + correction_deg)
    
                    # Clamp for safety
                    target_angle = max(60, min(120, target_angle))
    
                    # --- DEBUG: AVOID forward steering details ---
                    dbg(
                        f"AVOID FWD: color={obstacle_lock_color} cx={cx:.1f} "
                        f"off={offset_norm:.2f} t_off={target_offset:.2f} "
                        f"err={err:.2f} base={base_angle} yaw={current_yaw:.1f} "
                        f"angle={target_angle}"
                    )
    
                else:
                    # Locked obstacle not seen this frame – drive roughly straight
                    target_angle = imu_center_servo(
                        current_yaw,
                        YAW_DEADBAND_DEG_BASE,
                        YAW_KP_BASE,
                        SERVO_CORR_LIMIT_BASE
                    )
    
                # Hysteresis: we only exit AVOID when the locked color is
                # missing for OBSTACLE_CLEAR_FRAMES consecutive frames.
                if locked_seen:
                    obstacle_clear_streak = 0
                else:
                    obstacle_clear_streak += 1
    
                if obstacle_clear_streak >= OBSTACLE_CLEAR_FRAMES:
                    dbg(f"AVOID done: color={obstacle_lock_color}, clear_streak={obstacle_clear_streak}")
                    obstacle_lock_color   = None
                    obstacle_clear_streak = 0
                    avoid_phase           = None
                    fsm_state             = STATE_CRUISE
                    state_start_time      = now
                    settle_until_ts       = time.time() + SETTLE_DURATION
                    dbg("FSM AVOID -> CRUISE (obstacle cleared)")
                    set_run_state("cruise")


        # ==== SERVO OUTPUT (direct angle with optional settle) ====
        # After blue-backward, briefly bias steering toward the box direction
        t_now = time.time()
        if (not in_blue_backward 
            and back_follow_angle is not None 
            and t_now < post_back_follow_until):
            target_angle = back_follow_angle

        # After an emergency reverse, apply a short correction before returning
        # fully to normal cruise. This helps the robot move away from the wall
        # instead of continuing into it with only small yaw correction.
        if (fsm_state == STATE_CRUISE and
            t_now < front_emergency_recover_until and
            chosen_color is None and
            not line_trigger_raw):
            if front_emergency_recover_angle == CENTER_ANGLE:
                target_angle = imu_center_servo(
                    current_yaw,
                    YAW_DEADBAND_DEG_BASE,
                    YAW_KP_STRONG,
                    SERVO_CORR_LIMIT_STRONG
                )
            else:
                target_angle = front_emergency_recover_angle
            set_motor_speed(MOTOR_FWD, MOTOR_REV, FRONT_EMERGENCY_RECOVER_SPEED)

        if (time.time() < settle_until_ts and
            fsm_state not in (STATE_TURN, STATE_AVOID)):
            desired = CENTER_ANGLE
        else:
            desired = target_angle

        current_servo_angle = int(max(50, min(130, desired)))
        set_servo_angle(SERVO_CHANNEL, current_servo_angle)

        # Optional: if you still want ESC to stop the program:
        if cv2.waitKey(1) in [27, ord('q')]:
            dbg("ESC pressed, exiting main loop")
            break

    time.sleep(0.1)

finally:
    dbg("Shutting down: stopping camera and motors")
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    front_ultra.close()
    left_ultra.close()
    right_ultra.close()
    red_led.off()
    green_led.off()
