from controller import Robot
import sys
import math

from constants import *
from sensors import SensorManager
from navigation import Navigation
from perception import Perception
from mapping import HazardMap
from safety import SafetyManager


def instant_print(text):
    print(text)
    sys.stdout.flush()


# =========================================================
# INITIALISE ROBOT
# =========================================================

robot = Robot()

fl_motor = robot.getDevice("front left wheel")
bl_motor = robot.getDevice("back left wheel")
fr_motor = robot.getDevice("front right wheel")
br_motor = robot.getDevice("back right wheel")

motors = [fl_motor, fr_motor, bl_motor, br_motor]

for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)


# =========================================================
# SYSTEMS
# =========================================================

sensors = SensorManager(robot)
navigation = Navigation(fl_motor, fr_motor, bl_motor, br_motor)
perception = Perception(sensors)
hazard_map = HazardMap()

instant_print("[SYSTEM] Stable Navigation Controller Online")


# =========================================================
# STATE
# =========================================================

robot_mode = "PATROL_TO_GOAL"
escape_direction = "LEFT"

last_obstacle_time = 0
OBSTACLE_COOLDOWN = 2.5   # seconds

turn_lock_frames = 0
FORWARD_RECOVERY_FRAMES = 20

last_turn_direction = None


# =========================================================
# LOOP
# =========================================================

while robot.step(TIME_STEP) != -1:

    t = robot.getTime()
    gps = sensors.get_position()
    x, y, z = gps

    front = perception.get_front_clearance()

    # =====================================================
    # COOLDOWN LOCK (prevents re-trigger spam)
    # =====================================================

    if t - last_obstacle_time < OBSTACLE_COOLDOWN:
        # ignore new obstacles briefly
        front_threshold = 0
    else:
        front_threshold = 250.0  # normal trigger


    # =====================================================
    # PATROL MODE
    # =====================================================

    if robot_mode == "PATROL_TO_GOAL":

        # ----------------------------
        # OBSTACLE DETECTED
        # ----------------------------
        if front > 0 and front < front_threshold:

            instant_print(f"[OBSTACLE] detected ({front:.2f})")

            navigation.stop()

            escape_direction = perception.get_best_steering_direction()

            last_turn_direction = escape_direction
            last_obstacle_time = t
            turn_lock_frames = 0

            robot_mode = "PIVOT_AWAY"


        # ----------------------------
        # NORMAL DRIVE
        # ----------------------------
        else:

            rpy = sensors.imu.getRollPitchYaw()
            yaw = rpy[2]

            target = math.atan2(TARGET_GOAL_Z - z, TARGET_GOAL_X - x)

            error = math.atan2(math.sin(target - yaw),
                               math.cos(target - yaw))

            correction = error * 2.0

            navigation.set_speeds(
                MAX_SPEED - correction,
                MAX_SPEED + correction
            )


    # =====================================================
    # PIVOT MODE (LIMITED TURNING → NO CIRCLING)
    # =====================================================

    elif robot_mode == "PIVOT_AWAY":

        turn_lock_frames += 1

        # fixed turn duration (IMPORTANT FIX)
        if turn_lock_frames < 12:

            if escape_direction == "RIGHT":
                navigation.turn_right()
            else:
                navigation.turn_left()

        else:
            navigation.stop()
            robot_mode = "FORWARD_RECOVERY"
            turn_lock_frames = 0


    # =====================================================
    # RECOVERY MODE (FORCES STRAIGHT EXIT FROM TURN)
    # =====================================================

    elif robot_mode == "FORWARD_RECOVERY":

        navigation.move_forward()

        turn_lock_frames += 1

        # after moving forward a bit, re-enable patrol
        if turn_lock_frames > FORWARD_RECOVERY_FRAMES:

            robot_mode = "PATROL_TO_GOAL"
            turn_lock_frames = 0

            instant_print("[RECOVERY] Rejoining patrol path")


    # =====================================================
    # OPTIONAL SAFETY RESET (prevents stuck spin)
    # =====================================================

    if front > 800:
        # clear space → allow mode reset safety
        if robot_mode == "PIVOT_AWAY":
            robot_mode = "FORWARD_RECOVERY"