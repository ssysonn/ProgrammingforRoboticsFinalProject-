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

navigation = Navigation(
    fl_motor,
    fr_motor,
    bl_motor,
    br_motor
)

perception = Perception(sensors)

hazard_map = HazardMap()

instant_print(
    "[SYSTEM] Stable Navigation Controller Online"
)


# =========================================================
# STATE
# =========================================================

robot_mode = "PATROL_TO_GOAL"

escape_direction = "LEFT"

last_obstacle_time = 0

OBSTACLE_COOLDOWN = 2.5

turn_lock_frames = 0

FORWARD_RECOVERY_FRAMES = 20

wall_follow_frames = 0

MAX_WALL_FOLLOW_FRAMES = 100

CONTACT_REVERSE_FRAMES = 12

CONTACT_TURN_FRAMES = 18

CONTACT_FORWARD_FRAMES = 18

PIVOT_TURN_FRAMES = 24

last_turn_direction = None


# =========================================================
# FAILSAFE MODE
# =========================================================

FAILSAFE_TRIGGER_TIME = 80.0

failsafe_mode = False
shoreline_printed = False
flood_entry_printed = False


# =========================================================
# WATER TRACKING
# =========================================================

water_tracking_state = "DRY_LAND"

water_detection_frames = 0

dry_detection_frames = 0

water_entry_point = None

water_entry_height = None

last_depth_print = 0.0
max_depth = 0.0
depth_print_count = 0

# =========================================================
# LOOP
# =========================================================

while robot.step(TIME_STEP) != -1:

    t = robot.getTime()

    gps = sensors.get_position()

    x, y, z = gps

    # =====================================================
    # WATER DETECTION
    # =====================================================

    camera_sees_blue = sensors.detect_water()

    if camera_sees_blue:

        water_detection_frames += 1

        dry_detection_frames = 0

    else:

        dry_detection_frames += 1

        water_detection_frames = 0

    # =====================================================
    # WATER STATE MACHINE
    # =====================================================

    if (
        water_tracking_state == "DRY_LAND"
        and water_detection_frames > 5
    ):

        water_tracking_state = "APPROACHING"

    if not shoreline_printed:

        instant_print(
            "[TRACKER] Shoreline detected."
        )

        shoreline_printed = True

    elif (
        water_tracking_state == "APPROACHING"
        and dry_detection_frames > 12
    ):

        water_tracking_state = "FULLY_SUBMERGED"

        water_entry_point = (x, z)

        water_entry_height = y
        lowest_y = y

    if not flood_entry_printed:

        instant_print(
            f"[TRACKER] Flood entry confirmed "
            f"X:{x:.2f} Z:{z:.2f}"
        )

        flood_entry_printed = True

    elif (
        water_tracking_state == "FULLY_SUBMERGED"
        and water_detection_frames > 5
    ):

        water_tracking_state = "DRY_LAND"


    # =====================================================
    # DEPTH ESTIMATION
    # =====================================================

    if (
        water_tracking_state == "FULLY_SUBMERGED"
        and water_entry_height is not None
    ):

        # calculate vertical difference from entry point
        DEPTH_SCALE = 3.5

        current_depth = (
            water_entry_height - y
        ) * DEPTH_SCALE

        # prevent negative depth values
        if current_depth < 0:
            current_depth = 0

        # track deepest point reached
        if 'max_depth' not in globals():
            max_depth = current_depth

        if current_depth > max_depth:
            max_depth = current_depth

        # smooth noisy readings
        depth_estimate = round(current_depth, 2)

        # distance travelled through flood
        dx = x - water_entry_point[0]
        dz = z - water_entry_point[1]

        distance_travelled = math.sqrt(
            dx**2 + dz**2
        )

    # print only 5 times
    if (
        t - last_depth_print > 5.0
        and depth_print_count < 5
    ):

        instant_print(
            f"Max Depth: {max_depth:.2f}m"
        )

        last_depth_print = t

        depth_print_count += 1

    # =====================================================
    # FAILSAFE ACTIVATION
    # =====================================================

    if (
        t > FAILSAFE_TRIGGER_TIME
        and not failsafe_mode
    ):

        failsafe_mode = True

        instant_print(
            "[SAFE] Driving straight. "
        )

    front = perception.get_front_clearance()

    front_contact = (
        perception.is_front_contact()
    )

    left_contact = (
        perception.is_side_contact("LEFT")
    )

    right_contact = (
        perception.is_side_contact("RIGHT")
    )

    # =====================================================
    # FAILSAFE DRIVE MODE
    # =====================================================

    if failsafe_mode:

        navigation.set_speeds(
            MAX_SPEED,
            MAX_SPEED
        )

        # =================================================
        # MISSION COMPLETE
        # =================================================

        if (
            water_tracking_state == "DRY_LAND"
            and z > TARGET_GOAL_Z
        ):

            navigation.stop()

            instant_print(
                "[MISSION] Flood crossing complete."
            )

            break

        continue

    # =====================================================
    # COOLDOWN LOCK
    # =====================================================

    if (
        t - last_obstacle_time
        < OBSTACLE_COOLDOWN
    ):

        front_threshold = 0

    else:

        front_threshold = (
            OBSTACLE_DETECTION_THRESHOLD
        )

    # =====================================================
    # PATROL MODE
    # =====================================================

    if robot_mode == "PATROL_TO_GOAL":

        # -------------------------------------------------
        # PHYSICAL CONTACT
        # -------------------------------------------------

        if (
            front_contact
            or left_contact
            or right_contact
        ):



            navigation.stop()

            if (
                front > 0
                and front < PIVOT_CLEARANCE_THRESHOLD
            ):

                reverse_steps = (
                    CONTACT_REVERSE_FRAMES * 2
                )

                turn_steps = (
                    CONTACT_TURN_FRAMES * 2
                )

                turn_bias = 0.45



            else:

                reverse_steps = (
                    CONTACT_REVERSE_FRAMES
                )

                turn_steps = (
                    CONTACT_TURN_FRAMES
                )

                turn_bias = 0.6

            # -------------------------------------------------
            # REVERSE
            # -------------------------------------------------

            navigation.reverse()

            for _ in range(reverse_steps):

                if robot.step(TIME_STEP) == -1:
                    sys.exit(0)

            navigation.stop()

            # -------------------------------------------------
            # CHOOSE DIRECTION
            # -------------------------------------------------

            if left_contact and not right_contact:

                escape_direction = "RIGHT"

            elif right_contact and not left_contact:

                escape_direction = "LEFT"

            else:

                escape_direction = (
                    perception.get_best_steering_direction()
                )



            # -------------------------------------------------
            # TURN
            # -------------------------------------------------

            if escape_direction == "RIGHT":

                navigation.turn_right_arc(
                    bias=turn_bias
                )

            else:

                navigation.turn_left_arc(
                    bias=turn_bias
                )

            for _ in range(turn_steps):

                if robot.step(TIME_STEP) == -1:
                    sys.exit(0)

            navigation.stop()

            # -------------------------------------------------
            # DRIVE FORWARD
            # -------------------------------------------------

            navigation.move_forward()

            for _ in range(
                CONTACT_FORWARD_FRAMES
            ):

                if robot.step(TIME_STEP) == -1:
                    sys.exit(0)

            navigation.stop()

            last_turn_direction = (
                escape_direction
            )

            last_obstacle_time = t

            turn_lock_frames = 0

            wall_follow_frames = 0

            robot_mode = "PATROL_TO_GOAL"

            continue

        # -------------------------------------------------
        # OBSTACLE DETECTED
        # -------------------------------------------------

        left_clearance = (
            perception.get_side_clearance("LEFT")
        )

        right_clearance = (
            perception.get_side_clearance("RIGHT")
        )

        if (
            (
                front > 0
                and front < front_threshold
            )
            or left_clearance
            < SIDE_DETECTION_THRESHOLD
            or right_clearance
            < SIDE_DETECTION_THRESHOLD
        ):



            navigation.stop()

            if right_clearance > left_clearance:

                escape_direction = "RIGHT"

            else:

                escape_direction = "LEFT"



            last_turn_direction = (
                escape_direction
            )

            last_obstacle_time = t

            turn_lock_frames = 0

            wall_follow_frames = 0

            # -------------------------------------------------
            # STOP
            # -------------------------------------------------

            navigation.stop()

            for _ in range(4):

                if robot.step(TIME_STEP) == -1:
                    sys.exit(0)

            # -------------------------------------------------
            # REVERSE
            # -------------------------------------------------

            if (
                front > 0
                and front
                < PIVOT_CLEARANCE_THRESHOLD
            ):

                pre_reverse_steps = 20

            else:

                pre_reverse_steps = 10

            navigation.reverse()

            for _ in range(pre_reverse_steps):

                if robot.step(TIME_STEP) == -1:
                    sys.exit(0)

            navigation.stop()

            # -------------------------------------------------
            # PAUSE
            # -------------------------------------------------

            for _ in range(3):

                if robot.step(TIME_STEP) == -1:
                    sys.exit(0)

            robot_mode = "PIVOT_AWAY"

        # -------------------------------------------------
        # NORMAL DRIVE
        # -------------------------------------------------

        else:

            rpy = (
                sensors.imu.getRollPitchYaw()
            )

            yaw = rpy[2]

            target = math.atan2(
                TARGET_GOAL_Z - z,
                TARGET_GOAL_X - x
            )

            error = math.atan2(
                math.sin(target - yaw),
                math.cos(target - yaw)
            )

            correction = max(
                min(error * 2.0, 1.5),
                -1.5
            )

            navigation.set_speeds(
                MAX_SPEED - correction,
                MAX_SPEED + correction
            )

    # =====================================================
    # PIVOT MODE
    # =====================================================

    elif robot_mode == "PIVOT_AWAY":

        turn_lock_frames += 1

        if turn_lock_frames < PIVOT_TURN_FRAMES:



            if escape_direction == "RIGHT":

                navigation.turn_right_arc()

            else:

                navigation.turn_left_arc()

        else:

            navigation.stop()

            robot_mode = "FORWARD_RECOVERY"

            turn_lock_frames = 0

    # =====================================================
    # RECOVERY MODE
    # =====================================================

    elif robot_mode == "FORWARD_RECOVERY":

        navigation.move_forward()

        turn_lock_frames += 1

        if (
            turn_lock_frames
            > FORWARD_RECOVERY_FRAMES
        ):

            if (
                perception.get_front_clearance()
                < 300.0
                or perception.get_side_clearance(
                    escape_direction
                ) < 250.0
            ):

                robot_mode = "WALL_FOLLOW"

            else:

                robot_mode = "PATROL_TO_GOAL"



            turn_lock_frames = 0

    # =====================================================
    # WALL FOLLOW MODE
    # =====================================================

    elif robot_mode == "WALL_FOLLOW":

        side_clearance = (
            perception.get_side_clearance(
                escape_direction
            )
        )

        rpy = (
            sensors.imu.getRollPitchYaw()
        )

        yaw = rpy[2]

        target = math.atan2(
            TARGET_GOAL_Z - z,
            TARGET_GOAL_X - x
        )

        error = math.atan2(
            math.sin(target - yaw),
            math.cos(target - yaw)
        )

        wall_error = (
            TARGET_WALL_DISTANCE
            - side_clearance
        )

        correction = max(
            min(
                wall_error
                * WALL_FOLLOW_GAIN,
                1.0
            ),
            -1.0
        )

        if side_clearance < 0.4:

            base_speed = 1.2

        elif side_clearance < 0.8:

            base_speed = 1.8

        else:

            base_speed = 2.3

        heading_adjust = error * 0.15

        if escape_direction == "RIGHT":

            left_speed = (
                base_speed
                - correction
                - heading_adjust
            )

            right_speed = (
                base_speed
                + correction
                + heading_adjust
            )

        else:

            left_speed = (
                base_speed
                + correction
                - heading_adjust
            )

            right_speed = (
                base_speed
                - correction
                + heading_adjust
            )

        navigation.set_speeds(
            left_speed,
            right_speed
        )

        wall_follow_frames += 1

        # -------------------------------------------------
        # EXIT WALL FOLLOW
        # -------------------------------------------------

        if (
            perception.get_front_clearance()
            > 400.0
            and abs(error) < 0.8
        ):

            navigation.stop()

            robot_mode = "PATROL_TO_GOAL"

            wall_follow_frames = 0



        # -------------------------------------------------
        # STUCK TOO LONG
        # -------------------------------------------------

        if (
            wall_follow_frames
            > MAX_WALL_FOLLOW_FRAMES
        ):



            escape_direction = (
                "LEFT"
                if escape_direction == "RIGHT"
                else "RIGHT"
            )

            robot_mode = "PIVOT_AWAY"

            wall_follow_frames = 0

    # =====================================================
    # OPTIONAL SAFETY RESET
    # =====================================================

    if front > 800:

        if robot_mode == "PIVOT_AWAY":

            robot_mode = "FORWARD_RECOVERY"