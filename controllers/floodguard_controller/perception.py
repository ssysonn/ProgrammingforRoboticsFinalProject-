from constants import (
    HIT_CONTACT_THRESHOLD,
    CONTACT_DEBOUNCE_FRAMES
)

class Perception:

    def __init__(self, sensors):

        self.sensors = sensors

        # debounce counters
        self.front_counter = 0
        self.side_counters = {
            "LEFT": 0,
            "RIGHT": 0
        }

        # memory smoothing
        self.left_history = []
        self.right_history = []

        self.MAX_HISTORY = 6

    # =====================================================
    # FRONT CLEARANCE
    # =====================================================

    def get_front_clearance(self):

        return self.sensors.get_front_distance()

    # =====================================================
    # SIDE CLEARANCE
    # =====================================================

    def get_side_clearance(self, direction):

        if direction == "RIGHT":
            return self.sensors.get_right_distance()

        return self.sensors.get_left_distance()

    # =====================================================
    # FRONT CONTACT
    # =====================================================

    def is_front_contact(self):

        front_distance = self.get_front_clearance()

        if front_distance < HIT_CONTACT_THRESHOLD:

            self.front_counter += 1

        else:

            self.front_counter = 0

        return (
            self.front_counter >=
            CONTACT_DEBOUNCE_FRAMES
        )

    # =====================================================
    # SIDE CONTACT
    # =====================================================

    def is_side_contact(self, direction):

        side_distance = self.get_side_clearance(direction)

        if side_distance < HIT_CONTACT_THRESHOLD:

            self.side_counters[direction] += 1

        else:

            self.side_counters[direction] = 0

        return (
            self.side_counters[direction] >=
            CONTACT_DEBOUNCE_FRAMES
        )

    # =====================================================
    # SIDE BLOCKED
    # =====================================================

    def is_side_blocked(self, direction):

        side_val = self.get_side_clearance(direction)

        return side_val < 600.0

    # =====================================================
    # HISTORY SMOOTHING
    # =====================================================

    def update_histories(self):

        left = self.get_side_clearance("LEFT")
        right = self.get_side_clearance("RIGHT")

        self.left_history.append(left)
        self.right_history.append(right)

        if len(self.left_history) > self.MAX_HISTORY:
            self.left_history.pop(0)

        if len(self.right_history) > self.MAX_HISTORY:
            self.right_history.pop(0)

    # =====================================================
    # AVERAGE SIDE VALUES
    # =====================================================

    def get_average_left(self):

        self.update_histories()

        return (
            sum(self.left_history) /
            len(self.left_history)
        )

    def get_average_right(self):

        self.update_histories()

        return (
            sum(self.right_history) /
            len(self.right_history)
        )

    # =====================================================
    # SMART ESCAPE LOGIC
    # =====================================================

    def get_best_steering_direction(self):

        left_avg = self.get_average_left()
        right_avg = self.get_average_right()

        front = self.get_front_clearance()

        # ================================================
        # EXTREME BLOCK CHECK
        # ================================================

        # if left side is basically blocked
        # always force right

        if left_avg < 120 and right_avg > 220:

            return "RIGHT"

        # if right side is basically blocked
        # always force left

        if right_avg < 120 and left_avg > 220:

            return "LEFT"

        # ================================================
        # CORNER DETECTION
        # ================================================

        # if both sides are blocked,
        # choose the LESS blocked side

        if left_avg < 200 and right_avg < 200:

            if right_avg > left_avg:
                return "RIGHT"
            else:
                return "LEFT"

        # ================================================
        # OPEN SPACE PREFERENCE
        # ================================================

        # strongly prefer very open paths

        if right_avg > left_avg * 1.25:

            return "RIGHT"

        if left_avg > right_avg * 1.25:

            return "LEFT"

        # ================================================
        # FRONT JAM RECOVERY
        # ================================================

        # if front is VERY close,
        # prefer widest side aggressively

        if front < 120:

            if right_avg > left_avg:
                return "RIGHT"
            else:
                return "LEFT"

        # ================================================
        # DEFAULT BEHAVIOR
        # ================================================

        # standard comparison

        if right_avg > left_avg:

            return "RIGHT"

        return "LEFT"