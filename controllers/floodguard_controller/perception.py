class Perception:
    def __init__(self, sensors):
        self.sensors = sensors

    def get_front_clearance(self):
        """Returns the filtered distance reading from the front 4 sonar array."""
        return self.sensors.get_front_distance()

    def get_side_clearance(self, direction):
        """Returns the flank clearance for a given direction."""
        if direction == "RIGHT":
            return self.sensors.get_right_distance()
        return self.sensors.get_left_distance()

    def is_side_blocked(self, direction):
        """Checks if the object is still next to the vehicle frame during flanking."""
        side_val = self.get_side_clearance(direction)
        return side_val < 600.0

    def get_best_steering_direction(self):
        """Compares left side versus right side clearance to turn away from the obstacle."""
        left_side = self.get_side_clearance("LEFT")
        right_side = self.get_side_clearance("RIGHT")
        return "RIGHT" if left_side < right_side else "LEFT"
