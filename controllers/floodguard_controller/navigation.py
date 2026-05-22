from constants import MAX_SPEED

class Navigation:
    def __init__(self, fl_motor, fr_motor, bl_motor, br_motor):
        self.fl = fl_motor
        self.fr = fr_motor
        self.bl = bl_motor
        self.br = br_motor

    def set_speeds(self, left_speed, right_speed):
        # Clip speeds to keep them within maximum physics motor limits
        left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
        right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
        
        self.fl.setVelocity(left_speed)
        self.bl.setVelocity(left_speed)
        self.fr.setVelocity(right_speed)
        self.br.setVelocity(right_speed)

    def move_forward(self, speed=MAX_SPEED):
        # Moves the chassis forward based on your world's motor orientation
        self.set_speeds(-speed, -speed)

    def stop(self):
        self.set_speeds(0.0, 0.0)

    def reverse(self):

        # stronger reverse
        self.set_speeds(MAX_SPEED * 0.7, MAX_SPEED * 0.7)

    def turn_left(self):

        # stronger pivot
        self.set_speeds(MAX_SPEED * 0.8, -MAX_SPEED * 0.8)

    def turn_right(self):

        # stronger pivot
        self.set_speeds(-MAX_SPEED * 0.8, MAX_SPEED * 0.8)