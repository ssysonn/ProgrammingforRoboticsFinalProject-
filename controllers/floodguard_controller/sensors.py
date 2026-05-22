from controller import Camera, DistanceSensor, InertialUnit, GPS
from constants import TIME_STEP, WATER_BLUE_THRESHOLD

class SensorManager:
    def __init__(self, robot):
        self.robot = robot

        # Initialize Hardware
        self.camera = robot.getDevice("camera")
        self.camera.enable(TIME_STEP)

        self.gps = robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

        self.imu = robot.getDevice("inertial unit")
        self.imu.enable(TIME_STEP)

        # Initialize 16 Pioneer Sonar Sensors
        self.sonars = {}
        for i in range(16):
            sonar_name = f"so{i}"
            sensor = self.robot.getDevice(sonar_name)
            sensor.enable(TIME_STEP)
            self.sonars[sonar_name] = sensor

    def get_front_distance(self):
        values = [self.sonars["so2"].getValue(), self.sonars["so3"].getValue(),
                  self.sonars["so4"].getValue(), self.sonars["so5"].getValue()]
        valid = [v for v in values if v > 0.01]
        if valid:
            return min(valid)
        if any(v == 0.0 for v in values):
            return 0.01
        return 1000.0

    def get_left_distance(self):
        s0 = self.sonars["so0"].getValue()
        s1 = self.sonars["so1"].getValue()
        values = [s0, s1]
        valid = [v for v in values if v > 0.01]
        if valid:
            return min(valid)
        if any(v == 0.0 for v in values):
            return 0.01
        return 1000.0

    def get_right_distance(self):
        s6 = self.sonars["so6"].getValue()
        s7 = self.sonars["so7"].getValue()
        values = [s6, s7]
        valid = [v for v in values if v > 0.01]
        if valid:
            return min(valid)
        if any(v == 0.0 for v in values):
            return 0.01
        return 1000.0

    def get_position(self):
        return self.gps.getValues()

    def get_tilt(self):
        rpy = self.imu.getRollPitchYaw()
        return abs(rpy[0]) + abs(rpy[1])

    def detect_water(self):
        image = self.camera.getImage()
        if image is None: 
            return False

        width = self.camera.getWidth()
        height = self.camera.getHeight()
        blue_count = 0
        start_y = int(height * 0.6)
        total_checked = 0

        for x in range(0, width, 2):
            for y in range(start_y, height, 2):
                total_checked += 1
                b = self.camera.imageGetBlue(image, width, x, y)
                r = self.camera.imageGetRed(image, width, x, y)
                g = self.camera.imageGetGreen(image, width, x, y)

                if b > WATER_BLUE_THRESHOLD and b > r and b > g:
                    blue_count += 1

        if total_checked == 0: 
            return False
        return (blue_count / total_checked) > 0.05
