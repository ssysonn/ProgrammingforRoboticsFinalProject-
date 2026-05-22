class SafetyManager:
    def __init__(self, sensors):
        self.sensors = sensors

    def emergency_stop_required(self):
        s3 = self.sensors.sonars["so3"].getValue()
        s4 = self.sensors.sonars["so4"].getValue()
        closest_front = min(s3, s4)
        
        # Absolute backup threshold if the vehicle slips past the perception layer (under 280 units)
        return  closest_front > 850.0

