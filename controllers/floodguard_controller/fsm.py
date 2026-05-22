from constants import *

class FSM:
    def __init__(self):
        self.state = STATE_STARTUP

    def set_state(self, new_state):
        print(f"STATE CHANGE: {self.state} -> {new_state}")
        self.state = new_state

    def get_state(self):
        return self.state
