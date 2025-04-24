"""
NOT IMPLEMENTED YET!
"""

class Actuator:
    def __init__(self, name: str):
        self.name = name

    def apply(self, speed: float, theta: float):
        pass

    def apply_with_pass_filter(self, speed: float, theta: float):
        """
        Apply the speed and theta to the actuator with a pass filter.
        """
        # get the current speed and theta
        # if the speed and theta are within the pass filter range,
        # apply the speed and theta to the actuator
        self.apply()