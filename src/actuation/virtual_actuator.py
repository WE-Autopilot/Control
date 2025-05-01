"""
NOT IMPLEMENTED YET!
"""

class VirtualActuator:
    def __init__(self, name: str="Virtual Actuator"):
        self.name = name
        self.speed = 0.0
        self.theta = 0.0
        self.pass_filter_speed = 0.1
        self.pass_filter_theta = 0.1

    def apply(self, speed: float, theta: float):
        """
        Not yet implemented.

        Args:
            speed (float): _description_
            theta (float): _description_
        """
        print(f"{self.name} | Applying speed: {speed}, theta: {theta}")
        self.speed = speed
        self.theta = theta

    def apply_with_pass_filter(self, speed: float, theta: float):
        """
        Apply the speed and theta to the actuator with a pass filter.
        """
        # get the current speed and theta
        # if the speed and theta are within the pass filter range,
        # apply the speed and theta to the actuator
        if abs(speed - self.speed) < self.pass_filter_speed and abs(theta - self.theta) < self.pass_filter_theta:
            self.apply(speed, theta)