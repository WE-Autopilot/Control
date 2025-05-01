from abstract_model import AbstractModel
from perception import Lidar, VirtualLidar
from actuation import Actuator, VirtualActuator
from _test_model import TestModel
import time

def start(model: AbstractModel):
    print("Starting WEAP auto...")

    lidar = Lidar()
    actuator = Actuator()
    model.init()

    # TODO: guarantee loop time?

    start_time = time.time() * 1000 # millis
    while True:
        scan, err = lidar.get_scan()

        if err:
            print("Error getting scan:", err)
            continue
        
        current_time = int(time.time() * 1000 - start_time)
        # print(f"Current time: {current_time} ms")
        speed, theta = model.eval(scan, timestamp=current_time)
        # print(f"Speed: {speed}, Theta: {theta}")
        actuator.apply_with_pass_filter(speed, theta)

        if model.is_done():
            print("Model is done.")
            break

    model.shutdown()
    print("Stopping WEAP auto...")

if __name__ == "__main__":
    model = TestModel()
    start(model)
