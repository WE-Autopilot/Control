from abstract_model import AbstractModel
from perception import Lidar
from actuation import Actuator

def start(model: AbstractModel):
    print("Starting WEAP auto...")

    lidar = Lidar()
    actuator = Actuator()
    model.init()

    # TODO: guarantee loop time
    while True:
        scan, err = lidar.get_scan()

        if err:
            print("Error getting scan:", err)
            continue

        speed, theta = model.eval(scan)

        actuator.apply_with_pass_filter(speed, theta)

        if model.is_done():
            print("Model is done.")
            break

    model.shutdown()
    print("Stopping WEAP auto...")

if __name__ == "__main__":
    start()
