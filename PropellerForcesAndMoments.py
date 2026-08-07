from codrone_edu.drone import *
import time


# Connect to the drone
drone = Drone()
drone.pair()
time.sleep(1.0)

input("Press Enter for experiment to begin...")

t = time.time()

drone.set_motor_speed(500.0, 50.0, 50.0, 500.0, 3.0)
time.sleep(0.5)

drone.set_motor_speed(50.0, 500.0, 500.0, 50.0, 3.0)
time.sleep(0.5)

drone.set_motor_speed(50.0, 700.0, 50.0, 700.0, 3.0)
time.sleep(0.5)

drone.set_motor_speed(700.0, 50.0, 700.0, 50.0, 3.0)
time.sleep(0.5)

drone.set_motor_speed(0.0, 0.0, 0.0, 0.0)
drone.move()

drone.close()
