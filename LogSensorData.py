from PIRController import *
from SensorData import *
from DataLogger import *
import keyboardInput

keyboardInput.listen_for_keys()



# Initialize log name
log_file_name = "Assignment3.txt"
# Specify update rate
update_dt = 1.0 / 15.0 # 15 Hz
# Specify the end time of the data collect
t_end = 30
# Specify if you want to log data (will slow down the update loop)
LOG_DATA = True

# Connect to the drone
drone = Drone()
drone.pair()

# Get the current time
t_prev = time.time()
t_end = t_prev + t_end

# Initialize the sensor data class
sensors = SensorData(drone)

# Open and initialize the log file
header = ["time", "accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ", "phi", "theta", "psi", "velocityX",
         "velocityY", "pressure", "temperature", "baroAlt", "rangeFwd", "rangeDown"]
sensor_log = DataLogger(log_file_name, header, LOG_DATA)

first_time = 1
# Write the header for the log file
while True:

    t = time.time()

    if t > t_end:
        break

    if keyboardInput.stop_flag:
        print("Loop terminated by user.")
        break

    if t - t_prev >= update_dt:
        # Get updated sensor data from drone
        sensors.update_motion_data()
        sensors.update_alt_data()
        sensors.update_velocity_data()
        sensors.update_range_data()

        if first_time:
            t_prev = sensors.time
            first_time = False
        print("ax: ", sensors.accelX, " ay: ", sensors.accelY, " az: ", sensors.accelZ)
        print("gx: ", sensors.p, " gy: ", sensors.q, " gz: ", sensors.r)

        # Write the output
        data = [sensors.time, sensors.accelX, sensors.accelY, sensors.accelZ, sensors.p, sensors.q, sensors.r,
                sensors.phi, sensors.theta, sensors.psi, sensors.velocity_x, sensors.velocity_y, sensors.pressure,
                sensors.temperature, sensors.baroAltitude, sensors.range_fwd, sensors.range_btm]
        sensor_log.write_data(data)
        t_prev = sensors.time

sensor_log.close()
