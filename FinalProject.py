from PIRController import *
from SensorData import *
from DataLogger import *
import math as m
import numpy as np
import keyboardInput
keyboardInput.listen_for_keys()
def eulerToRotationMatrix(phi, theta, psi):
    R_roll = np.array([[1.0, 0.0, 0.0],
                       [0.0, m.cos(phi), m.sin(phi)],
                       [0.0, -m.sin(phi), m.cos(phi)]])

    R_pitch = np.array([[m.cos(theta), 0.0, -m.sin(theta)],
                        [0.0, 1.0, 0.0],
                        [m.sin(theta), 0.0, m.cos(theta)]])

    R_yaw = np.array([[m.cos(psi), m.sin(psi), 0.0],
                      [-m.sin(psi), m.cos(psi), 0.0],
                      [0.0, 0.0, 1.0]])

    R = np.dot(R_roll, np.dot(R_pitch, R_yaw))

    return R

# Initialize log name
log_file_name = "Part2.txt"
# Specify update rate
update_dt = 1.0 / 15.0  # 15 Hz
# Specify the end time of the data collect
t_end = 200
# Specify if you want to log data (will slow down the update loop)
LOG_DATA = True
# degrees to radians conversions
d2r = m.pi / 180.0
# gravity m/s^2
gravity = 9.8
# Run controller flag # If this is set to False, algorithms will run but the vehicle will not fly
RUN_CONTROL = False

# Navigation aiding source flags
USE_AHRS = True          # Turns on/off attitude aiding using AHRS equations
USE_BARO_ALT = True     # Turns on/off altitude aiding using barometric altitude measurements
USE_FLOW_VEL = True     # Turns on/off velocity aiding using optical flow measurements

# Initialization variables for landing logic
WALL_FOUND = False
landing_count = 0

# Connect to the drone
drone = Drone()
drone.pair()

# Initialize the sensor data class
sensors = SensorData(drone)

# Get the current time
t_prev = sensors.time
t_end = t_prev + t_end

# Initialize Kalman Filter Parameters
# ======================================================================================================================
# ======================================================================================================================
# Gyro uncertainty
sigma_gyro = 1/3*d2r # radians/s
# Accel uncertainty
sigma_accel = 1/30*d2r # m/s^2

# Initial Covariances - 3 attitude, 3 position, 3 velocity
sigma_att0 = 2*d2r # radians
sigma_pos0 = 1 # m
sigma_vel0 = 1 # m/s
P = np.array([[sigma_att0 ** 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, sigma_att0 ** 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, sigma_att0 ** 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, sigma_pos0 ** 2, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, sigma_pos0 ** 2, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, sigma_pos0 ** 2, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigma_vel0 ** 2, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigma_vel0 ** 2, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigma_vel0 ** 2]])

# Initialize the process noise
Q = np.array([[sigma_gyro ** 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, sigma_gyro ** 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, sigma_gyro ** 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigma_accel ** 2, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigma_accel ** 2, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sigma_accel ** 2]])

# Initialize the measurement noise
# Accel measurements for AHRS
R_att = np.array([[sigma_accel ** 2, 0.0, 0.0], [0.0, sigma_accel ** 2, 0.0], [0.0, 0.0, sigma_accel ** 2]])*1e4
# Barometric altitude measurements
R_alt = ((0.5/3) ** 2)
# Optical flow velocity measurements
R_vel = np.array([[0.5 ** 2, 0.0], [0.0, 0.5 ** 2]])

# Initialize the kalman filter
# [ roll, pitch, yaw, pIX, pIY, pIZ, vIX, vIY, vIZ]
x_hat = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [-1.0], [0.0], [0.0], [0.0]])

# Initialize the Autopilot Parameters
# ======================================================================================================================
# ======================================================================================================================
# Initialize the altitude controller
kp_alt = 75.0  # .1/15
ki_alt = 0.1
kd_alt = 0.0  # 0.001
pir_alt = PIRController(kp_alt, ki_alt, kd_alt, -75, 75, t_prev)

# Initialize the inertial X position controller - pitch to X position
kp_pxI = 20
ki_pxI = 4
kd_pxI = 10
pir_pxI = PIRController(kp_pxI, ki_pxI, kd_pxI, -25, 25, t_prev)

# Initialize the inertial Y position controller - roll to Y position
kp_pyI = 20
ki_pyI = 4
kd_pyI = 10
pir_pyI = PIRController(kp_pyI, ki_pyI, kd_pyI, -25, 25, t_prev)

# Set initial commands
altitude_cmd = 2.0 # m
vx_cmd = 0.0
vy_cmd = 0.0

# Set some initial measurement variables to 0 that will be logged
vx_meas = 0.0
vy_meas = 0.0
chiSq = np.zeros((1,1))

# Open and initialize the log file
# ======================================================================================================================
# ======================================================================================================================
header = ["time", "dt", "accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ", "phi", "theta", "psi", "phi_est",
          "theta_est", "psi_est", "pIX_est", "pIY_est", "pIZ_est", "vIX_est", "vIY_est", "vIZ_est",
          "pIX_cmd", "pIY_cmd", "altitude_cmd", "vx_meas", "vy_meas",  "pitch_power", "roll_power", "v_chiSq",
          "phi_sigma", "theta_sigma", "psi_sigma", "pIX_sigma", "pIY_sigma", "pIZ_sigma", "vIX_sigma", "vIY_sigma",
          "vIZ_sigma","alt_meas"]

sensor_log = DataLogger(log_file_name, header, LOG_DATA)

input("Press Enter to take off...")

# Update sensor data to get altitude
# Repeat until a valid baro altitude is received
sensors.update_alt_data()
while sensors.baroAltitude == 0:
    sensors.update_alt_data()

# Save off the launch barometric altitude
altitude0 = sensors.baroAltitude
print("Takeoff altitude (asl): ", altitude0)

# Takeoff
if RUN_CONTROL:
    drone.takeoff()
    drone.set_throttle(0.0)

first_time = 1
while True:
    # Update sensor data
    sensors.update_motion_data()
    sensors.update_velocity_data()
    sensors.update_alt_data()
    sensors.update_position_data()
    sensors.update_range_data()

    # Update time
    t = sensors.time

    if first_time:
        t_prev = t
        first_time = False

    if t > t_end:
        break

    if keyboardInput.stop_flag:
        print("Loop terminated by user.")
        break

    if t - t_prev > update_dt:

# RUN KALMAN FILTER
# ======================================================================================================================
# ======================================================================================================================
        # Compute dt
        dt = t - t_prev
        # Update previous time
        t_prev = t

        # Pull off measurements
        # Gyro (rad/s)
        p = sensors.p * d2r
        q = sensors.q * d2r
        r = sensors.r * d2r
        # Accelerometer (m/s/s)
        ax = sensors.accelX
        ay = sensors.accelY
        az = sensors.accelZ
        # Optical flow (m/s)
        vx_meas = sensors.velocity_x/10.0
        vy_meas = sensors.velocity_y/10.0
        # Barometric pressure (m)
        alt_meas = sensors.baroAltitude-altitude0

        # Propagate states
        N = 10  # Number of sub-steps for propagation each sample period
        for i in range(1, N):
            # Update phi, theta, and psi
            phi = x_hat[0]
            theta = x_hat[1]
            psi = x_hat[2]
            pIX = x_hat[3]
            pIY = x_hat[4]
            pIZ = x_hat[5]
            vIX = x_hat[6]
            vIY = x_hat[7]
            vIZ = x_hat[8]

            # Compute conversion from body rates to euler rates
            bodyRate2EulerRate = [[1, m.sin(phi)*m.tan(theta), m.cos(phi)*m.tan(theta)],
                     [0, m.cos(phi), -m.sin(phi)],
                     [0, m.sin(phi)/m.cos(theta), m.cos(phi)/m.cos(theta)]]

            # Compute the "Inertial" to body frame DCM
            R_i2b = eulerToRotationMatrix(phi, theta, psi)

            # Compute the state propagation f
            f_att = np.dot(bodyRate2EulerRate,np.array([[p], [q], [r]]))
            f_pos = np.array([vIX, vIY, vIZ])
            f_vel = np.dot(np.transpose(R_i2b), np.array([[ax], [ay], [az]])) + np.array([[0.0], [0.0], [9.8]])
            f = np.concatenate((f_att, f_pos, f_vel))

            # Compute the Jacobian of f
            A = np.array([[m.tan(theta)*(q*m.cos(phi)-r*m.sin(phi)), (r*m.cos(phi)+q*m.sin(phi))*(1+m.tan(theta)*m.tan(theta)), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [-r*m.cos(phi)-q*m.sin(phi), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [(q*m.cos(phi)-r*m.sin(phi))/m.cos(theta), m.sin(theta)*(r*m.cos(phi)+q*m.sin(phi))/(m.cos(theta)*m.cos(theta)), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                              [ay*(m.sin(phi)*m.sin(psi) + m.cos(phi)*m.cos(psi)*m.sin(theta)) + az*(m.cos(phi)*m.sin(psi) - m.cos(psi)*m.sin(phi)*m.sin(theta)), m.cos(psi)*(az*m.cos(phi)*m.cos(theta) - ax*m.sin(theta) + ay*m.cos(theta)*m.sin(phi)), az*(m.cos(psi)*m.sin(phi) - m.cos(phi)*m.sin(psi)*m.sin(theta)) - ay*(m.cos(phi)*m.cos(psi) + m.sin(phi)*m.sin(psi)*m.sin(theta)) - ax*m.cos(theta)*m.sin(psi), 0, 0, 0, 0, 0, 0],
                              [-ay*(m.cos(psi)*m.sin(phi) - m.cos(phi)*m.sin(psi)*m.sin(theta)) - az*(m.cos(phi)*m.cos(psi) + m.sin(phi)*m.sin(psi)*m.sin(theta)), m.sin(psi)*(az*m.cos(phi)*m.cos(theta) - ax*m.sin(theta) + ay*m.cos(theta)*m.sin(phi)), az*(m.sin(phi)*m.sin(psi) + m.cos(phi)*m.cos(psi)*m.sin(theta)) - ay*(m.cos(phi)*m.sin(psi) - m.cos(psi)*m.sin(phi)*m.sin(theta)) + ax*m.cos(psi)*m.cos(theta), 0, 0, 0, 0, 0, 0],
                              [m.cos(theta)*(ay*m.cos(phi) - az*m.sin(phi)), - ax*m.cos(theta) - az*m.cos(phi)*m.sin(theta) - ay*m.sin(phi)*m.sin(theta), 0, 0, 0, 0, 0, 0, 0]])

            # Propagate the state
            x_hat = x_hat + (dt / N) * f

            # Propagate the covariance
            P = P + (dt / N) * (np.dot(A, P) + np.dot(P, A.transpose()) + Q)
            P = np.real(0.5 * P + 0.5 * P.transpose())

        if (USE_AHRS):
            # AHRS Measurement Update
            # ----------------------------------------------------------------------------------------
            # Update states after propagation
            phi = x_hat[0]
            theta = x_hat[1]
            psi = x_hat[2]
            pIX = x_hat[3]
            pIY = x_hat[4]
            pIZ = x_hat[5]
            vIX = x_hat[6]
            vIY = x_hat[7]
            vIZ = x_hat[8]

            # Calculate Va
            Va = m.sqrt(vIX*vIX + vIY*vIY + vIZ*vIZ)

            # Form the vector to hold the measurement
            y_att = np.array([[ax], [ay], [az]])

            # Form the measurement model
            h_att = ((np.cross(np.array([p, q, r]),
                               np.array([Va * m.cos(theta), 0.0, Va * m.sin(theta)])))
                     - gravity * np.array([-m.sin(theta), m.cos(theta) * m.sin(phi), m.cos(theta) * m.cos(phi)])).reshape(-1,1)

            # Take the jacobian of the measurement model
            C_att = np.array([[0.0, q*Va*m.cos(theta)+gravity * m.cos(theta), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [-gravity * m.cos(theta) * m.cos(phi), -r*Va*m.sin(theta)-p*Va*m.cos(theta) + gravity * m.sin(theta) * m.sin(phi), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [gravity * m.cos(theta) * m.sin(phi), q*Va*m.sin(theta) + gravity * m.sin(theta) * m.cos(phi), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

            # Calculate the Kalman gain
            L_att = np.dot(P, np.dot(C_att.transpose(),
                                         np.linalg.inv(np.dot(C_att, np.dot(P, C_att.transpose())) + R_att)))
            # Update the covariance matrix
            I9 = np.identity(9)
            P = np.dot((I9 - np.dot(L_att, C_att)), P)
            P = np.real(0.5 * P + 0.5 * P.transpose())

            # Update the state
            x_hat = x_hat + np.dot(L_att, (y_att-h_att))

        # Barometric pressure altitude update
        # ----------------------------------------------------------------------------------------
        if (USE_BARO_ALT):
            phi = x_hat[0]
            theta = x_hat[1]
            psi = x_hat[2]
            pIX = x_hat[3]
            pIY = x_hat[4]
            pIZ = x_hat[5]
            vIX = x_hat[6]
            vIY = x_hat[7]
            vIZ = x_hat[8]

            # Form the vector to hold the measurement
            y_alt = alt_meas

            # Form the measurement model
            h_alt = pIZ

            # Take the jacobian of the measurement model
            C_alt = np.array([[0, 0, 0, 0, 0, 1, 0, 0, 0]]) # YOU REPLACE THIS

            # Calculate the Kalman gain
            L_alt = np.dot(P, np.dot(C_alt.transpose(),
                                     np.linalg.inv(np.dot(C_alt, np.dot(P, C_alt.transpose())) + R_alt)))
            # Update the covariance matrix
            I9 = np.identity(9)
            P = np.dot((I9 - np.dot(L_alt, C_alt)), P)
            P = np.real(0.5 * P + 0.5 * P.transpose())
            # Update the state
            x_hat = x_hat + L_alt*(y_alt - h_alt)

        # Velocity update
        # ----------------------------------------------------------------------------------------
        if (USE_FLOW_VEL):
            phi = x_hat[0,0]
            theta = x_hat[1,0]
            psi = x_hat[2,0]
            pIX = x_hat[3,0]
            pIY = x_hat[4,0]
            pIZ = x_hat[5,0]
            vIX = x_hat[6,0]
            vIY = x_hat[7,0]
            vIZ = x_hat[8,0]

            # Form the vector to hold the measurement
            y_vel = np.array([[vx_meas], [vy_meas]]) # YOU REPLACE THIS

            # Form the modeled measurement
            h_vel = np.array([[m.cos(psi)*vIX + m.sin(psi)*vIY],[-m.sin(psi)*vIX + m.cos(psi)*vIY]]) # YOU REPLACE THIS

            # Take the Jacobian of the measurement model
            C_vel = np.array([[0, 0, -m.sin(psi)*vIX + m.cos(psi)*vIY, 0, 0, 0, m.cos(psi), m.sin(psi), 0], # YOU REPLACE THIS
                              [0, 0, -m.cos(psi)*vIX - m.sin(psi)*vIY, 0, 0, 0, -m.sin(psi), m.cos(psi), 0]])

            # Compute Chi Square to make sure a valid measurement has been received
            chiSq = np.dot(np.transpose((y_vel - h_vel)), np.dot(np.linalg.inv(np.dot(C_vel, np.dot(P, C_vel.transpose())) + R_vel), (y_vel - h_vel)))
            if (chiSq < 20):
                # Calculate the Kalman gain
                L_vel = np.dot(P, np.dot(C_vel.transpose(),
                                         np.linalg.inv(np.dot(C_vel, np.dot(P, C_vel.transpose())) + R_vel)))
                L_vel[2] = 0.0 # Don't let velocity measurement update yaw measurement
                # Update the covariance matrix
                I9 = np.identity(9)
                P = np.dot((I9 - np.dot(L_vel, C_vel)), P)
                P = np.real(0.5 * P + 0.5 * P.transpose())
                # Update the state
                x_hat = x_hat + np.dot(L_vel, (y_vel - h_vel))

# RUN Trajectory CONTROL
# ======================================================================================================================
# ======================================================================================================================
        # Extract Kalman Filter states
        phi = x_hat[0,0]
        theta = x_hat[1,0]
        psi = x_hat[2,0]
        pIX = x_hat[3,0]
        pIY = x_hat[4,0]
        pIZ = x_hat[5,0]
        vIX = x_hat[6,0]
        vIY = x_hat[7,0]
        vIZ = x_hat[8,0]

        pxI_cmd = 0
        pyI_cmd = 0
        if t > 5:
            if not WALL_FOUND:
                if (sensors.range_fwd > 500):
                    pxI_cmd = 0.5 + pIX # YOU REPLACE THIS
                    pyI_cmd = 0 # YOU REPLACE THIS
                elif (sensors.range_fwd <= 500):
                    WALL_FOUND = True
                    print("WALL FOUND at: ", pIX+sensors.range_fwd/10, "m X, and ", pIY, " m Y.")
                    drone.set_pitch(0)
            else:
                pxI_cmd = 0 # YOU REPLACE THIS
                pyI_cmd = 0 # YOU REPLACE THIS
                if (pIX <= 0.5 and pIY <= 0.5): # YOU REPLACE THIS
                    landing_count = landing_count + 1
                else:
                    landing_count = 0.0

                if landing_count > 10:
                    drone.land()
                    exit()

        pxError = pxI_cmd - pIX
        pyError = pyI_cmd - pIY

# RUN AUTOPILOT
# ======================================================================================================================
# ======================================================================================================================
        # Compute the "Inertial" to local level frame rotation matrix
        R_i2ll = eulerToRotationMatrix(phi, theta, psi)

        pxI_cmd = 1 # REMOVE THIS WHEN YOU ARE READY TO USE TRAJECTORY CONTROL
        pyI_cmd = -1 # REMOVE THIS WHEN YOU ARE READY TO USE TRAJECTORY CONTROL

        # Control the inertial x position
        # -----------------------------------------------------------------------
        tiltxI_cmd = pir_pxI.pir(pxI_cmd, pIX, vIX, t)

        # Control the inertial y position
        # -----------------------------------------------------------------------
        tiltyI_cmd = pir_pyI.pir(pyI_cmd, pIY, vIY, t)

        # Rotate command from Inertial to local level
        tiltLL_cmd = np.dot(R_i2ll, np.array([[tiltxI_cmd], [tiltyI_cmd], [0]]))
        drone.set_pitch(tiltLL_cmd[0])
        drone.set_roll(tiltLL_cmd[1])

        # Control altitude
        # -----------------------------------------------------------------------
        # Get the updated altitude data
        altitude = sensors.baroAltitude
        # Run the PIR altitude controller
        throttle_cmd = pir_alt.pir(altitude_cmd, altitude - altitude0, 0.0, t)
        # Set "Throttle" which really commands an up/down velocity
        drone.set_throttle(throttle_cmd)  # Throttle from -100-100

        # Send the command to the CoDrone
        if RUN_CONTROL:
            drone.move()

        print("pIX : ", pIX, ", pIY: ", pIY)

        if (P[0,0] < 0 or P[1,1] <0 or P[2,2]< 0 or P[3,3]<0 or P[4,4]<0 or P[5,5]<0 or P[6,6]<0 or P[7,7]<0 or P[8,8]<0):
            drone.land()
            print("Drone landing due to EKF instability")
            exit()
# LOG DATA
# ==============================================
# ==============================================
        data = [t, dt, sensors.accelX, sensors.accelY, sensors.accelZ, sensors.p, sensors.q, sensors.r,
                sensors.phi, sensors.theta, sensors.psi, phi, theta, psi, pIX, pIY, pIZ,
                vIX, vIY, vIZ, pxI_cmd, pyI_cmd, altitude_cmd, vx_meas, vy_meas, tiltLL_cmd[0,0], tiltLL_cmd[1,0], chiSq[0,0],
                m.sqrt(P[0,0]), m.sqrt(P[1,1]), m.sqrt(P[2,2]),
                m.sqrt(P[3,3]), m.sqrt(P[4,4]), m.sqrt(P[5,5]),
                m.sqrt(P[6,6]), m.sqrt(P[7,7]), m.sqrt(P[8,8]), alt_meas]
        sensor_log.write_data(data)

sensor_log.close()
drone.land()

