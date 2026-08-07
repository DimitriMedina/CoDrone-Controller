from codrone_edu.drone import *

# Gets sensor data and puts it into a forward-right-down frame
# CoDrone default frame is forward-left-up


class SensorData:
    def __init__(self, drone):
        self.drone = drone
        self.time = 0.0  # s
        self.temperature = 0.0  # Celsius
        self.pressure = 0.0  # Pa
        self.baroAltitude = 0.0  # m above sea level
        self.rangeHeight = 0.0  # m from sensor

        self.accelX = 0.0  # m/s^2
        self.accelY = 0.0  # m/s^2
        self.accelZ = 0.0  # m/s^2
        self.p = 0.0  # deg/sec
        self.q = 0.0  # deg/sec
        self.r = 0.0  # deg/sec
        self.phi = 0.0  # deg
        self.theta = 0.0  # deg
        self.psi = 0.0  # deg

        self.position_x = 0.0  # meters
        self.position_y = 0.0  # meters
        self.position_z = 0.0  # meters

        self.velocity_x = 0.0  # cm/sec
        self.velocity_y = 0.0  # cm/sec

        self.range_fwd = 0.0  # meters
        self.range_btm = 0.0  # meters

        self.modeSystem = 0
        self.modeFlight = 0
        self.modeControlFlight = 0
        self.headless = 0
        self.sensorOrientation = 0
        self.battery = 0
        self.modeMovement = 0
        self.controlSpeed = 0

    def update_position_data(self, dt=0.01):
        # Get the position data - only populates after takeoff command has been given
        position_data = self.drone.get_position_data(dt)

        self.time = position_data[0]
        self.position_x = position_data[1]  # meters
        self.position_y = -position_data[2]  # meters
        self.position_z = -position_data[3]  # meters

    def update_motion_data(self, dt=0.01):
        # Get the accelerometer, gyroscope "measurements", and CoDrone estimated Euler angles
        motion_data = self.drone.get_motion_data(dt)          # 0.01 second delay
        self.time = motion_data[0]
        self.accelX = motion_data[1] / 10.0  # m/s^2
        self.accelY = -motion_data[2] / 10.0  # m/s^2
        self.accelZ = -motion_data[3] / 10.0  # m/s^2
        self.p = motion_data[4]  # deg/sec
        self.q = -motion_data[5]  # deg/sec
        self.r = -motion_data[6]  # deg/sec
        self.phi = motion_data[7]  # deg
        self.theta = -motion_data[8]  # deg
        self.psi = -motion_data[9]  # deg

    def update_range_data(self, dt=0.01):
        range_data = self.drone.get_range_data(dt)            # 0.01 second delay
        self.time = range_data[0]
        self.range_fwd = range_data[1]  # mm
        self.range_btm = range_data[2]  # mm

    def update_velocity_data(self, dt=0.01):
        # Get "velocity" data from optical flow sensor
        flow_data = self.drone.get_flow_data(dt)              # 0.01 second delay
        self.time = flow_data[0]
        self.velocity_x = flow_data[1]/10.0  # cm/second - approximately
        self.velocity_y = -flow_data[2]/10.0 # cm/second - approximately

    def update_alt_data(self, dt=0.01):
        # Get the altitude data, to include temperature and pressure
        alt_data = self.drone.get_altitude_data(dt)           # 0.01 second delay
        self.time = alt_data[0]
        self.temperature = alt_data[1]  # Celsius
        self.pressure = alt_data[2]  # Pa
        self.baroAltitude = alt_data[3]  # m above sea level
        self.rangeHeight = alt_data[4]  # m from sensor

