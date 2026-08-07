class PIRController:
    def __init__(self, kp, ki, kd, u_lower_limit, u_upper_limit, time):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.u_lower_limit = u_lower_limit
        self.u_upper_limit = u_upper_limit
        self.error_int = 0.0
        self.time = time

    def pir(self, y_c, y, y_dot, time, wrap_error_180=False):

        # Perform "PI with rate feedback"
        error = y_c - y

        # If the flag is set, wrap the error to +/- 180 degrees. This is used for yaw control
        if wrap_error_180:
            while error >= 180:
                error = error - 180
            while error <= -180:
                error = error + 180

        dt = time - self.time
        self.time = time
        self.error_int = self.error_int + dt*error
        u = self.kp*error + self.ki*self.error_int - self.kd*y_dot

        # Output saturation & integrator clamping
        #   - Limit u to u_upper_limit & u_lower_limit
        #   - Clamp if error is driving you past limit
        if u > self.u_upper_limit:
            u = self.u_upper_limit
            if self.ki*error > 0:
                self.error_int = self.error_int - dt*error
        elif u < self.u_lower_limit:
            u = self.u_lower_limit
            if self.ki*error < 0:
                self.error_int = self.error_int - dt*error

        return u
