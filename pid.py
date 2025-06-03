class PID:
    def __init__(self, Kp, Ki, Kd, saturation, dH=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.saturation = saturation
        self.dH = dH

        self.setpoint = 0
        self.previous_error = 0
        self.total_error = 0
        self.output = 0
        self.derivative = 0

    def update_setpoint(self, setpoint):
        self.setpoint = min(setpoint, self.saturation)

    def compute(self, sensor_value):
        error = self.setpoint - sensor_value

        proportional = self.Kp * error / 10
        self.total_error += error
        integral = self.Ki * self.total_error / self.dH / 10

        diff = error - self.previous_error
        alpha = 0.1
        self.derivative = alpha * (self.Kd * diff * self.dH / 10) + (1 - alpha) * self.derivative
        self.previous_error = error

        output = proportional + integral + self.derivative

        if self.saturation > 0:
            output = max(-self.saturation, min(output, self.saturation))

        self.output = output
        return output
