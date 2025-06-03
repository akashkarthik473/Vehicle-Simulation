class PID:
    def __init__(self, kp, ki, kd, saturation, gain_factor=10):
        self.Kp, self.Ki, self.Kd = kp, ki, kd
        self.sat  = saturation
        self.gf   = gain_factor        # 10 → 0.1 resolution like the C code
        self.set  = 0
        self.errI = 0
        self.prev = 0
        self.out  = 0

    def update_setpoint(self, val): self.set = val

    def compute(self, value):
        err = self.set - value
        self.errI += err
        d_err = err - self.prev
        self.prev = err

        raw = (
            self.Kp * err     +
            self.Ki * self.errI / self.gf +
            self.Kd * d_err * self.gf
        )
        self.out = max(-self.sat, min(raw, self.sat))
        return self.out
