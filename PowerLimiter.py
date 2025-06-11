from pid import PID

class PowerLimiter:
    """
    Mirrors PowerLimit struct + methods in powerLimit.c (mode 1 & 2)
    """
    def __init__(self):
        self.pid  = PID(0, 0, 0, 231, 10)
        self.mode = 1
        self.target_kw      = 80
        self.discrepancy_kw = 20
        self.always_on      = True
        self.enabled        = False
        self.torque_cmd_Nm  = 0

    # helper -------------------------------------------------------------

    @property
    def init_threshold_kw(self):
        return self.target_kw - self.discrepancy_kw

    # public API ---------------------------------------------------------

    def step(self, mech_power_kw, rpm, driver_torque_request_Nm):
        """Return limited torque command (Nm)"""
        # ------------- enable/disable -------------
        if driver_torque_request_Nm == 0:
            self.enabled = False
        else:
            if mech_power_kw > self.init_threshold_kw:
                if self.always_on or not self.enabled:
                    self.enabled = True

        if not self.enabled:
            self.torque_cmd_Nm = driver_torque_request_Nm
            return self.torque_cmd_Nm

        # ------------- mode 1 : torque-equation -------------
        if self.mode == 1:
            setpoint_Nm = (self.target_kw*1000 - 2000) * 9549 / max(rpm, 1)
            self.pid.sat = 231
            self.pid.update_setpoint(setpoint_Nm)
            correction = self.pid.compute(driver_torque_request_Nm)
            self.torque_cmd_Nm = max(0, min(231, setpoint_Nm))  # exact copy of C
            return self.torque_cmd_Nm

        # ------------- mode 2 : power-PID -------------
        elif self.mode == 2:
            self.pid.sat = 8000
            self.pid.update_setpoint(self.target_kw*1000)
            correction = self.pid.compute(mech_power_kw*1000)
            tq = (correction + mech_power_kw*1000) / (max(rpm, 1)*9.549)
            self.torque_cmd_Nm = max(0, min(tq, 231))
            return self.torque_cmd_Nm
