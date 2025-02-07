class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = None
        self.integral = 0

    def compute(self, setpoint, current_value, dt, kd_multiplier, ki_multiplier, integral_limit=200.0):
        error = setpoint - current_value
        self.integral = max(-integral_limit, min(integral_limit, (self.integral) + error * dt))
        # self.integral = max(-3/1.1, min(3/1.1, self.integral))
        self.derivative = (error - self.previous_error) / dt if dt > 0 and self.previous_error is not None else 0.0
        self.previous_error = error


        output = self.Kp * error + self.Ki*(10**ki_multiplier) * self.integral + self.Kd*(10**kd_multiplier) * self.derivative
        # return (output, self.Kp * error, self.Kd, derivative, dt)
        return (output, error, self.integral, self.derivative)
    
    def update_consts(self, new_Kp, new_Ki, new_Kd):
        self.Kp = new_Kp
        self.Ki = new_Ki
        self.Kd = new_Kd
        return f'Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}'
        # return f'Kp: {new_Kp}, Ki: {new_Ki}, Kd: {new_Kd}'
    
    def __str__(self):
        return f'Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}'