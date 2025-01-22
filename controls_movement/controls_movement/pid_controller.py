class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = None
        self.integral = 0

    def compute(self, setpoint, current_value, dt, kd_multiplier, ki_multiplier):
        error = setpoint - current_value
        self.integral = (self.integral*0.95) + error * dt / (10**ki_multiplier)
        derivative = (error - self.previous_error) / dt / (10**kd_multiplier) if dt > 0 and self.previous_error is not None else 0.0
        self.previous_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # return (output, self.Kp * error, self.Kd, derivative, dt)
        return (output, self.Ki * self.integral, self.Kd, self.Kd * derivative)
    
    def update_consts(self, new_Kp, new_Ki, new_Kd):
        self.Kp = new_Kp
        self.Ki = new_Ki
        self.Kd = new_Kd
        return f'Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}'
        # return f'Kp: {new_Kp}, Ki: {new_Ki}, Kd: {new_Kd}'
    
    def __str__(self):
        return f'Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}'