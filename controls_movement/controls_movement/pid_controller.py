import numpy as np
from rclpy.logging import get_logger

class PIDController:
    def __init__(self, Kp, Ki, Kd, isOri=False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.isOri = isOri
        self.previous_error = None
        self.integral = 0
        self.logger = get_logger("vert_pid_node")

    def boundAngle(self, angle):
        """
        Bound angle to [-pi, pi]
        """
        angle = angle % (2 * np.pi)
        if angle < -np.pi:
            return 2 * np.pi + angle
        elif angle > np.pi:
            return angle - 2 * np.pi
        else:
            return angle

    # def correctIMU(self, currAtt):
    #     """
    #     Correct IMU by subtracting imuZero.
    #     """
    #     corrAtt = []
    #     for att, zero in zip(currAtt, IMU_ZERO):
    #         corrAtt.append(self.boundAngle(att - zero))

    #     return corrAtt

    def getAngleError(self, currAngle, targetAngle):
        """
        If target = pi/2 and curr = pi,
            Unbounded error is pi/2.
            Bounded error is pi/2.
            Curr is pi/2 bigger than target.
            Rotate clockwise.
            -ve acceleration.

        If target = -pi/2 and curr = pi,
            Unbounded error is 3*pi/2.
            Bounded error is -pi/2.
            Curr is pi/2 smaller than target.
            Rotate anti-clockwise.
            +ve acceleration.

        If target = -pi/2 and curr = 0,
            Unbounded error is pi/2.
            Bounded error is pi/2.
            Curr is pi/2 bigger than target.
            Rotate clockwise.
            -ve acceleration.

        If target = 0 and curr = -pi,
            Unbounded error is -pi.
            Bounded error is -pi.
            Curr is pi smaller than target.
            Rotate anti-clockwise.
            +ve acceleration
        """
        # Might not need this
        currAngle = self.boundAngle(currAngle)
        targetAngle = self.boundAngle(targetAngle)

        return self.boundAngle(currAngle - targetAngle)


    def compute(self, setpoint, current_value, dt, kd_multiplier=0, ki_multiplier=0, integral_limit=200.0):        
        if self.isOri:
            # account for wrap around if we are dealing with orientation PID (roll, pitch , yaw)
            error = self.getAngleError(currAngle=current_value/180*np.pi, targetAngle=setpoint/180*np.pi)
            error = -error * 180 / np.pi
            if self.previous_error is not None:
                derror = self.getAngleError(error, self.previous_error)
            else:
                derror = 0
        else:
            error = setpoint - current_value
            if self.previous_error is not None:
                derror = (error - self.previous_error) / dt
            else:
                derror = 0

        finalError = initialError = error

        self.integral = max(-integral_limit, min(integral_limit, (self.integral) + error * dt))
        self.derivative = derror / dt if dt > 0 else 0.0
        self.previous_error = error

        output = self.Kp * error + self.Ki*(10**ki_multiplier) * self.integral + self.Kd*(10**kd_multiplier) * self.derivative
        return (output, initialError, finalError, self.derivative)
    
    def update_consts(self, new_Kp, new_Ki, new_Kd):
        self.Kp = new_Kp
        self.Ki = new_Ki
        self.Kd = new_Kd
        return f'Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}'
        # return f'Kp: {new_Kp}, Ki: {new_Ki}, Kd: {new_Kd}'
    
    def __str__(self):
        return f'Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd}'