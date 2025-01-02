from thruster_allocator import ThrustAllocator
import time

'''
Standard PID implementation
Used as a class since both yaw and pitch use different instances of the same PID algo
'''
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value, dt):
        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output


'''
This class uses visual feed to determine yaw and pitch
From which we conduct PID

e.g., on the camera feed
object is detected at (20, 25)
when the bot moves straight forward, it moves towards the point at (0, 0) (this point is fixed regardless of the robot's true orientation)
then, we will use our error values as (0, 0) - (20, 25) == (-20, -25)
on which, we will do PID
'''
class OrientationController:
    def __init__(self, thrust_allocator):

        self.thrust_allocator = thrust_allocator
        #self.roll_pid = PIDController(*pid_gains['roll'])   No need for roll in this situation but it can be implemented too ig
        self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)  
        self.yaw_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)     

    def correct_orientation(self, current_pitch, current_roll, current_yaw, dt):

        roll_output = 0
        pitch_output = self.pitch_pid.compute(setpoint=0, current_value=current_pitch, dt=dt)
        yaw_output = self.yaw_pid.compute(setpoint=0, current_value=current_yaw, dt=dt)

        # Get PWM values from the thrust allocator
        pwm_values = self.thrust_allocator.getRotationPwm([roll_output, pitch_output, yaw_output])
        return pwm_values






#constant
dt = 0.4

def main():
    t = ThrustAllocator()
    controller = OrientationController(thrust_allocator=t)
    while True:
        current_pitch = 5  #example values. need to find a way to read from perc side the actual values!
        current_roll = 0
        current_yaw = 3

        if (abs(current_pitch) > 3 or abs(current_yaw) > 3): #if error exceeds a certain range, we use PID to correct for it
            pwm_values = controller.correct_orientation(current_pitch, current_roll, current_yaw, dt)
            print(pwm_values)
        else:
            print(t.getTranslationPwm([10, 0, 0])) #move straight ahead

        time.sleep(dt)




if __name__ == "__main__":
    main()