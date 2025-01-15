import rclpy
from rclpy.node import Node

from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   

from control_panel.control_panel import create_control_panel, ControlPanelItem as CPI #this is a package in PL repo

x=100

values = {
    # 'z offset': CPI(value=1.0, maximum=50, minimum=0.1, multiplier=10),
    'x': CPI(value=x/2, maximum=x, minimum=0, multiplier=1),
    'y': CPI(value=x/2, maximum=x, minimum=0, multiplier=1),
    'z': CPI(value=x/2, maximum=x, minimum=0, multiplier=1),
}
# can't seem to use simultaneously with thruster biases control panel... sometimes. idk.
create_control_panel("verti PID", values)

'''
[X, Y, Z]
- +X is right
- +Y is forward
- +Z is up

[R, P, Y]
 - +R is roll right
 - +P is pitch up
 - +Y is yaw left
'''

def main():
    thrusterAllocator = ThrustAllocator()
    thrusterController = ThrusterControl()
    try:
        while True:
            # Translation = [0, 0, 0]
            Rotation = [values["x"].value-x/2, values["y"].value-x/2, values["z"].value-x/2]

            #Rotation = [0, 0, 0]
            #if all(val == 0 for val in Translation):

            thruster_pwm = thrusterAllocator.getRotationPwm(Rotation)

            print(thruster_pwm)

            thrusterController.setThrusters(thrustValues=thruster_pwm)
    except KeyboardInterrupt:
        thrusterController.killThrusters()
    finally:
        thrusterController.killThrusters()
        rclpy.shutdown()


if __name__ == "__main__":
    main()