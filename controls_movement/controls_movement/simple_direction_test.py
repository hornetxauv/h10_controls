import rclpy
from rclpy.node import Node

from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   

from control_panel.control_panel import create_control_panel, ControlPanelItem as CPI #this is a package in PL repo

values = {
    # 'z offset': CPI(value=1.0, maximum=50, minimum=0.1, multiplier=10),
    'x': CPI(value=20, maximum=40, minimum=0, multiplier=1),
    'y': CPI(value=20, maximum=40, minimum=0, multiplier=1),
    'z': CPI(value=20, maximum=40, minimum=0, multiplier=1),
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

    while True:

        # Translation = [0, 0, 0]
        Translation = [values["x"].value-20, values["y"].value-20, values["z"].value-20]

        #Rotation = [0, 0, 0]
        #if all(val == 0 for val in Translation):

        thruster_pwm = thrusterAllocator.getTranslationPwm(Translation)

        print(thruster_pwm)

        thrusterController.setThrusters(thrustValues=thruster_pwm)

if __name__ == "__main__":
    main()