import rclpy
from rclpy.node import Node

from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   

from control_panel.control_panel import create_control_panel, ControlPanelItem as CPI #this is a package in PL repo



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

        Translation = [0, 0, 0]

        #Rotation = [0, 0, 0]
        #if all(val == 0 for val in Translation):

        thruster_pwm = thrusterAllocator.getTranslationPwm(Translation)

        self.thrusterControl.setThrusters(thrustValues=thruster_pwm)

if __name__ == "__main__":
    main()