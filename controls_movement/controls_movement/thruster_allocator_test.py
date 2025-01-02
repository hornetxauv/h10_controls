"""
To run this test manually, run it from _ws directory, as such
Otherwise, [Errno 2] No such file or directory for thrust_map.csv

izen@Izen-Pavillion:~/HornetXAuv_ws$ python3 ./src/h10_controls/controls_movement/controls_movement/thruster_allocator_test.py

"""

import numpy as np
from thruster_allocator import ThrustAllocator, thruster_names, thrust_map

def test_thruster_matrix(t):
    for i in range(7):
        s = thruster_names[i] + "\t"
        for j in range(6):
            s += " " + str(t.parameters[j][i])
    
        print(s)

def test_translation(t):
    ''' Test translation '''
    vfunc = pwm_to_thrust()
    print("+X translation: ", vfunc(t.getTranslationPwm([10, 0, 0]))) # Translation in x-axis
    print("+Y translation: ", vfunc(t.getTranslationPwm([0, 10, 0]))) # Translation in y-axis
    print("+Z translation: ", vfunc(t.getTranslationPwm([0, 0, 10]))) # Translation in z-axis

def test_rotation(t):
    ''' Test rotation'''
    vfunc = pwm_to_thrust()
    print("+Roll rotation: ", vfunc(t.getRotationPwm([10, 0, 0]))) # Roll right
    print("-Roll rotation: ", vfunc(t.getRotationPwm([-10, 0, 0]))) # Roll left
    print("+Pitch rotation: ", vfunc(t.getRotationPwm([0, 10, 0]))) # Pitch up
    print("-Pitch rotation: ", vfunc(t.getRotationPwm([0, -10, 0]))) # Pitch down
    print("+Yaw rotation: ", vfunc(t.getRotationPwm([0, 0, 10]))) # Yaw left 
    print("-Yaw rotation: ", vfunc(t.getRotationPwm([0, 0, -10]))) # Yaw right

def pwm_to_thrust():
    def get_thrust(pwm):
        idx = np.searchsorted(thrust_map[:, 1], pwm, 'left')
        return thrust_map[idx][0]
    
    return np.vectorize(get_thrust)

def main():
    t = ThrustAllocator()

    print("--- Thruster matrix ---")
    test_thruster_matrix(t)
    print("\n\n")

    print("--- Thruster translation test---")
    test_translation(t)
    print("\n\n")
    
    print("--- Thruster rotation test ---")
    test_rotation(t)   
    print("\n\n")

if __name__ == "__main__":
    main()
