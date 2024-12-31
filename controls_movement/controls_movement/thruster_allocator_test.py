"""
To run this test manually, run it from _ws directory, as such
Otherwise, [Errno 2] No such file or directory for thrust_map.csv

izen@Izen-Pavillion:~/HornetXAuv_ws$ python3 ./src/h10_controls/controls_movement/controls_movement/thruster_allocator_test.py

"""


from thruster_allocator import ThrustAllocator

def main():
    t = ThrustAllocator()
    print(t.parameters)
    ''' Test translation '''
    print("X translation: ", t.getTranslationPwm([10, 0, 0])) # Translation in x-axis
    print("Y translation: ", t.getTranslationPwm([0, 10, 0])) # Translation in y-axis
    print("Z translation: ", t.getTranslationPwm([0, 0, 10])) # Translation in z-axis
    
    ''' Test rotation'''
    print("XY +ve rotation: ", t.getRotationPwm([0, 0, -10])) # Positive Yaw
    print("XY -ve rotation: ", t.getRotationPwm([0, 0, 10])) # Negative Yaw
    print("Z +ve rotation: ", t.getRotationPwm([-10, 0, 0])) # Positive Pitch
    print("Z -ve rotation: ", t.getRotationPwm([10, 0, 0])) # Negative Pitch

if __name__ == "__main__":
    main()
