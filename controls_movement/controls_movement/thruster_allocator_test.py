from thruster_allocator import ThrustAllocator

def main():
    t = ThrustAllocator()
    print(t.parameters)
    ''' Test translation '''
    print("X translation: ", t.getTranslationPwm([10, 0, 0])) # Translation in x-axis
    print("Y translation: ", t.getTranslationThrusts([0, 10, 0])) # Translation in y-axis
    print("Z translation: ", t.getTranslationThrusts([0, 0, 10])) # Translation in z-axis
    
    ''' Test rotation'''
    print("XY +ve rotation: ", t.getRotationThrusts([0, 0, -10])) # Positive Yaw
    print("XY -ve rotation: ", t.getRotationThrusts([0, 0, 10])) # Negative Yaw
    print("Z +ve rotation: ", t.getRotationThrusts([-10, 0, 0])) # Positive Pitch
    print("Z -ve rotation: ", t.getRotationThrusts([10, 0, 0])) # Negative Pitch

if __name__ == "__main__":
    main()
