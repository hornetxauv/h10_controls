from pid_controller import PIDController

r = PIDController(0.1, 0, 0.1)
# r = PIDController(0.1, 0, 0)
print(r.compute(0, 20, 0.01))
print(r.compute(0, 10, 0.01))