class Servo:
    def __init__(self, pin):
        self.pin = pin
    def write_pwm(self, pwm):
        print("Pin " + self.pin + " output: " + pwm)