import libraries.Servo

class Thruster_Controller:
    def __init__(self, pin):
        self.servo = libraries.Servo(pin)

    def set_pwm(self, pwm):
        self.servo.write_pwm(pwm)