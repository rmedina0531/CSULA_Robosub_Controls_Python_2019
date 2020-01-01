from SubController.PID import PID
from SubController.thruster_controller import Thruster_Controller
from SubController.angle_optimization import angle_optimization


class Horizontal_Controller:
    def __init__(self, left_pin, right_pin, h_pid_konstants, x_pid_konstants, hor_setpoint = 0, angle_setpoint = 0):
        #set limits to default
        self.linear_max, self.linear_min = 1800, 1200
        self.angular_max, self.angular_min = 1800, 1200


        self.left_thruster = Thruster_Controller(left_pin)
        self.right_thruster = Thruster_Controller(right_pin)
        self.angle_setpoint = angle_setpoint
        self.hor_setpoint = hor_setpoint
        #create PID controllers with defaults
        self.h_pid = PID()
        self.a_pid = PID()

        #set the kp, ki, and kd
        self.h_pid.tunings(h_pid_konstants[0],h_pid_konstants[1],h_pid_konstants[2])
        self.a_pid.tunings(x_pid_konstants[0],x_pid_konstants[1],x_pid_konstants[2])

        #set the setpoints
        self.set_setpoints(hor_setpoint, angle_setpoint)

    def __call__(self, h_current, a_current):
        #this is where the mixer is added in to the stuff

        #calculaion to reframe the problem
        real_angle_setpoint = angle_optimization(a_current)
        a_current = 0

        #update the setpoints, and input to the modified values
        self.set_setpoints(0, real_angle_setpoint)

        #calculate the new outputs
        #make sure they are wihin the defined limits
        h_output = self.linear_limit(self.h_pid(h_current))
        a_output = self.angular_limit(self.a_pid(a_current))

        #mix the values
        outputs = self.mixer(h_output, a_output)

        #send the modified signals to the thrusters
        self.left_thruster.set_pwm(outputs[0])
        self.right_thruster.set_pwm(outputs[1])

    def set_setpoints(self, hor_setpoint, angle_setpoint):
        self.hor_setpoint = hor_setpoint
        self.angle_setpoint = angle_setpoint

        # set the setpoints
        self.h_pid.set_setpoint(self.hor_setpoint)
        self.a_pid.set_setpoint(self.angle_setpoint)

    def mixer(self, h_output, a_output):
        #set the neutral on the pwm signal
        neutral = 1500
        left_output = neutral + h_output + a_output
        right_output = neutral + h_output - a_output
        return (left_output, right_output)

    def set_limits(self, linear_limits, angular_limits):
        self.linear_min = linear_limits[0]
        self.linear_max = linear_limits[1]

        self.angular_min = angular_limits[0]
        self.angular_max = angular_limits[1]

    def linear_limit(self, value):
        if value > self.linear_max:
            return self.linear_max
        if value < self.linear_min:
            return self.linear_min
        return value

    def angular_limit(self, value):
        if value > self.angular_max:
            return self.angular_max
        if value < self.angular_min:
            return self.angular_min
        return value
