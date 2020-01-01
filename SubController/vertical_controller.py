from SubController.PID import PID
from SubController.angle_optimization import angle_optimization
from SubController.thruster_controller import Thruster_Controller


class Vertical_Controller:
    def __init__(self, front_left_pin, front_right_pin, back_left_pin, back_right_pin, v_pid_konstants, y_pid_konstants, z_pid_konstants,
                 v_setpoint = 0, y_setpoint = 0, z_setpoint = 0):
        self.front_left = Thruster_Controller(front_left_pin)
        self.front_right = Thruster_Controller(front_right_pin)
        self.back_left = Thruster_Controller(back_left_pin)
        self.back_right = Thruster_Controller(back_right_pin)

        #create PID controllers with defaults
        self.v_pid = PID()
        self.y_pid = PID()
        self.z_pid = PID()

        #set the initial setpoints
        self.set_setpoints(v_setpoint, y_setpoint, z_setpoint)

        #set the kp, ki, and kd
        self.v_pid.tunings(v_pid_konstants[0],v_pid_konstants[1],v_pid_konstants[2])
        self.y_pid.tunings(y_pid_konstants[0],y_pid_konstants[1],y_pid_konstants[2])
        self.z_pid.tunings(z_pid_konstants[0],z_pid_konstants[1],z_pid_konstants[2])


    def set_setpoints(self, v_setpoint, y_setpoint = 0, z_setpoint = 0):
        self.v_setpoint = v_setpoint
        self.y_setpoint = y_setpoint
        self.z_setpoint = z_setpoint

        self.v_pid.set_setpoint(v_setpoint)
        self.y_pid.set_setpoint(y_setpoint)
        self.z_pid.set_setpoint(z_setpoint)

    def __call__(self, v_current, y_current, z_current):
        # this is where the mixer is added in to the stuff

        # calculaion to reframe the problem
        real_y_setpoint = angle_optimization(y_current)
        real_z_setpoint = angle_optimization(z_current)
        y_current = 0
        z_current = 0

        # update the setpoints, and input to the modified values
        self.set_setpoints(self.v_setpoint, real_y_setpoint, real_z_setpoint)

        # calculate the new outputs
        # make sure they are within defined limits
        v_output = self.linear_limit(self.h_pid(v_current))
        y_output = self.angular_limit(self.y_pid(y_current))
        z_output = self.angular_limit(self.z_pid(z_current))

        # mix the values
        outputs = self.mixer(v_output, y_output, z_output)

        # send the modified signals to the thrusters
        self.front_left.set_pwm(outputs[0])
        self.front_right.set_pwm(outputs[1])
        self.back_left.set_pwm(outputs[2])
        self.back_right.set_pwm(outputs[3])

    def mixer(self, v_output, y_output, z_output):
        neutral = 1500
        front_right = neutral + v_output + y_output + z_output
        front_left = neutral + v_output - y_output + z_output
        back_left = neutral + v_output - y_output - z_output
        back_right = neutral + v_output + y_output - z_output

        return (front_left, front_right, back_left, back_right)

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