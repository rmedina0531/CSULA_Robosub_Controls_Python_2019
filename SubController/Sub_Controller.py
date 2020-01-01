from SubController.horizontal_controller import Horizontal_Controller
from SubController.vertical_controller import Vertical_Controller


class Sub_Controller:
    def __init__(self, pins, h_konstants, v_konstants, x_konstants, y_konstants, z_konstants):
        self.h_controller = Horizontal_Controller(pins[0], pins[1], h_konstants, x_konstants)
        self.v_controller = Vertical_Controller(pins[2],pins[3],pins[4],pins[5], v_konstants, y_konstants, z_konstants)

    def set_setpoints(self, h_setpoint, v_setpoint, x_setpoint, y_setpoint, z_setpoint):
        self.h_controller(h_setpoint, x_setpoint)
        self.v_controller(v_setpoint, y_setpoint, z_setpoint)

    def __call__(self, h_current, v_current, x_current, y_current, z_current):
        self.h_controller(h_current, x_current)
        self.v_controller(v_current, y_current, z_current)

    def set_limits(self, linear_limits, angular_limits):
        self.h_controller.set_limits(linear_limits, angular_limits)
        self.v_controller.set_limits(linear_limits, angular_limits)

