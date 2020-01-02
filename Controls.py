from SubController.Sub_Controller import Sub_Controller

#initial test with multiple subscrubers
import rospy
from std_msgs.msg import Int32

class Controls:
    def listener(self):
        # subscribers setup
        rospy.init_node('listener')

        #names of the two subscribers
        rospy.Subscriber('/mission_control', Float64MultiArray, self.mission_control_data_callback)
        rospy.Subscriber('/nav', Float64MultiArray, self.nav_data_callback)

        rospy.spin()

    def setup(self):

        #pin for each thruster
        left_thruster, right_thruster = 1,2
        front_left_thruster, front_right_thruster, back_left_thruster, back_right_thruster = 3,4,5,6

        #define all the constants

        h_pid_konstants = [1,0,0]
        #formerly barometer constants
        v_pid_konstants = [400,0,0]

        x_pid_konstants = [3.5,0.0005,4]
        y_pid_konstants = [3.5,0.0005,4]
        z_pid_konstants = [3.5,0.0005,4]

        #initialize the sub controller
        self.sub = Sub_Controller([left_thruster,right_thruster,front_left_thruster,front_right_thruster,back_left_thruster, back_right_thruster],
                             h_pid_konstants,v_pid_konstants,x_pid_konstants, y_pid_konstants, z_pid_konstants)

        #define the limits for the verticle horizontal and angle pids
        linear_limits = (-300, 300)
        angle_limits = (-100, 100)
        self.sub.set_limits(linear_limits, angle_limits)

    def nav_data_callback(self, msg):
        #current nav data updated
        #send new values to the controllers

        h_current = msg.data[0]
        v_current = msg.data[1]
        x_current = msg.data[2]
        y_current = msg.data[3]
        z_current = msg.data[4]

        self.sub(h_current, v_current, x_current, y_current, z_current)


    def mission_control_data_callback(self, msg):
        #new setpoints received from Mission control
        #set new setpoints to all the controllers
        h_setpoint = msg.data[0]
        v_setpoint = msg.data[1]
        x_setpoint = msg.data[2]
        y_setpoint = msg.data[3]
        z_setpoint = msg.data[4]


        self.sub.set_setpoints(h_setpoint, v_setpoint, x_setpoint, y_setpoint, z_setpoint)

    def __init__(self):
        self.setup()
        self.listener()

if __name__ == "main":
    c = Controls()

    #uncomment to run test
    #controls_test()

