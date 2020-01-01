from SubController.Sub_Controller import Sub_Controller


def setup():
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
    sub = Sub_Controller([left_thruster,right_thruster,front_left_thruster,front_right_thruster,back_left_thruster, back_right_thruster],
                         h_pid_konstants,v_pid_konstants,x_pid_konstants, y_pid_konstants, z_pid_konstants)

    #define the limits for the verticle horizontal and angle pids
    linear_limits = (-300, 300)
    angle_limits = (-100, 100)
    sub.set_limits(linear_limits, angle_limits)






def loop():
    #read the topic
    #update setpoints
    #Compute PID
    True


def controls_test():
    True

def controls():
    setup()
    loop()

if __name__ == "main":
    #run if not imported
    controls_test()

