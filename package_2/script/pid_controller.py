#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, Twist

# Initialize global variables
pub = None
sub = None
rate= None

data_out = Twist()
set_point = 0.5
x_data = [0, 0]
y_data = [0, 0]
y_array = [3.5, -3.5]

def pid_calculation(data):
    global x_data, y_data, data_out, pub,rate
    
    # PI controller calculation
    x_data[1] = set_point - data.x
    y_data[1] = (y_array[0] * x_data[1]) + (y_array[1] * x_data[0]) + y_data[0]

    # Update for next iteration
    y_data[0] = y_data[1]
    x_data[0] = x_data[1]

    # Prepare and publish the Twist message
    data_out.linear.x = y_data[1]
    pub.publish(data_out)

    rate.sleep



def pid_controller():
    global pub,sub,rate

    rospy.init_node('pid_x_controller', anonymous=False)
    pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/pioneer/position', Pose2D, pid_calculation)
    rate = rospy.Rate(50)  # 50 Hz
    rospy.spin()

if __name__ == '__main__':
    try:
        pid_controller()
    except rospy.ROSInterruptException:
        pass
