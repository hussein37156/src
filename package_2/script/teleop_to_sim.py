#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

pub = None 


def call_fun(data):
    pub.publish(data)

def teleop_to_sim():
    global pub
    rospy.loginfo('you loged in to teleop to simulation node')
    pub=rospy.Publisher('sim_ros_interface/pioneer/cmd_vel', Twist, queue_size=10)
    rospy.init_node('teleop_to_sim')
    rospy.Subscriber('turtle1/cmd_vel',Twist,call_fun)
    rospy.spin()


if __name__=="__main__":
    
    try:
        teleop_to_sim()
    except rospy.ROSInterruptException:
        pass

