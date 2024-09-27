#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def call_fun(in_string):
    rospy.loginfo(in_string.data)

def listner():
    rospy.init_node('listener',anonymous=False)
    rospy.Subscriber('out_string',String,call_fun)
    rospy.spin()

if __name__=='__main__':
    try:
        listner()
    except rospy.ROSInterruptException:
        pass

