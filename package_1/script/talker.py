#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=False)
    pub = rospy.Publisher('out_string', String, queue_size=10)
    rate = rospy.Rate(1)
    rospy.loginfo('you loged in to talker node')
    while not rospy.is_shutdown():
        out_string = 'hello hussein %s' % rospy.get_time()
        pub.publish(out_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
