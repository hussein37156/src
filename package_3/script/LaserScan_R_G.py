import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

pub = None
position_matrix = None
transformation_matrix = None
data_out = LaserScan()

def call_back_1(data_in):
    global position_matrix, data_out
    data_out = data_in
    laser = np.array(data_in.ranges)
    angles = np.linspace(data_in.angle_min, data_in.angle_max, len(laser))
    x = laser * np.cos(angles)
    y = laser * np.sin(angles)
    z = np.ones_like(x)
    dummy_row = np.ones_like(x)
    position_matrix = np.column_stack((x, y, z, dummy_row)).T

def call_back_2(data_in):
    global transformation_matrix
    transformation_matrix = np.array(data_in.data).reshape(3, 4)

def calc():
    global transformation_matrix, position_matrix, data_out
    if transformation_matrix is None or position_matrix is None:
        return
    transformed_position = transformation_matrix.dot(position_matrix)
    transformed_ranges = np.sqrt(np.square(transformed_position[0]) + np.square(transformed_position[1]))
    data_out.header.frame_id = 'map'
    data_out.ranges = transformed_ranges
    pub.publish(data_out)

def LaserScan_R_G():
    global pub
    rospy.init_node('LaserScan_R_G', anonymous=False)
    pub = rospy.Publisher('/Hokuyo/R_G', LaserScan, queue_size=10)
    rospy.Subscriber('pioneer/hokuyo', LaserScan, call_back_1)
    rospy.Subscriber('pioneer/hokuyo/transformation', Float32MultiArray, call_back_2)

    while not rospy.is_shutdown():
        calc()


if __name__ == '__main__':
    try:
        LaserScan_R_G()
    except rospy.ROSInternalException:
        pass
