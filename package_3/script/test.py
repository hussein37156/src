import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray 
from sensor_msgs.msg import LaserScan

pub = None
laser = None
transformation = None
angles = None

x_position = None
y_position = None
z_position = None
dummy = None

data_out = LaserScan()
data_inn = None

def call_back_1(data_in):
    global laser, angles, data_out
    laser = np.array(data_in.ranges)+data_in.range_max
    num_steps = len(laser)
    angles = np.linspace(data_in.angle_min, data_in.angle_max, num_steps)
    data_out = data_in

def call_back_2(data_in):
    global transformation
    transformation = np.array(data_in.data).reshape(3, 4)

def calc():
    global x_position, y_position, z_position, dummy, transformation, data_out
    if laser is not None and angles is not None and transformation is not None:
        x_position = laser * np.cos(angles)
        y_position = laser * np.sin(angles)
        z_position = np.full_like(x_position, 0.5)
        dummy = np.ones_like(x_position)
        positions = np.column_stack((x_position, y_position, z_position, dummy)).T

        transformed_positions = transformation.dot(positions)
        ranges_transformed = np.sqrt(np.square(transformed_positions[0]) + np.square(transformed_positions[1]))
        
        data_out.header.frame_id = 'map'
        data_out.ranges = ranges_transformed.tolist()
        
        pub.publish(data_out)

def listener():
    global pub
    rospy.init_node('listener', anonymous=False)
    pub = rospy.Publisher('/hokuyo_G', LaserScan, queue_size=10)
    rospy.Subscriber('pioneer/hokuyo', LaserScan, call_back_1)
    rospy.Subscriber('pioneer/hokuyo/transformation', Float32MultiArray, call_back_2)
    
    while not rospy.is_shutdown():
        calc()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
