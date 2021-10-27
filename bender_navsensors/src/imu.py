#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

msg = Float64MultiArray()
distance_x = Float64()
distance_y = Float64()
velo_x = Float64()
velo_y = Float64()

def callback(data):
    global msg
    global distance_x
    global distance_y
    global velo_x
    global velo_y

    msg = Float64MultiArray()

    rospy.loginfo(rospy.get_caller_id() + "\nAccel x: [{}]\n Accel y: [{}]\n Accel z: [{}]".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    rospy.loginfo(rospy.get_caller_id() + "\nAngular x: [{}]\n Angular y: [{}]\n Angular z: [{}]".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    
    accel_x = Float64()
    accel_x.data = float(0)
    accel_y = Float64()
    accel_y.data = float(0)

    if (data.linear_acceleration.x > 0.1 or data.linear_acceleration.x < -0.1):
        accel_x.data = data.linear_acceleration.x

    if (data.linear_acceleration.y > 0.1 or data.linear_acceleration.y < -0.1):
        accel_y.data = data.linear_acceleration.y

    velo_x.data = float(velo_x.data + (accel_x.data))
    velo_y.data = float(velo_y.data + (accel_y.data))

    distance_x.data = float(distance_x.data + velo_x.data)
    distance_y.data = float(distance_y.data + velo_y.data)

    msg.data = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z, velo_x.data, velo_y.data, distance_x.data, distance_y.data]

def imu():
    pub = rospy.Publisher('bender_navsensors_imu', Float64MultiArray, queue_size=10)
    
    rospy.init_node('bender_navsensors_imu', anonymous=False)

    rospy.Subscriber("/mavros/imu/data", Imu, callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
   
if __name__ == '__main__':
       try:
          imu()
       except rospy.ROSInterruptException:
           pass
