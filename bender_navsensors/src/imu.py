#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

msg = Float64MultiArray()
tolerance = 2  ## sets tolerance, or acceptable jump in data values between clock cycles
distance_x = Float64()
distance_y = Float64()
velo_x = Float64()
velo_y = Float64()
last_x_accel = None
last_y_accel = None

def callback(data):
    global msg
    global distance_x
    global distance_y
    global velo_x
    global velo_y
    
    ## data for previous x and y acceleration values
    global last_x
    global last_y
    ## acceptable gap between data per clock cycle
    global tolerance

    msg = Float64MultiArray()

    rospy.loginfo(rospy.get_caller_id() + "\nAccel x: [{}]\n Accel y: [{}]\n Accel z: [{}]".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    rospy.loginfo(rospy.get_caller_id() + "\nAngular x: [{}]\n Angular y: [{}]\n Angular z: [{}]".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))

    accel_x = Float64()
    accel_x.data = float(0)
    accel_y = Float64()
    accel_y.data = float(0)

    ## check if data for last acceleration values is uninitialized, if so, set it to 0
    if last_x_accel is None:
        last_x_accel = float(0)
    if last_y_accel is None:
        last_y_accel = float(0)


    ## only accepts data that is within a given tolerance level between clock cycles ie. if jump is too big, data is thrown out
    if (abs(data.linear_acceleration.x - last_x_accel) < tolerance):

        ## filters out acceleration data that is too small ie. within margin of accepted error
       if (data.linear_acceleration.x > 0.1 or data.linear_acceleration.x < -0.1):
          accel_x.data = data.linear_acceleration.x

    ## only accepts data that is within a given tolerance level between clock cycles ie. if jump is too big, data is thrown out
    if (abs(data.linear_acceleration.y - last_y_accel) < tolerance):
      
    ## filters out acceleration data that is too small ie. within margin of accepted error
        if (data.linear_acceleration.y > 0.1 or data.linear_acceleration.y < -0.1):
          accel_y.data = data.linear_acceleration.y


    ## sets acceleration data to be used for next cycle filtering
    last_x_accel = float(accel_x.data * 0.1)
    last_y_accel = float(accel_y.data * 0.1)

    velo_x.data = float(velo_x.data + (accel_x.data * 0.1))
    velo_y.data = float(velo_y.data + (accel_y.data * 0.1))

    distance_x.data = float(distance_x.data + velo_x.data * 0.1)
    distance_y.data = float(distance_y.data + velo_y.data * 0.1)

    rospy.loginfo(rospy.get_caller_id() + "Velo x: [{}]\nVelo y:[{}]\nDist x:[{}]\nDist y:[{}]".format(velo_x.data, velo_y.data, distance_x.data, distance_y.data))

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
