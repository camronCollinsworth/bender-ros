#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

msg = Float64MultiArray()
tolerance = 1   ## sets tolerance, or acceptable jump in data values between clock cycles
distance_x = Float64()
distance_y = Float64()
velo_x = Float64()
velo_y = Float64()
last_x_accel = None
last_y_accel = None
accel_x_list = []
accel_y_list = []
velo_x_list = []
velo_y_list = []

def callback(data):
    global msg
    global distance_x
    global distance_y
    global velo_x
    global velo_y
    global accel_x_list
    global accel_y_list
    global velo_x_list
    global velo_y_list
    
    ## data for previous x and y acceleration values
    global last_x_accel
    global last_y_accel

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
          
    else:
        accel_x.data = last_x_accel

    ## only accepts data that is within a given tolerance level between clock cycles ie. if jump is too big, data is thrown out
    if (abs(data.linear_acceleration.y - last_y_accel) < tolerance):
      
    ## filters out acceleration data that is too small ie. within margin of accepted error
        if (data.linear_acceleration.y > 0.1 or data.linear_acceleration.y < -0.1):
          accel_y.data = data.linear_acceleration.y

    else:
        accel_y.data = last_y_accel

    ## sets acceleration data to be used for next cycle filtering
    last_x_accel = accel_x.data
    last_y_accel = accel_y.data

    ## add acceleration and velocity data to arrays
    accel_x_list.append(accel_x.data)
    accel_y_list.append(accel_y.data)

    velo_x.data = float(accumulator(accel_x_list))
    velo_y.data = float(accumulator(accel_y_list))

    velo_x_list.append(velo_x.data)
    velo_y_list.append(velo_y.data)

    distance_x.data = float(accumulator(velo_x_list))
    distance_y.data = float(accumulator(velo_y_list))


    ## shorten the size of lists to prevent getting too large
    if len(accel_x_list) > 999:
       accel_x_list = shortenList(accel_x_list)
    if len(accel_y_list) > 999:
       accel_y_list = shortenList(accel_y_list)
    if len(velo_x_list) > 999:
        velo_x_list = shortenList(velo_x_list)
    if len(velo_y_list) > 999:
        velo_y_list = shortenList(velo_y_list)

    rospy.loginfo(rospy.get_caller_id() + "\nVelo x: [{}]\nVelo y:[{}]\nDist x:[{}]\nDist y:[{}]".format(velo_x.data, velo_y.data, distance_x.data, distance_y.data))

    msg.data = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z, velo_x.data, velo_y.data, distance_x.data, distance_y.data]

def imu():
    pub = rospy.Publisher('bender_navsensors_imu', Float64MultiArray, queue_size=10)
    
    rospy.init_node('bender_navsensors_imu', anonymous=False)

    rospy.Subscriber("/mavros/imu/data", Imu, callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

def accumulator(array):
    sum = 0
    for i in array:
        sum += i * 0.1

    return sum


## takes an array and returns a new array with one element that is the sum of the given array
def shortenList(array):
    sum = 0
    newArray = []

    for i in array:
        sum += i

    newArray.append(sum)    
    return newArray

   
if __name__ == '__main__':
       try:
          imu()
       except rospy.ROSInterruptException:
           pass
