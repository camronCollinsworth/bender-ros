#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix

msg = Float64MultiArray()

def callback(data):
    global msg

    msg = Float64MultiArray()

    rospy.loginfo(rospy.get_caller_id() + "\nLatitude: [{}]\n Longitude: [{}]\n".format(data.latitude, data.longitude))
    
    msg.data = [data.latitude, data.longitude]

def gps():
    pub = rospy.Publisher('bender_navsensors_gps', Float64MultiArray, queue_size=10)
    
    rospy.init_node('bender_navsensors_gps', anonymous=False)

    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
   
if __name__ == '__main__':
       try:
          gps()
       except rospy.ROSInterruptException:
           pass
