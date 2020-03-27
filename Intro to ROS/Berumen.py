#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def Berumen():
    pub = rospy.Publisher('/homework0', String, queue_size=10)
    rospy.init_node('Berumen', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "Robotics is fun! %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def hw_zero(data):
         rospy.loginfo(rospy.get_caller_id() + "Homework0 message: %s", data.data) # This is the callback function

def listener():

    # Anonymous?
    rospy.init_node('Berumen', anonymous=False)

    rospy.Subscriber("/homework0", String, hw_zero)
    #rospy.loginfo(hello_str)
    # keeps from exiting until node stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
