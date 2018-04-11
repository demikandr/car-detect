#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from car_detect.msg import  TrackedObject, DimensionsWithCovariance

def talker():
    pub = rospy.Publisher('chatter', TrackedObject, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(TrackedObject())
        rate.sleep()

if  __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
