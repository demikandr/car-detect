#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from car_detect.msg import  TrackedObject, DimensionsWithCovariance

def track():
    pub = rospy.Publisher('chatter', TrackedObject, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    object = TrackedObject()
    object.dims.dimensions.x = 1
    object.dims.dimensions.y = 2
    object.dims.dimensions.z = 3
    object.pose.pose.orientation.x = 4
    object.pose.pose.orientation.y = 5
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(object)
        rate.sleep()

if  __name__ == '__main__':
    try:
        track()
    except rospy.ROSInterruptException:
        pass
