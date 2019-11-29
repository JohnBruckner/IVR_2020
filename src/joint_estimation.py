#!/usr/bin/env python

import roslib
import sys
import rospy
import json
import message_filters
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge import CvBridge, CvBridgeError


def callback(str1, str2):
    str1 = eval(str1.data)
    str2 = eval(str2.data)

    # ja1 will always be 0
    # ja2 from both will be a combination of x, y of the blue joint
    # ja3 corresponds to the angle x of joint 4
    # the rotation (angle z of base joint) can be determined by the overall orientation of the arm

    print('cam1')
    print(str1)
    print('cam2')
    print(str2)

    try:
        # est_pub.publish(str(jas))
        pass
    except CvBridgeError as e:
        print(e)


rospy.init_node("ja_estimation", anonymous=True)

cam1_sub = message_filters.Subscriber('/joints_pos_cam1', String)
cam2_sub = message_filters.Subscriber('/joints_pos_cam2', String)
est_pub = rospy.Publisher('joint_estimation', String, queue_size=1)

ts = message_filters.ApproximateTimeSynchronizer([cam1_sub, cam2_sub], 100, slop=0.3, allow_headerless=True)
ts.registerCallback(callback)

rospy.spin()
