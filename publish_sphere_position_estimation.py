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

    x = str1['x']
    y = str2['y']
    z = (str1['z'] + str2['z']) / 2

    coords = {'x': x, 'y': y, 'z': z}

    try:
        pos_pub.publish(str(coords))
    except CvBridgeError as e:
        print(e)


rospy.init_node("sphere_position_estimation", anonymous=True)

xz_sub = message_filters.Subscriber('/sphere_xz', String)
yz_sub = message_filters.Subscriber('/sphere_yz', String)
pos_pub = rospy.Publisher("sphere_position", String, queue_size=1)

ts = message_filters.ApproximateTimeSynchronizer([xz_sub, yz_sub], 100, slop=0.1, allow_headerless=True)
ts.registerCallback(callback)

rospy.spin()
