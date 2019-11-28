#!/usr/bin/env python
from attr import dataclass

import roslib
import sys
import rospy
import cv2
import numpy as np
from rospy import numpy_msg
from std_msgs import msg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class estimate_xz:

    # Defines publisher and subscriber
    def __init__(self, cam):
        self.cam = cam

        # initialize the node named image_processing
        rospy.init_node("sphehre_position_estimation_xz", anonymous=True)

        self.h = msg.Header()
        self.h.stamp = rospy.Time.now()

        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("sphere_xz", String, queue_size=1)       # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def detect_orange(self, image):

        #  msk = cv2.inRange(image, (70, 110, 140), (80, 190, 225))
        #  output = cv2.bitwise_and(image, image, mask=msk)

        # Isolate the orange colour in the image as a binary image

        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lwr = np.asarray([5, 10, 20], dtype="uint8")
        uppr = np.asarray([25, 255, 255], dtype="uint8")

        msk = cv2.inRange(img, lwr, uppr)

        return msk

    def detect_circle(self, image):
        _, contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            approx = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True)
            area = cv2.contourArea(c)

            if len(approx) > 8:
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                return cX, cY

    def detect_rectangle(self, image):
        _, contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            approx = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True)
            area = cv2.contourArea(c)

            if len(approx) < 8:
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                return cX, cY

    # Recieve data from camera 1, process it, and publish
    def callback2(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)

        mask = self.detect_orange(self.cv_image1)

        im1 = cv2.imshow("window2", self.cv_image1)
        cv2.waitKey(1)

        cx, cy = self.detect_circle(mask)

        y_coord = self.cam.locate_obj(cx, cy, self.cv_image1)

        coords = "X: " + str(y_coord[0]) + \
                 "Z: " + str(-y_coord[1])

        coords_dict = {'x': y_coord[0], 'z': -y_coord[1]}

        # Publish the results
        try:
            self.image_pub1.publish(str(coords_dict))
        except CvBridgeError as e:
            print(e)


class camera:
    def __init__(self):
        self.cx = 400
        self.cy = 550
        self.f = 476.7

    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_green(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 3 / np.sqrt(dist)

    def locate_obj(self, px, py, image):
        conversion = self.pixel2meter(image)
        px = conversion * (px - self.cx)
        py = conversion * (py - self.cy)

        return px, py


def main(args):
    c = camera()
    ic = estimate_xz(c)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == "__main__":
    main(sys.argv)
