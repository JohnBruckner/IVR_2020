#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  def detect_orange(self, image):

    #  msk = cv2.inRange(image, (70, 110, 140), (80, 190, 225))
    #  output = cv2.bitwise_and(image, image, mask=msk)

    # Isolate the orange colour in the image as a binary image
    
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lwr = np.asarray([5, 10, 20], dtype='uint8')
    uppr = np.asarray([25, 255, 255], dtype='uint8')

    msk = cv2.inRange(img, lwr, uppr)  

    return msk

  def detect_circle(self, image):
    _, contours, _ = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
      approx = cv2.approxPolyDP(c, 0.01*cv2.arcLength(c, True), True)
      area = cv2.contourArea(c)
      
      if len(approx) > 8:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        return cX, cY

  def detect_rectangle(self, image):
    _, contours, _ = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
      approx = cv2.approxPolyDP(c, 0.01*cv2.arcLength(c, True), True)
      area = cv2.contourArea(c)
      
      if len(approx) < 8:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        return cX, cY
    
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    self.cv_image1 = self.detect_orange(self.cv_image1)

    im1 = cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)

    print(self.detect_circle(self.cv_image1))


    # Publish the results
    # try: 
      # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    # except CvBridgeError as e:
      # print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


