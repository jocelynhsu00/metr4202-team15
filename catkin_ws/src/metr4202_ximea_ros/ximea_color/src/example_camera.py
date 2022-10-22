#!/usr/bin/env python

import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from cv_bridge import CvBridge, CvBridgeError

img = None

class CameraViewer:

  def __init__(self, serial):
    self.bridge = CvBridge()
    self.serial = serial
    self.image_sub = rospy.Subscriber(f"/ximea_ros/ximea_{self.serial}/image_raw", Image, self.callback)
    self.color_pub = rospy.Publisher("/colour_string", String, queue_size=10)

  def callback(self,data):
    global img
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
    color = ColorRGBA()
    color.r = bgr[2]
    color.g = bgr[1]
    color.b = bgr[0]
    print(color)

    if color.r >= 250 and color.g >=215:
      colour = 'yellow'
    elif color.r >= 250:
      colour = 'red'
    elif color.g >= 195:
      colour = 'green'
    elif color.b >= 140:
      colour = 'blue'
    else:
      colour = None
    viewer.color_pub.publish(colour)



if __name__ == '__main__':  
  rospy.init_node('image_node', anonymous=True)
  viewer = CameraViewer('32704451')
  try:
    while not rospy.is_shutdown():
        if img is not None:
            cv2.imshow("Image", img)
            cv2.waitKey(1)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()