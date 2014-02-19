#!/usr/bin/python

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge, CvBridgeError


class Debug:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher('LineFollow/debug_image', Image)
    self.image_sub = rospy.Subscriber('LineFollow/color_image', Image, self.cbImage, None, 1)
    self.active_sub = rospy.Subscriber('LineFollow/active', Bool, self.cbActive)
    self.cog_sub = rospy.Subscriber('LineFollow/cog', Point, self.cbCog)
    self.sonar_sub = rospy.Subscriber('Sensors/sonar', PointCloud, self.cbSonar)

    self.sonar = None
    self.cog = None
    self.active = False
    
  def overlayInfo(self, img):
    font = cv2.FONT_HERSHEY_PLAIN
    fac = 1.0

    img_h, img_w, bpp = img.shape
    
    if self.sonar is not None:
      sonar_len = len(self.sonar)
      spacer = img_h/2/sonar_len
      for i, p in enumerate(self.sonar):
        cv2.line(img, (1,5 + spacer*i), (1 + int(p.x*img_w/2/100), 5 + spacer*i), (0,255,255), 2)
    
    org = (1, img_h-2)
    if self.active:
      cv2.putText(img, "Line follow ON", org, font, fac, (0,255,255))
    else:
      cv2.putText(img, "Line follow OFF", org, font, fac, (255,255,255))



  def cbActive(self, b):
    self.active = b.data

  def cbSonar(self, pc):
    self.sonar = pc.points

  def cbCog(self, p):
    self.cog = p

  def cbImage(self, img):
    try:
      color = self.bridge.imgmsg_to_cv(img, 'bgr8')
    except CvBridgeError, e:
      print e
      return
    color = np.asarray(color)
    
    # publish image
    self.overlayInfo(color)
    try:
      tmp = cv2.cv.fromarray(color)
      self.image_pub.publish(self.bridge.cv_to_imgmsg(tmp, 'bgr8'))
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('debug')
  d = Debug()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print 'Shutting down {0}'.format(rospy.getName())


if __name__ == '__main__':
  main(sys.argv)
