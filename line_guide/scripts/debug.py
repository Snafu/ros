#!/usr/bin/python

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, CompressedImage, PointCloud


class Debug:
  def __init__(self):
    self.image_pub = rospy.Publisher('LineFollow/debug_image/compressed', CompressedImage)
    self.image_sub = rospy.Subscriber('LineFollow/color_image/compressed', CompressedImage, self.cbImage, queue_size = 1)
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
    # decode image
    np_arr = np.fromstring(img.data, np.uint8)
    color = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    
    self.overlayInfo(color)
    
    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', color)[1]).tostring()
    # Publish new image
    self.image_pub.publish(msg)

def main(args):
  rospy.init_node('debug')
  d = Debug()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print 'Shutting down {0}'.format(rospy.getName())


if __name__ == '__main__':
  main(sys.argv)
