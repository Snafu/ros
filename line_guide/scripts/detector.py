#!/usr/bin/python

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Detector:
  def __init__(self):
    self.busy = False
    self.win = 'Line Detector'
    self.opt = 'Options'
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('webcam_image', Image, self.handlecb)
    self.color_pub = rospy.Publisher('color_image', Image)
    self.mask_pub = rospy.Publisher('mask_image', Image)

    #cv2.namedWindow(self.win)
    self.hsv_min = [100, 60, 30]
    self.hsv_max = [150, 255, 255]

    self.showOptions()
  def hsvH_min(self, val):
    self.hsv_min[0] = val

  def hsvS_min(self, val):
    self.hsv_min[1] = val

  def hsvV_min(self, val):
    self.hsv_min[2] = val

  def hsvH_max(self, val):
    self.hsv_max[0] = val

  def hsvS_max(self, val):
    self.hsv_max[1] = val

  def hsvV_max(self, val):
    self.hsv_max[2] = val

  def handlecb(self, img):
    if self.busy:
      return
    self.busy = True
    self.detect(img)
    self.busy = False

  def detect(self, img):
    try:
      color = self.bridge.imgmsg_to_cv(img, 'bgr8')
    except CvBridgeError, e:
      print e
      return
    color = np.asarray(color)
    color = cv2.medianBlur(color, 9)
    
    # convert to HSV
    mask = cv2.cvtColor(color, cv2.cv.CV_BGR2HSV)
    mask = cv2.inRange(mask, np.array(self.hsv_min), np.array(self.hsv_max))

    # erode & dilate
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
    mask = cv2.erode(mask, element)
    mask = cv2.dilate(mask, element)

    # find contour with largest area
    contours = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
    max_area = 0
    max_cont = None
    for c in contours:
      area = cv2.contourArea(c)
      if area > max_area:
        max_area = area
        max_cont = c

    # calculate center of gravity position
    if max_cont is not None:
      M = cv2.moments(max_cont)
      cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
      cv2.circle(color, (cx,cy), 5, (0,0,255), -1)
      cv2.drawContours(color, [max_cont], -1, (0,255,0), 5)

    try:
      tmp = cv2.cv.fromarray(mask)
      self.mask_pub.publish(self.bridge.cv_to_imgmsg(tmp, 'mono8'))
    except CvBridgeError, e:
      print e

    try:
      tmp = cv2.cv.fromarray(color)
      self.color_pub.publish(self.bridge.cv_to_imgmsg(tmp, 'bgr8'))
    except CvBridgeError, e:
      print e

    # show image
    # cv2.imshow(self.win, color)

    # menu
    key = cv2.waitKey(10)
    """
    if key == ord('o'):
      self.showOptions()
    """

  def showOptions(self):
    cv2.namedWindow(self.opt)
    cv2.createTrackbar("H_min", self.opt, self.hsv_min[0], 180, self.hsvH_min)
    cv2.createTrackbar("S_min", self.opt, self.hsv_min[1], 255, self.hsvS_min)
    cv2.createTrackbar("V_min", self.opt, self.hsv_min[2], 255, self.hsvV_min)
    cv2.createTrackbar("H_max", self.opt, self.hsv_max[0], 180, self.hsvH_max)
    cv2.createTrackbar("S_max", self.opt, self.hsv_max[1], 255, self.hsvS_max)
    cv2.createTrackbar("V_max", self.opt, self.hsv_max[2], 255, self.hsvV_max)


def main(args):
  d = Detector()
  rospy.init_node('detector')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print 'Shutting down {0}'.format(rospy.getName())
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
