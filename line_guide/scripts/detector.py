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
    self.h_min = 100
    self.s_min = 60
    self.v_min = 30

    self.h_max = 150
    self.s_max = 255
    self.v_max = 225 
    self.showOptions()
  def hsvH_min(self, val):
    self.h_min = val

  def hsvS_min(self, val):
    self.s_min = val

  def hsvV_min(self, val):
    self.v_min = val

  def hsvH_max(self, val):
    self.h_max = val

  def hsvS_max(self, val):
    self.s_max = val

  def hsvV_max(self, val):
    self.v_max = val

  def handlecb(self, img):
    if self.busy:
      return
    self.busy = True
    self.detect(img)
    self.busy = False

  def detect(self, img):
    try:
      cv_img = self.bridge.imgmsg_to_cv(img, 'bgr8')
    except CvBridgeError, e:
      print e
      return
    cv_img = np.asarray(cv_img)
    cv_img = cv2.medianBlur(cv_img, 25)

    try:
      tmp = cv2.cv.fromarray(cv_img)
      self.color_pub.publish(self.bridge.cv_to_imgmsg(tmp, 'bgr8'))
    except CvBridgeError, e:
      print e
    
    # convert to HSV
    mask = cv2.cvtColor(cv_img, cv2.cv.CV_BGR2HSV)
    hsv_min = np.array([self.h_min, self.s_min, self.v_min])
    hsv_max = np.array([self.h_max, self.s_max, self.v_max])
    mask = cv2.inRange(mask, hsv_min, hsv_max) 
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (5,5))
    mask = cv2.erode(mask, element)

    try:
      tmp = cv2.cv.fromarray(mask)
      self.mask_pub.publish(self.bridge.cv_to_imgmsg(tmp, 'mono8'))
    except CvBridgeError, e:
      print e

    # show image
    # cv2.imshow(self.win, cv_img)

    # menu
    key = cv2.waitKey(10)
    """
    if key == ord('o'):
      self.showOptions()
    """

  def showOptions(self):
    cv2.namedWindow(self.opt)
    cv2.createTrackbar("H_min", self.opt, self.h_min, 180, self.hsvH_min)
    cv2.createTrackbar("S_min", self.opt, self.s_min, 255, self.hsvS_min)
    cv2.createTrackbar("V_min", self.opt, self.v_min, 255, self.hsvV_min)
    cv2.createTrackbar("H_max", self.opt, self.h_max, 180, self.hsvH_max)
    cv2.createTrackbar("S_max", self.opt, self.s_max, 255, self.hsvS_max)
    cv2.createTrackbar("V_max", self.opt, self.v_max, 255, self.hsvV_max)


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
