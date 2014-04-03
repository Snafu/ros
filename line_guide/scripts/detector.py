#!/usr/bin/python

import roslib
import sys
import rospy
import cv2
import numpy as np
from math import copysign
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, CompressedImage

class Detector:
  def __init__(self):
    self.busy = False
    self.opt = 'Options'
    self.image_sub = rospy.Subscriber('camera/image_raw/compressed', CompressedImage, self.handlecb, queue_size = 1)
    self.color_pub = rospy.Publisher('LineFollow/color_image/compressed', CompressedImage)
    self.mask_pub = rospy.Publisher('LineFollow/mask_image/compressed', CompressedImage)
    self.follow_pub = rospy.Publisher('LineFollow/follow', Bool)
    self.cmd_vel = rospy.Publisher('LineFollow/cmd_vel', Twist)
    self.cog = rospy.Publisher('LineFollow/cog', Point)

    self.hsv_min = [0, 100, 50]
    self.hsv_max = [10, 255, 255]

    self.follow_line = False
    self.move = False
    self.last_move = False
    self.rotAdjust = 0.1
    self.accAdjust = 0.2
    self.cxAvg = np.zeros(5)
    self.cyAvg = np.zeros(5)
    self.cogX_last = -1
    self.cogY_last = -1

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
    # decode image
    np_arr = np.fromstring(img.data, np.uint8)
    color = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

    color = cv2.medianBlur(color, 7)
    img_h, img_w, bpp = color.shape
    # clip image to bottom third
    color = color[2*img_h/3:, :]
    img_h, img_w, bpp = color.shape
    
    # convert to HSV
    mask = cv2.cvtColor(color, cv2.cv.CV_BGR2HSV)
    hsv_min2 = list(self.hsv_min)
    hsv_min2[0] = 180 - self.hsv_max[0]
    hsv_max2 = list(self.hsv_max)
    hsv_max2[0] = 180 - self.hsv_min[0]
    mask2 = cv2.inRange(mask, np.array(hsv_min2), np.array(hsv_max2))
    mask = cv2.inRange(mask, np.array(self.hsv_min), np.array(self.hsv_max))
    mask = cv2.bitwise_or(mask, mask2)

    # erode & dilate
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
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
    cogX = self.cogX_last
    cogY = self.cogY_last
    if max_cont is not None:
      M = cv2.moments(max_cont)
      ccx, ccy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
      self.cxAvg, cx = self.runningAvg(self.cxAvg, ccx)
      self.cyAvg, cy = self.runningAvg(self.cyAvg, ccy)
      cogX, cogY = float(cx)/img_w - 0.5, float(img_h - cy)/img_h

      cv2.circle(color, (ccx,ccy), 3, (0,0,255), -1)
      cv2.circle(color, (int(cx),int(cy)), 5, (255,0,0), -1)
      cv2.drawContours(color, [max_cont], -1, (0,255,0), 5)
    else:
      cogX, cogY = 0,0

    if True or (cogX, cogY) != (self.cogX_last, self.cogY_last):
      self.cogX_last = cogX
      self.cogY_last = cogY
      # rospy.loginfo("Center of gravity at %.2f, %.2f", cogX, cogY)
      self.cog.publish(Point(cogX,cogY,0))

    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', color)[1]).tostring()
    # Publish new image
    self.color_pub.publish(msg)

    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', mask)[1]).tostring()
    # Publish new image
    self.mask_pub.publish(msg)

    # menu
    key = cv2.waitKey(10) % 0x100
    if key == ord('k'):
      self.follow_line = not self.follow_line
      self.follow_pub.publish(self.follow_line)


  def runningAvg(self, l, i): 
    l = np.roll(l, 1)
    l[0] = i 
    return l, np.convolve(l, np.ones(l.shape)/l.shape[0], 'valid')[0]


  def showOptions(self):
    cv2.namedWindow(self.opt)
    cv2.createTrackbar("H_min", self.opt, self.hsv_min[0], 180, self.hsvH_min)
    cv2.createTrackbar("S_min", self.opt, self.hsv_min[1], 255, self.hsvS_min)
    cv2.createTrackbar("V_min", self.opt, self.hsv_min[2], 255, self.hsvV_min)
    cv2.createTrackbar("H_max", self.opt, self.hsv_max[0], 180, self.hsvH_max)
    cv2.createTrackbar("S_max", self.opt, self.hsv_max[1], 255, self.hsvS_max)
    cv2.createTrackbar("V_max", self.opt, self.hsv_max[2], 255, self.hsvV_max)


def main(args):
  rospy.init_node('detector')
  d = Detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print 'Shutting down {0}'.format(rospy.getName())
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
