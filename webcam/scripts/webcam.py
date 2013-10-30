#!/usr/bin/python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class webcam:
  def __init__(self):
    self.image_pub = rospy.Publisher("webcam_image",Image)
    self.win = "Webcam"
    cv2.namedWindow(self.win)
    self.c = cv2.VideoCapture(0)
    self.bridge = CvBridge()
    self.alive = True

  def grab(self):
      if not self.alive:
        return
      img = self.c.read()[1]
      #cv2.imshow(self.win, img)
      #key = cv2.waitKey(100)
      img = cv2.cv.fromarray(img)

      try:
        self.image_pub.publish(self.bridge.cv_to_imgmsg(img, "bgr8"))
      except CvBridgeError, e:
        print e

      #if key == 27:
      #  rospy.signal_shutdown("User pressed ESC")

  def shutdown(self):
    print "Shutting down {0}".format(rospy.get_name())
    self.alive = False
    cv2.destroyWindow(self.win)


def main(args):
  wc = webcam()
  rospy.init_node('webcam')
  rospy.on_shutdown(wc.shutdown)

  try:
    while not rospy.is_shutdown():
      wc.grab()
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
  main(sys.argv)
