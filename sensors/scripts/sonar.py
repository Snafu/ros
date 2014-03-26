#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
import math


class Sonar:
  def __init__(self):
    self.sub = rospy.Subscriber('/RosAria/sonar', PointCloud, self.callback)
    self.pub = rospy.Publisher('/Sensors/sonar', PointCloud)

  def callback(self, pc):
    for p in pc.points:
      dist = (math.sqrt(math.pow(p.x,2) + math.pow(p.y,2))*100.0)
      # subtract bot radius
      if dist > 30:
        dist -= 30
      else:
        dist = 0
      p.x = dist
      p.y = 0
      p.z = 0

    try:
      # publish distances in cm
      self.pub.publish(pc)
    except:
      pass


if __name__ == '__main__':
  rospy.init_node('sonar')
  son = Sonar()
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
