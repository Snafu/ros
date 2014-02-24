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
from ROSARIA.msg import BumperState


class Guide:
  def __init__(self):
    self.bumper_sub = rospy.Subscriber('RosAria/bumper_state', BumperState, self.cbBumper, queue_size=1)
    self.cog_sub = rospy.Subscriber('LineFollow/cog', Point, self.cbCog, queue_size=1)
    self.sonar_sub = rospy.Subscriber('Sensors/sonar', PointCloud, self.cbSonar)
    self.active_sub = rospy.Subscriber('LineFollow/follow', Bool, self.cbFollow)
    self.active_pub = rospy.Publisher('LineFollow/active', Bool)
    self.cmd_vel = rospy.Publisher('RosAria/cmd_vel', Twist)

    self.sonar = None
    self.cog = Point(0,0,0)
    self.active = False
    self.active_last = False
    self.moving = False
    self.moving_last = False
    self.rotAdjust = 0.2
    self.accAdjust = 0.1
    self.backupDuration = 1.0
    self.backup = False
    
  def cbBumper(self, bump):
    mdir = 0
    if any(bump.front_bumpers):
      mdir = -0.5*self.accAdjust
    if any(bump.rear_bumpers):
      mdir =  0.5*self.accAdjust

    if mdir != 0: 
      self.backup = True
      t = Twist()
      t.linear.x = mdir
      self.move(t)
      rospy.sleep(self.backupDuration)
      self.move()
      self.backup = False

  def cbSonar(self, pc):
    self.sonar = pc.points
    vector = [0,0]
    length = len(pc.points)-1
    scale = max([p.x for p in pc.points])*length
    #rospy.loginfo("#####################")
    for i,p in enumerate(pc.points):
      vector[0] += p.x * -math.cos(math.pi*i/length)
      vector[1] += p.x * math.sin(math.pi*i/length)
      #rospy.loginfo("{0:.2f} {1:.2f} {2:.2f}".format(p.x, math.cos(math.pi*i/length), math.sin(math.pi*i/length)))
    #rospy.loginfo("{0:.2f} {1:.2f}".format(vector[0], 180-(180*vector[1]/math.pi)))
    val = sum([math.pow(x,2) for x in vector])
    phi = 180*math.atan2(vector[1], vector[0])/math.pi
    rospy.loginfo("{0:.2f} {1:.2f}".format(val, phi))
      
    self.update()

  def cbCog(self, p):
    self.cog = p
    self.update()

  def cbFollow(self, b):
    self.active_last = self.active
    self.active = b.data
    self.update()

  def update(self):
    if self.backup:
      return
    t = Twist()
    self.moving = False
    if self.active:
      if not -0.02 <= self.cog.x <= 0.02:
        t.angular.z = math.copysign(0.5, self.cog.x) * -1 * self.rotAdjust
        self.moving = True

      if self.cog.y > 0.2:
        t.linear.x = max(self.cog.y, 0.5) * self.accAdjust
        self.moving = True

    # check if we should drive automatically
    if self.moving or (self.moving_last and not self.moving):
      self.move(t)    
    self.moving_last = self.moving

    if bool(self.active) != bool(self.active_last):
      try:
        self.active_pub.publish(self.active)
      except:
        pass

  def move(self, t = Twist()):
    try:
      self.cmd_vel.publish(t)
    except:
      pass


def main(args):
  rospy.init_node('guide')
  d = Guide()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print 'Shutting down {0}'.format(rospy.getName())


if __name__ == '__main__':
  main(sys.argv)
