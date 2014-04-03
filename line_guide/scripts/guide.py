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
    self.backupDuration = 1.0
    self.backup = False

    # steering PID settings
    self.s_Kp = 0.5
    self.s_Ki = 0.3
    self.s_Kd = 0.1
    self.s_err = 0
    self.s_err_prev = 0
    self.s_int = 0
    self.s_der = 0
    self.s_max = 0.6

    # throttle PID settings
    self.t_Kp = 0.2
    self.t_Ki = 0.5
    self.t_Kd = 0.1
    self.t_err = 0
    self.t_err_prev = 0
    self.t_int = 0
    self.t_der = 0
    self.t_max = 1.0
    self.t_factor = 0.1

    self.lastupdate = rospy.rostime.get_time()
    
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
    """
    for i,p in enumerate(pc.points):
      vector[0] += p.x * -math.cos(math.pi*i/length)
      vector[1] += p.x * math.sin(math.pi*i/length)
    val = sum([math.pow(x,2) for x in vector])
    phi = 180*math.atan2(vector[1], vector[0])/math.pi
    #rospy.loginfo("{0:.2f} {1:.2f}".format(val, phi))
    """  
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

    time = rospy.get_time()
    dt = time - self.lastupdate
    # reset if dt too large
    if dt > 1:
      dt = 0.1
      s_int = 0
      s_err_prev = 0
      t_int = 0
      t_err_prev = 0

    s_output = 0
    t_output = 0

    if self.active:
      if not -0.01 <= self.cog.x <= 0.01:
        # steering PID
        self.s_err = 0 - self.cog.x
        self.s_int = self.s_int + self.s_err*dt
        self.s_der = (self.s_err - self.s_err_prev)/dt
        s_output = self.s_Kp*self.s_err + self.s_Ki*self.s_int + self.s_Kd*self.s_der
        self.s_err_prev = self.s_err

        s_output = -self.s_max if s_output < -self.s_max else self.s_max if s_output > self.s_max else s_output
        t.angular.z = s_output # * self.rotAdjust

        self.moving = True

      if self.cog.y > 0.01:
        # throttle PID (not really necessary, always max throttle)
        self.t_err = self.cog.y
        self.t_int = self.t_int + self.t_err*dt
        self.t_der = (self.t_err - self.t_err_prev)/dt
        t_output = self.t_Kp*self.t_err + self.t_Ki*self.t_int + self.t_Kd*self.t_der
        self.t_err_prev = self.t_err

        # limit max throttle
        t_output = self.t_max if t_output > self.t_max else t_output

        t.linear.x = t_output * self.t_factor

        self.moving = True

    self.lastupdate = time

    # check if we should drive automatically
    if self.moving or (self.moving_last and not self.moving):
      rospy.loginfo("{0:4.2f},{1:4.2f} @ {2:.3f}".format(s_output, t_output, dt))
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
