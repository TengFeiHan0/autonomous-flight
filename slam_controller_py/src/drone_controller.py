#!/usr/bin/env python

# Basic drone control functionality + state tracking
# adapted from Mike Hammer's tutorials

import rospy

from geometry_msgs.msg    import Twist
from std_msgs.msg         import Empty
from ardrone_autonomy.msg import Navdata
from drone_status         import DroneStatus


class BasicDroneController(object):
	COMMAND_PERIOD = 100 #ms
	def __init__(self):
		self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata, queue_size = 20)

		self.pubLand    = rospy.Publisher('/ardrone/land'    , Empty, queue_size = 20)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff' , Empty, queue_size = 20)
		self.pubReset   = rospy.Publisher('/ardrone/reset'   , Empty, queue_size = 20)
		self.pubCommand = rospy.Publisher('/cmd_vel'         , Twist, queue_size = 20)

		self.status     = DroneStatus.Unknown
		self.command    = Twist()
		dur = self.COMMAND_PERIOD / 1000.0
		self.commandTimer = rospy.Timer(rospy.Duration(dur), self.SendCommand)

		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		self.status = navdata.state

	def SendTakeoff(self):
		if(self.status == DroneStatus.Landed): # double takeoff forbidden
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		self.pubLand.publish(Empty()) # land regardless of state

	def SendEmergency(self):
		self.pubReset.publish(Empty()) # emergency regardless of state

	def SetCommand(self, roll = 0, pitch = 0, yaw_velocity = 0, z_velocity = 0):
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event): # called on timer
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)

