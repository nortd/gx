"""Connects to an ABB robot through the open-abb-driver.

The open-abb-driver is a RAPID program that runs on the ABB
controller (IRC5) and opens a socket connection. through
this socket it receives commands that get mapped to RAPID
command pretty much one-to-one.
"""

import form
import oad

import baserobot


ABB_CONTROLLER_IP = "192.168.125.1"


class Robot(baserobot.Robot):

	def __init__(self, ip=ABB_CONTROLLER_IP, port=5000):
		baserobot.Robot.__init__(self)
		self.oadrobot = oad.robot.Robot(IP=ip, PORT=port)

	def connect(self):
		self.oadrobot.connect()

	def move_linear(self, pose, robot=1):
		"""
		pose: either [location_pt, orientaion_quat] or [px,py,pz,q1,q2,q3,q4]
		"""
		self.oadrobot.setCartesian(pose)

			