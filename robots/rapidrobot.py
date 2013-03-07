"""Writes ABB RAPID code and ftp-uploads it to the controller."""

import form

import baserobot


ABB_CONTROLLER_IP = "192.168.125.1"
ABB_CONTROLLER_PATH = "/HOME/open-abb-driver/tasks"
PATH_ROBOT1 = "robot1"
PATH_ROBOT1 = "robot2"
PATH_ROBOT1 = "robot3"


class Robot(baserobot.Robot):

	def __init__(self, ip=ABB_CONTROLLER_IP, numRobots=1):
		self.baserobot.Robot.__init__(self)
		if numRobots < 1 or numRobots > 3:
			print "Error: Only multi-move systems with 1 to 3 robots are supported."
			return
		self.ip = ip
		self.num_robots = numRobots
		self.rapid = [[],[],[]]  # each is a list of RAPID commands
		# defaults
	    self.currentSpeed = [100, 50, 0, 0]
	    self.currentZone = [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]  #z0
	    self.currentTool = [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]]
	    self.currentWobj = [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]]


	def upload(dest=ABB_CONTROLLER_PATH):
		"""FTP-upload RAPID code to ABB controller (IRC5)"""
		for rap in rapid:
			radpid_code = '\n'.join(rap)
			print "======="
			print radpid_code

		print "ftp upload not yet implemented"


	def move_linear(self, pose, robot=1):
		if len(pose) == 7: pose = [pose[0:3], pose[3:7]]
		self.rapid[robot-1].append("MoveL %s, %s, %s, %s \WObj:=%s ;") %
			(pose, self.currentSpeed, self.currentZone, self.currentTool, self.currentWobj)
