"""Writes ABB RAPID code and ftp-uploads it to the controller."""

import os
import tempfile
import ftplib
import StringIO

from gx import settings
from gx.libs import form
from gx.libs.robots import baserobot


ABB_CONTROLLER_IP = "192.168.125.1"
ABB_CONTROLLER_FTP_PORT = 21
# ABB_CONTROLLER_IP = "localhost"
# ABB_CONTROLLER_FTP_PORT = 2121

# typically /HOME/gx/tasks/T_ROBx.mod
ABB_CONTROLLER_TASKS_ROOT = os.path.join(settings.ABB_CONTROLLER_PATH, 'tasks')
ABB_CONTROLLER_TASK_NAMES = ['T_ROB1', 'T_ROB2', 'T_ROB3']
ABB_CONTROLLER_TASK_PATHS = [os.path.join(ABB_CONTROLLER_TASKS_ROOT, 'T_ROB1'),
							 os.path.join(ABB_CONTROLLER_TASKS_ROOT, 'T_ROB2'),
							 os.path.join(ABB_CONTROLLER_TASKS_ROOT, 'T_ROB3')]

temppath = tempfile.gettempdir()
local_output = os.path.join(temppath, "gx")
LOCAL_OUTPUT_PATH = [os.path.join(local_output, "T_ROB1.mod"),
				     os.path.join(local_output, "T_ROB2.mod"),
					 os.path.join(local_output, "T_ROB3.mod")]
# LOCAL_OUTPUT_PATH = [os.path.expanduser("~/temp/robot1.mod"),
# 				     os.path.expanduser("~/temp/robot2.mod"),
# 					 os.path.expanduser("~/temp/robot3.mod")]


class Robot(baserobot.Robot):

	def __init__(self, ip=ABB_CONTROLLER_IP, numRobots=1):
		# baserobot.Robot.__init__(self)
		if numRobots < 1 or numRobots > 3:
			print "Error: Only systems with 1 to 3 robots are supported."
			return
		self.ip = ip
		self.num_robots = numRobots
		self.vars = [[],[],[]]
		self.main_head = [[],[],[]]
		self.main = [[],[],[]]
		self.tcounter = 0  # robtarget counter
		### defaults
		# speed in mm/s an deg/sec
		# [linear speed, rotational speed, linear external, rot ext]
		# self.currentSpeed = [100, 50, 0, 0]
		self.currentSpeed = "v100"
		self.currentZone = "z0"
		# self.currentTool = [True,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]]
		self.currentTool = "tool0"
		self.currentWobj = "wobj0"  # using default where user and work frame is the same as world
		self.confData = [0,0,0,0]  # Specifies robot axes angles
		# self.confData = [1,0,0,0]  # Specifies robot axes angles
		self.externalAxes = [9e9, 9e9, 9e9, 9e9, 9e9, 9e9]
		# self.externalAxes = "externalAxis0"


	def upload(self, dest=ABB_CONTROLLER_TASKS_ROOT):
		"""FTP-upload RAPID code to ABB controller (IRC5)"""
		ftp = ftplib.FTP()
		ftp.connect(ABB_CONTROLLER_IP, ABB_CONTROLLER_FTP_PORT)
		ftp.login('gx', 'gx')
		try:
			ftp.cwd(ABB_CONTROLLER_TASKS_ROOT)
		except ftplib.error_perm:
			print("ERROR: Need directory on the ABB controller: %s" %
				  (ABB_CONTROLLER_TASKS_ROOT))
			print("INFO: host:%s:%s" %
				  (ABB_CONTROLLER_IP, ABB_CONTROLLER_FTP_PORT))
			raise ftplib.error_perm

		for n in range(self.num_robots):
			mod_file = ABB_CONTROLLER_TASK_NAMES[n] + '.mod'
			pgf_file = ABB_CONTROLLER_TASK_NAMES[n] + '.pgf'
			code = self._compile(n+1)
			fp = StringIO.StringIO(code)
			ftp.storbinary('STOR %s' % (mod_file), fp)
			code_pgf = self._compile_pgf_file(n+1)
			fp_pgf = StringIO.StringIO(code_pgf)
			ftp.storbinary('STOR %s' % (pgf_file), fp_pgf)
			print("ftp upload done: %s, %s" % (mod_file, pgf_file))

		ftp.quit()


	def reset():
		self.vars = [[],[],[]]
		self.main_head = [[],[],[]]
		self.main = [[],[],[]]


	def dump(self, robot=1):
		code = self._compile(robot)
		print(code)


	def filedump(self, filename=None, robot=1):
		if not filename:
			filename = LOCAL_OUTPUT_PATH[robot-1]
		filedir = os.path.split(filename)[0]
		if not os.path.exists(filedir):
			os.makedirs(filedir)
		code = self._compile(robot)
		with open(filename, 'w') as f:
			f.write(code)
			print("INFO: RAPID witten to: %s" % (filename))
		return filename


	def _compile(self, robot):
		"""Assemble RAPID instructions."""
		self.wrist_singularity(reorient=True, robot=robot)
		# self.monitor_conf_linear(on=False)
		# self.monitor_conf_joint(on=False)
		code = []
		code.append("MODULE MainModule")
		# code.append("!Find the current external axis values so they don't move when we start")
		# code.append("VAR extjoint externalAxis0;")
		# code.append("jointsTarget := CJointT();")
		# code.append("externalAxis0 := jointsTarget.extax;")
		code.extend(self.vars[robot-1])
		code.append('PERS tasks task_list{3} := [["T_ROB1"],["T_ROB2"],["T_ROB3"]];')
		code.append("VAR syncident SyncOn;")
		code.append("VAR syncident SyncOff;")
		code.append("PROC Main()")
		code.extend(self.main_head[robot-1])
		code.extend(self.main[robot-1])
		code.append("MoveAbsJ [[0,0,0,0,0,0],[0,0,0,0,0,0]], %s, %s, %s \Wobj:=%s;" % 
			         (self.currentSpeed, self.currentZone, self.currentTool, self.currentWobj))
		code.append("ENDPROC")
		code.append("ENDMODULE")
		return '\n'.join(code)


	def _compile_pgf_file(self, robot):
		"""Assemble RAPID pgf xml file."""
		code = []
		code.append('<?xml version="1.0" encoding="ISO-8859-1" ?>')
		code.append("<Program>")
		code.append("<Module>%s.mod</Module>" % (ABB_CONTROLLER_TASK_NAMES[robot-1]))
		code.append("</Program>")
		return '\n'.join(code)


	def move_linear(self, pos, orient, robot=1, duration=None, setSignal=None):
		"""Linear move to position and orientation.

		This is an inverse kinematics move and maps to a RAPID MoveL call. 
		It typically takes a robtarget, speed, zone, tool data, and work 
		coordinate system. A robtarget is primarily a position and orientation
		but also includes confData, and positions for external axes.

		duration: in seconds, if specified the controller will overwrite 
		the speed setting and it will try to reach the pose in that time.

		setSignal: tuple with (name, True|False), if specified signal will
		           be changed when closest to robtarget

		For details see: "RAPID Reference Instructions, Functions, and Data"

		ABB robots can be set up as a "Multi-Move" system. This allows them
		to move in sync. For this to work the code section needs to be in a
		SyncMove block and the number of moves within this section need to
		be the same on all robots.

		This means that start_sync_move() and stop_sync_move() needs to be
		called on all robots involved in the synced motion and the waypoints
		added within need to be the same.

		MoveL PtTarg15 \ID:=15,PullFoam,z1,tube \Wobj:=Wobj0;
		"""
		command = "MoveL"
		tname = "pose" + str(self.tcounter)
		self.tcounter += 1
		self._add_robtarget_var(tname, pos, orient, self.confData, self.externalAxes)
		if duration:
			duration = "\T:=%s" % (duration)
		else:
			duration = ""
		if setSignal:
			command = "MoveLDO"
			setSignal = ", %s,%s" % (setSignal[0], int(setSignal[1]))
		else:
			setSignal = ""
		self.main[robot-1].append("%s %s, %s %s, %s, %s \WObj:=%s%s;" % \
			(command, tname, self.currentSpeed, duration, self.currentZone, 
			 self.currentTool, self.currentWobj, setSignal))


	def move_joint(self, pos, orient, robot=1, duration=None, setSignal=None):
		"""Linear move to position and orientation, joint interpolated.

		This is a inverse kinematics move that maps to a RAPID MoveJ call.
		It typically takes a robtarget, speed, zone, tool data, and work 
		coordinate system. A robtarget is primarily a position and orientation 
		but also includes confData, and positions for external axes.

		duration: in seconds, if specified the controller will overwrite 
		the speed setting and it will try to reach the pose in that time.

		setSignal: tuple with (name, True|False), if specified signal will
		           be changed when closest to robtarget

		For details see: "RAPID Reference Instructions, Functions, and Data"
		MoveJ start, v2000, z40, grip3 \WObj:=fixture;
		MoveJ [\Conc] robtarget [\ID] Speed [\V] | [\T] Zone [\Z] [\Inpos] Tool [\WObj]
		"""
		command = "MoveJ"
		tname = "pose" + str(self.tcounter)
		self.tcounter += 1
		self._add_robtarget_var(tname, pos, orient, self.confData, self.externalAxes)
		if duration:
			duration = "\T:=%s" % (duration)
		else:
			duration = ""
		if setSignal:
			command = "MoveJDO"
			setSignal = ", %s,%s" % (setSignal[0], int(setSignal[1]))
		else:
			setSignal = ""
		self.main[robot-1].append("%s %s, %s %s, %s, %s \WObj:=%s%s;" % \
			(command, tname, self.currentSpeed, duration, self.currentZone, 
			 self.currentTool, self.currentWobj, setSignal))


	def digital_out(self, signal="do1", state=1, delay=0, wait=0, sync=False, robot=1):
		"""Set digital out pin.

		delay: in seconds before setting the signal
		wait: in seconds after setting the signal
		sync: wait until the signal is physically set
			  similar to: SetDO do1,1; WaitDO do1 ,1;

		SetDO do15, 1;
		SetDO \SDelay := 0.2, weld, high;
		SetDO [ \SDelay ]|[ \Sync ] Signal Value
		WaitTime 0.2

		I/O signals are set to 0 when the robot starts up. They are however not
		affected by emergency stops.

		See also: SetDO, PulseDO, MoveLDO, WaitTime
		          RAPID overview/IO Principles
		"""
		if delay:
			delay = " \SDelay:=%s" % (delay)
		else:
			delay = ""
		if sync:
			sync = " \Sync:=%s," % (int(sync))
		else:
			sync = ""
		self.main[robot-1].append("SetDO%s%s %s, %s;" %
			                      (delay, sync, signal, int(state)))
		# self.main[robot-1].append("SetDO %s, %s;" %
		# 	                      (signal, int(state)))
		if wait:
			self.main[robot-1].append("WaitTime %s;" % (wait))
		

	def monitor_conf_linear(self, on=True, robot=1):
		"""Monitor if arm configuration is like computed.

		A simple rule to avoid problems, both for ConfL\On and \Off, is to 
		insert intermediate	points to make the movement of each axis less 
		than 90 degrees between points. More precisely, the sum of 
		movements for any of the par of axes (1+4), (1+6), (3+4) or (3+6) 
		should not exceed 180 degrees.

		See also: SingArea
		"""
		main = self.main_head[robot-1]
		if on:
			main.append("ConfL \On;")
		else:
			main.append("ConfL \On;")


	def monitor_conf_joint(self, on=True, robot=1):
		"""Monitor if arm configuration is like computed."""
		main = self.main_head[robot-1]
		if on:
			main.append("ConfJ \On;")
		else:
			main.append("ConfJ \On;")


	def wrist_singularity(self, reorient=True, robot=1):
		"""Define how to deal with wrist singulrities.

		A wrist singularity happens when axis 5 is at 0deg and
		therefore axis 4 and 6 are parallel and counter-act each other.

		The controller can automatically change the orientation a bit to
		avoid the singularity. If not then slowdown and stange sweeps 
		may happen.
		"""
		main = self.main_head[robot-1]
		if reorient:
			main.append("SingArea \Wrist;")
		else:
			main.append("SingArea \Off;")
			



	def start_sync_move(self, robot):
		main = self.main[robot-1]
		main.append("SyncMoveOn SyncOn, task_list;")


	def stop_sync_move(self, robot):
		main = self.main[robot-1]
		main.append("SyncMoveOff SyncOff;")
		main.append("UNDO")
		main.append("  SyncMoveUndo;")


	def _add_workobject_var(self, varname, holdingWork=False, fixed=True, unitExt="",
	                        userFramePos=[0,0,0], userFrameOrient=[1,0,0,0],
	                        workFramePos=[0,0,0], workFrameOrient=[1,0,0,0],  robot=1):
		"""Define work object coordinate system.

		The array has the following data:
		0: is the robot holding the work object (instead of the tool)
		1: using fixed coord system (instead of synced with external axis)
		2: unit of movement for un-fixed coord system
		3: user coord system (e.g. work table): 
		   position/orientation in world coordinated
		   (in wrist coords if robot is holding work object)
		4: object coord system (e.g. work piece on table): 
		   position/orientation in user coord system
		
		RAPID has a predefined work object coordinate system "wobj0" as follows:
		PERS wobjdata wobj0 := [FALSE, TRUE, "", [[0, 0, 0], [1, 0, 0,0]], [[0, 0, 0], [1, 0, 0 ,0]]];
		"""
		vars_ = self.vars[robot-1]
		frame = [holdingWork,fixed,unitExt,[userFramePos,userFrameOrient],[workFramePos,workFrameOrient]]
		vars_.append("PERS wobjdata %s := %s;" % (varname, frame))


	def _add_speed_var(self, varname, lin=100, rot=500, lin_ext=5000, rot_ext=1000, robot=1):
		"""Define speed for linear and rotational moves.

		The array has the following data:
		0: linear speed in mm/s
		1: rotationsal speed in def/s
		2: linear speed external axes
		3: rotational speed external axes
		"""
		vars_ = self.vars[robot-1]
		speed = [lin, rot, lin_ext, rot_ext]
		vars_.append("VAR speeddata %s := %s;" % (varname, speed))


	def _add_robtarget_var(self, varname, pos, orient, conf, extAxes, robot=1):
		"""Define a robtarget for moves."""
		vars_ = self.vars[robot-1]
		robtarget = [pos, orient, conf, extAxes]
		vars_.append("CONST robtarget %s := %s;" % (varname, robtarget))