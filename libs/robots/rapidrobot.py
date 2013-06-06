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

# temppath = tempfile.gettempdir()
# local_output = os.path.join(temppath, "gx")
# LOCAL_OUTPUT_PATH = [os.path.join(local_output, "T_ROB1.mod"),
#                    os.path.join(local_output, "T_ROB2.mod"),
#                    os.path.join(local_output, "T_ROB3.mod")]
thislocation = os.path.dirname(os.path.realpath(__file__))
temppath = os.path.join(thislocation, '..', '..', 'temp')
LOCAL_OUTPUT_PATH = [os.path.join(temppath, "T_ROB1.mod"),
                     os.path.join(temppath, "T_ROB2.mod"),
                     os.path.join(temppath, "T_ROB3.mod")]


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
        self.currentFrame = "wobj0"  # using default where user and work frame is the same as world
        self.confData = [0,0,0,0]  # Specifies robot axes angles
        # self.confData = [1,0,0,0]  # Specifies robot axes angles
        self.externalAxes = [9e9, 9e9, 9e9, 9e9, 9e9, 9e9]
        # self.externalAxes = "externalAxis0"



    def go(self, dest=ABB_CONTROLLER_TASKS_ROOT):
        """FTP-upload RAPID code to ABB controller (IRC5)"""
        ftp = ftplib.FTP()
        ftp.connect(ABB_CONTROLLER_IP, ABB_CONTROLLER_FTP_PORT, 6)
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


    # def dump(self, robot=1):
    #     code = self._compile(robot)
    #     print(code)

    def dump(self, filename=None, robot=1):
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
                     (self.currentSpeed, self.currentZone, self.currentTool, self.currentFrame))
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



    # ###########################################
    # Trajectory

    def path(self, path, robot=1):
        """Set path of the robot."""

        # add all tool definitions
        for varname,data in path.tooldefs[1].iteritems():
            pose = data[0]
            mass = data[1]
            massCenterPose = data[2]
            modelFile = [3]
            modelPose = [4]
            self._add_tool_var(varname, pos=pose.pos, rot=pose.rot, 
                               mass=mass, massCenterPos=massCenterPose.pos, 
                               massCenterRot=massCenterPose.rot, robot=robot)
        # add all frame definitions
        for varname,data in path.framedefs[1].iteritems():
            frame = data[0]
            self._add_frame_var(varname, workFramePos=frame.pos, 
                                workFrameOrient=frame.rot, robot=robot)
        # add all speed definitions
        for varname,data in path.speeddefs[1].iteritems():
            lin = data[0]
            rot = data[1]
            self._add_speed_var(varname, lin=lin, rot=rot, robot=robot)
        # add all zone definitions
        for varname,data in path.zonedefs[1].iteritems():
            radius = data[0]
            finep = False
            if radius == 0:
                finep = True
            self._add_zone_var(varname, finep=finep, radius=radius, robot=robot)
        # add commands with reference to above definitions
        for command in path.commands:
            typ = command[0]
            if typ == "target":
                pos = command[1]
                rot = command[2]
                dur = command[3]
                inter = command[4]
                tool = command[5]
                frame = command[6]
                speed = command[7]
                zone = command[8]
                signal = command[9]
                state = command[10]
                # set context
                if tool:
                    self.currentTool = tool
                if frame:
                    self.currentFrame = frame
                if speed:
                    self.currentSpeed = speed
                if zone:
                    self.currentZone = zone
                # add target
                if inter == 'LIN':
                    self.move_linear(pos, rot, dur, signal, state, robot)                        
                elif inter == 'PTP':
                    self.move_joint(pos, rot, dur, signal, state, robot) 
                else:
                    print "ERROR: invalid interpolation mode"
            elif typ == "axistarget":
                axes = command[1]
                dur = command[2]
                speed = command[3]
                if speed:
                    self.currentSpeed = speed
                self.move_joint_abs(self, axes, None, dur, robot)
            elif typ == "output":
                signal = command[1]
                state = command[2]
                delay = command[3]
                wait = command[4]
                sync = command[5]
                digital_out(signal, state, delay, wait, sync, robot)
            elif typ == "input":
                pass
            elif typ == "tool":
                pass
            elif typ == "frame":
                pass
            elif typ == "speed":
                # speed already packed with target
                pass
            elif typ == "zone":
                # speed already packed with target
                pass
            else:
                raise Exception("invalid command type")



    def move_linear(self, pos, orient, duration=None, signal=None, state=1, robot=1):
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
        if signal:
            command = "MoveLDO"
            setSignal = ", %s,%s" % (signal, int(state))
        else:
            setSignal = ""
        self.main[robot-1].append("%s %s, %s %s, %s, %s \WObj:=%s%s;" % \
            (command, tname, self.currentSpeed, duration, self.currentZone, 
             self.currentTool, self.currentFrame, setSignal))


    def move_joint(self, pos, orient, duration=None, signal=None, state=1, robot=1):
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
        if signal:
            command = "MoveJDO"
            setSignal = ", %s,%s" % (signal, int(state))
        else:
            setSignal = ""
        self.main[robot-1].append("%s %s, %s %s, %s, %s \WObj:=%s%s;" % \
            (command, tname, self.currentSpeed, duration, self.currentZone, 
             self.currentTool, self.currentFrame, setSignal))


    def move_joint_abs(self, robaxes, extaxes, duration=None, robot=1):
        """Move the robot to a joint position.

        This is a forward kinematics move that maps to a RAPID MoveAbsJ call.
        It typically takes a jointtarget, speed, zone, tool data. Tool data is
        only used for its information about mass.

        duration: in seconds, if specified the controller will overwrite 
        the speed setting and it will try to reach the pose in that time.

        MoveAbsJ p50, v1000, z50, tool2;
        MoveAbsJ *, v1000\T:=5, fine, grip3;
        MoveAbsJ [\Conc] ToJointPos [\ID] [\NoEOffs] Speed [\V] | [\T] Zone [\Z] [\Inpos] Tool [\WObj]
        """
        command = "MoveAbsJ"
        tname = "axes" + str(self.tcounter)
        self.tcounter += 1
        if not extaxes:
            extaxes = self.externalAxes
        self._add_axistarget_var(tname, robaxes, extaxes)
        if duration:
            duration = "\T:=%s" % (duration)
        else:
            duration = ""
        self.main[robot-1].append("%s %s, %s %s, %s, %s;" % \
            (command, tname, self.currentSpeed, duration, self.currentZone, 
             self.currentTool))



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
            delay = " \SDelay:=%s," % (delay)
        else:
            delay = ""
        if sync:
            sync = " \Sync:=%s," % (int(sync))
        else:
            sync = ""
        self.main[robot-1].append("SetDO%s%s %s, %s;" %
                                  (delay, sync, signal, int(state)))
        # self.main[robot-1].append("SetDO %s, %s;" %
        #                         (signal, int(state)))
        if wait:
            self.main[robot-1].append("WaitTime %s;" % (wait))
        

    def monitor_conf_linear(self, on=True, robot=1):
        """Monitor if arm configuration is like computed.

        A simple rule to avoid problems, both for ConfL\On and \Off, is to 
        insert intermediate points to make the movement of each axis less 
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


    def _add_robtarget_var(self, varname, pos, orient, conf, extaxes, robot=1):
        """Define a robtarget for moves."""
        vars_ = self.vars[robot-1]
        robtarget = [pos, orient, conf, extaxes]
        vars_.append("CONST robtarget %s := %s;" % (varname, robtarget))


    def _add_axistarget_var(self, varname, robaxes, extaxes, robot=1):
        """Define an axis target for abs moves."""
        vars_ = self.vars[robot-1]
        axistarget = [list(robaxes), list(extaxes)]
        vars_.append("CONST jointtarget %s := %s;" % (varname, axistarget))
        # CONST jointtarget calib_pos := [[ 0, 0, 0, 0, 0, 0], [ 0, 0, 0, 0, 0, 0]];


    def _add_tool_var(self, varname, holding=True, pos=[0,0,0], rot=[1,0,0,0], 
                      mass=0.001, massCenterPos=[0,0,0], massCenterRot=[1,0,0,0],
                      robot=1):
        """Define a tool.

        The array has the following data:
        0: is the robot holding the tool (instead of work object)
        1: tool center point transformation
        2: tool mass definitions
        
        RAPID has a predefined tool0 when not using any tool:
        PERS tooldata tool0 := [TRUE, [[0,0,0], [1,0,0,0]], [0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
        """
        vars_ = self.vars[robot-1]
        tool = [holding,[pos,rot],[mass,massCenterPos,massCenterRot,0,0,0]]
        vars_.append("PERS tooldata %s := %s;" % (varname, tool))


    def _add_frame_var(self, varname, holdingWork=False, fixed=True, unitExt="",
                       userFramePos=[0,0,0], userFrameOrient=[1,0,0,0],
                       workFramePos=[0,0,0], workFrameOrient=[1,0,0,0],
                       robot=1):
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


    def _add_speed_var(self, varname, lin=100, rot=500, extlin=5000, extrot=1000, robot=1):
        """Define speed for linear and rotational motion.

        lin: linear velocity (mm/s)
        rot: rotationsal speed (deg/s)
        extlin: linear velocity external axes (mm/s)
        extrot: rotational speed external axes (deg/s)

        Stock definitions:
        v5 5 mm/s 500deg/s 5000 mm/s 1000deg/s
        v10 10 mm/s 500deg/s 5000 mm/s 1000deg/s
        v25 20 mm/s 500deg/s 5000 mm/s 1000deg/s
        v30 30 mm/s 500deg/s 5000 mm/s 1000deg/s
        v40 40 mm/s 500deg/s 5000 mm/s 1000deg/s
        v50 Name 50 mm/s 500deg/s 5000 mm/s 1000deg/s
        v60 60 mm/s 500deg/s 5000 mm/s 1000deg/s
        v80 80 mm/s 500deg/s 5000 mm/s 1000deg/s
        v100 100 mm/s 500deg/s 5000 mm/s 1000deg/s
        v150 150 mm/s 500deg/s 5000 mm/s 1000deg/s
        v200 200 mm/s 500deg/s 5000 mm/s 1000deg/s
        v300 300 mm/s 500deg/s 5000 mm/s 1000deg/s
        v400 400 mm/s 500deg/s 5000 mm/s 1000deg/s
        v500 500 mm/s 500deg/s 5000 mm/s 1000deg/s
        v600 600 mm/s 500deg/s 5000 mm/s 1000deg/s
        v800 800 mm/s 500deg/s 5000 mm/s 1000deg/s
        v1000 1000 mm/s 500deg/s 5000 mm/s 1000deg/s
        v1500 1500 mm/s 500deg/s 5000 mm/s 1000deg/s
        v2000 2000 mm/s 500deg/s 5000 mm/s 1000deg/s
        v2500 2500 mm/s 500deg/s 5000 mm/s 1000deg/s
        v3000 3000 mm/s 500deg/s 5000 mm/s 1000deg/s
        v4000 4000 mm/s 500deg/s 5000 mm/s 1000deg/s
        v5000 5000 mm/s 500deg/s 5000 mm/s 1000deg/s
        v6000 6000 mm/s 500deg/s 5000 mm/s 1000deg/s
        v7000 7000 mm/s 500deg/s 5000 mm/s 1000deg/s
        vmax * 500deg/s 5000 mm/s 1000deg/s
        """
        vars_ = self.vars[robot-1]
        speed = [lin, rot, extlin, extrot]
        vars_.append("VAR speeddata %s := %s;" % (varname, speed))
        #VAR speeddata vmedium := [ 1000, 30, 200, 15 ];


    def _add_zone_var(self, varname, finep=False, radius=0.3, pzone_ori=None, 
                      pzone_eax=None, zone_ori=None, zone_leax=None, zone_reax=None,
                      robot=1):
        """Define a zone data definition.

        This determines how trajectory segments are blended and how
        close the TCP has to go to a target point.
        finep: is stop point
        radius: displacement radius (mm)
        pzone_ori: displacement orientation (mm)
        pzone_eax: displacement radius for external axes (mm)

        The following are only used if TCP is at standstill or much slower
        compared to external axes:
        zone_ori: displacement orientation if robot is holding work object (deg)
        zone_leax: displacement radius external axes (mm)
        zone_reax: displacement orientation external axes (deg)

        Stock definitions:
        fine ... stop point
        z0 0.3 mm 0.3 mm 0.3 mm 0.03deg 0.3 mm 0.03deg
        z1 1 mm 1 mm 1 mm 0.1deg 1 mm 0.1deg
        z5 5 mm 8 mm 8 mm 0.8deg 8 mm 0.8deg
        z10 10 mm 15 mm 15 mm 1.5deg 15 mm 1.5deg
        z15 15 mm 23 mm 23 mm 2.3deg 23 mm 2.3deg
        z20 20 mm 30 mm 30 mm 3.0deg 30 mm 3.0deg
        z30 30 mm 45 mm 45 mm 4.5deg 45 mm 4.5deg
        z40 40 mm 60 mm 60 mm 6.0deg 60 mm 6.0deg
        z50 50 mm 75 mm 75 mm 7.5deg 75 mm 7.5deg
        z60 60 mm 90 mm 90 mm 9.0deg 90 mm 9.0deg
        z80 80 mm 120 mm 120 mm 12deg 120 mm 12deg
        z100 100 mm 150 mm 150 mm 15deg 150 mm 15deg
        z150 150 mm 225 mm 225 mm 23deg 225 mm 23deg
        z200 200 mm 300 mm 300 mm 30deg 300 mm 30deg
        """

        if not pzone_ori:
            pzone_ori = 1.5*radius
        if not pzone_eax:
            pzone_eax = 1.5*radius
        if not zone_ori:
            zone_ori = 0.1*radius
        if not zone_leax:
            zone_leax = radius
        if not zone_reax:
            zone_reax = 0.1*radius

        vars_ = self.vars[robot-1]
        zonedata = [finep, radius, pzone_ori, 
                    pzone_eax, zone_ori, zone_leax, zone_reax]
        vars_.append("CONST zonedata %s := %s;" % (varname, zonedata))
        # VAR zonedata path := [ FALSE, 25, 40, 40, 10, 35, 5 ];

