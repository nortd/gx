"""Connects to an ABB robot through the open-abb-driver.

The open-abb-driver is a RAPID program that runs on the ABB
controller (IRC5) and opens a socket connection. through
this socket it receives commands that get mapped to RAPID
command pretty much one-to-one.
"""

import os
import sys
import math
import json
import time
from gx.libs import form
from gx.libs.robots import baserobot
from gx.libs.euclid import euclid
from gx.libs.vectormath import *

try:
    import FreeCAD
    import Part
    import Draft
    import Robot as fcRobot
    from PyQt4 import QtGui,QtCore
except ImportError:
    print("\nError: must use FreeCAD for simrobot module\n")


TO_RAD = math.pi/180.0
TO_DEG = 180.0/math.pi


class Robot(baserobot.Robot):

    def __init__(self, name="Robot"):
        baserobot.Robot.__init__(self)

        if not FreeCAD.activeDocument():
            FreeCAD.newDocument()
        doc = FreeCAD.activeDocument()

        # create a robot
        self.rob = doc.addObject("Robot::RobotObject", name)
        # self.rob.RobotVrmlFile = FreeCAD.getResourceDir()+"Mod/Robot/Lib/Kuka/kr500_1.wrl"
        # self.rob.RobotKinematicFile = FreeCAD.getResourceDir()+"Mod/Robot/Lib/Kuka/kr500_1.csv"
        thislocation = os.path.dirname(os.path.realpath(__file__))
        # self.rob.RobotVrmlFile = os.path.join(thislocation, 'kr16.wrl')
        # self.rob.RobotKinematicFile = os.path.join(thislocation, 'irb2600-20-1.65.csv')
        # self.rob.RobotKinematicFile = os.path.join(thislocation, 'kr_16.csv')
        defs = self.dynamic_def_files('irb2600')
        self.rob.RobotVrmlFile = defs['wrl']
        self.rob.RobotKinematicFile = defs['csv']

        # tool
        self.tcp_pos = V()          # actual TCP pos
        self.tcp_rot = R()          # actual TCP rot
        self.tool_pos = V()
        self.tool_rot = R()
        self.tool_pos_inv = V()
        self.tool_rot_inv = R()

        # work frame
        self._workframe = Pose(V(), R())
        self._workframe_inv = Pose(V(), R())

        # trajectory
        self._path = None

        # change Workbench
        # if hasattr(FreeCAD, 'Gui'):
        #     # FreeCAD.Gui.activateWorkbench("RobotWorkbench")
        #     FreeCAD.Gui.activateWorkbench("NoneWorkbench")

        # show all
        if hasattr(FreeCAD, 'Gui'):
            FreeCAD.Gui.SendMsgToActiveView("ViewFit")
            FreeCAD.Gui.activeDocument().ActiveView.viewFront()
            doc.recompute()

        # animations, this is robot animation chain
        self.animations = []

        # some positional presets
        self.FLOOR = V(1000,0,0)
        self.TABLE = V(800,0,800)

        # some rotational presets
        self.UP = R()
        self.FRONT = aR(pi/2, V(0,1,0))
        self.DOWN = aR(pi, V(0,1,0))
        self.LEFT = self.FRONT * aR(-pi/2, V(1,0,0))
        self.RIGHT = self.FRONT * aR(pi/2, V(1,0,0))

        # init pose
        self.axes = (0,0,0,0,0,0)
        _p = self.rob.Tcp.Base
        self.pos = V(_p.x, _p.y, _p.z)
        # self.toolrotate_to(0, math.pi/2.0, 0)
        self.rot = self.FRONT



    # ###########################################
    # Posing

    # rot properties
    @property
    def rot(self):
        return self.tcp_rot
    @rot.setter
    def rot(self, q):
        self.tcp_rot = q
        q_m = q * self.tool_rot_inv
        self.rob.Tcp.Rotation = FreeCAD.Rotation(q_m.x, q_m.y, q_m.z, q_m.w)
        # correct pos
        self.pos = self.tcp_pos


    # pos property
    @property
    def pos(self):
        return self._workframe_inv * self.tcp_pos
    @pos.setter
    def pos(self, p):
        p = self._workframe * p   # apply frame
        self.tcp_pos = p
        _q = self.rob.Tcp.Rotation.Q
        q = R(_q[3], _q[0], _q[1], _q[2])
        p_m = p + (q * self.tool_pos_inv)
        self.rob.Tcp.Base = (p_m.x, p_m.y, p_m.z)


    # axes property
    @property
    def axes(self):
        return (self.rob.Axis1, self.rob.Axis2, self.rob.Axis3, 
                self.rob.Axis4, self.rob.Axis5, self.rob.Axis6)
    @axes.setter
    def axes(self, axes):
        if len(axes) != 6:
            print "ERROR: invalid axes list"
        self.rob.Axis1 = axes[0]
        self.rob.Axis2 = axes[1]
        self.rob.Axis3 = axes[2]
        self.rob.Axis4 = axes[3]
        self.rob.Axis5 = axes[4]
        self.rob.Axis6 = axes[5]
        # correct position
        _p = self.rob.Tcp.Base
        p = V(_p[0], _p[1], _p[2])
        _q = self.rob.Tcp.Rotation.Q
        q = R(_q[3], _q[0], _q[1], _q[2])
        self.pos = p + (q * self.tool_pos)
        self.rot = q * self.tool_rot


    # tool rotation shortcuts
    def rotxyz(self, ang_x, ang_y, ang_z):
        """Set tool rotation by axis-aligned angles (relative).

        ang_x: angle around x-axis in radians
        ang_y: angle around y-axis in radians
        ang_z: angle around z-axis in radians
        """
        qx = aR(ang_x, V(1, 0, 0))
        qy = aR(ang_y, V(0, 1, 0))
        qz = aR(ang_z, V(0, 0, 1))
        self.rot = qx * qy * qz * self.rot


    def rotxyz_to(self, ang_x, ang_y, ang_z):
        """Set tool rotation by axis-aligned angles (absolute).

        ang_x: angle around x-axis in radians
        ang_y: angle around y-axis in radians
        ang_z: angle around z-axis in radians
        """
        qx = aR(ang_x, V(1, 0, 0))
        qy = aR(ang_y, V(0, 1, 0))
        qz = aR(ang_z, V(0, 0, 1))
        self.rot = qx * qy * qz


    def pose(self, pose):
        """Pose the tool by setting position and rotation."""
        self.rot = pose.rot
        self.pos = pose.pos


   
    # ###########################################
    # Trajectory

    def path(self, path):
        """Set path of the robot."""
        self._path = []  # reset
        for command in path.commands:
            typ = command[0]
            if typ == "target":
                pos = command[1]
                rot = command[2]
                dur = command[3]
                # inter = command[4]
                # tool = command[5]
                # frame = command[6]
                speed = command[7]
                # zone = command[8]
                # signal = command[9]
                # state = command[10]
                # tooldata = path.gettool(tool)
                # framedata = path.getframe(frame)
                speeddata = path.getspeed(speed)
                # zonedata = path.getzone(zone)
                linspeed = speeddata[0]
                rotspeed = speeddata[1]
                # finally add the data we need
                # other data is not used yet but can be in future
                self._path.append((Pose(pos,rot), dur, linspeed, rotspeed))
            elif typ == "axistarget":
                # axes = command[1]
                # dur = command[2]
                # speed = command[3]
                # speeddata = path.getspeed(speed)
                # rotspeed = speeddata[1]
                # self._path.append(axes, dur, rotspeed)
                pass
            elif typ == "gpio":
                # name = command[1]
                # state = command[2]
                # delay = command[3]
                # wait = command[4]
                # sync = command[5]
                pass
            else:
                raise Exception("invalid command type")

        self.generate_freecad_trajectory()  # for visualization mostly



    def generate_freecad_trajectory(self):
        """Generate a FreeCAD trajectory.

        This allows for using the build-in robot simulation as opposed
        to the gx simulation. Native simulation may be useful for
        debugging. To use this, select robot and this trajectory from
        FreeCAD's tree view and press the simulate button.
        """
        doc = FreeCAD.activeDocument()
        traj = doc.addObject("Robot::TrajectoryObject","Trajectory").Trajectory
        for target in self._path:
            pos = target[0].pos
            rot = target[0].rot
            velocity = target[2]
            _pose = FreeCAD.Placement(FreeCAD.Vector(pos.x, pos.y, pos.z), 
                                      FreeCAD.Rotation(rot.x, rot.y, rot.z, rot.w))
            wp = fcRobot.Waypoint(_pose, "LIN", "Pt", velocity, False)
            traj.insertWaypoints(wp)


    def add_demo_trajectory(self):
        self.trajects = []
        doc = FreeCAD.activeDocument()
        t = doc.addObject("Robot::TrajectoryObject","Trajectory").Trajectory
        # startTcp = self.rob.Tcp # = FreeCAD.Placement(FreeCAD.Vector(500,500,500), FreeCAD.Rotation(1,0,0,1))
        startTcp = self.rob.Tcp = FreeCAD.Placement(FreeCAD.Vector(1177.93, 0.0, 430.462), FreeCAD.Rotation(0,1,0,0))
        t.insertWaypoints(fcRobot.Waypoint(startTcp, "LIN","Pt",4000))
        for i in range(7):
            # rot = aR(ang_x*TO_RAD, V(1, 0, 0))
            qx = aR(-90*TO_RAD, V(1, 0, 0))
            qy = aR(i*20*TO_RAD, V(0, 1, 0))
            rot = qx*qy
            # FreeCAD.Rotation (-0.479426,-0,-0,0.877583)
            fcOrient = FreeCAD.Rotation(rot.x, rot.y, rot.z, rot.w)
            fcPos = FreeCAD.Vector(i*30+800, 500, -i*50+800)
            # t.insertWaypoints(fcRobot.Waypoint(FreeCAD.Placement(fcPos, fcOrient),"LIN","Pt"))
            t.insertWaypoints(fcRobot.Waypoint(FreeCAD.Placement(fcPos, fcOrient),"LIN","Pt",4000))
        t.insertWaypoints(fcRobot.Waypoint(startTcp, "LIN","Pt",4000)) # end point of the trajectory
        # App.activeDocument().Trajectory.Trajectory = t
        self.trajects.append(t)



    # ###########################################
    # Robot State

    def tool(self, tool_name, search_path=None):
        """Loads a tool definition from file.

        This function adds a tool to the robot based on a tool definition.
        A tool definition is a json file specifying tcp pose, mass, center of
        mass pos, model pose. A model file (.wrl) of the same name is used too.
        name: Corresponds to tool_name.json and tool_name.wrl.
        search_path: The directory to look for tool. By default this function 
                     looks for the tool in libs/robots/tools

        json file format:
        {
            "pos": {"x":0, "y":-50, "z":225},
            "rot": {"w":1,"x":0,"y":0,"z":0},
            "modelPos": {"x":0, "y":0, "z":0},
            "modelRot": {"w":1,"x":0,"y":1,"z":0},
            "mass": 5.0,
            "massCenterPos": {"x":0, "y":0, "z":0},
        }
        """
        if not search_path:
            thislocation = os.path.dirname(os.path.realpath(__file__))
            search_path = os.path.join(thislocation, 'tools')
        tooldefjson = os.path.join(search_path, tool_name + '.json')
        tooldefwrl = os.path.join(search_path, tool_name + '.wrl')
        if not os.path.exists(tooldefjson):
            print("ERROR: tool def json file not found: %s" % (tooldefjson))
            return
        if not os.path.exists(tooldefwrl):
            modelFile = None
        else:
            modelFile = tooldefwrl

        # open, attach wrl file
        FreeCAD.Gui.insert(tooldefwrl, FreeCAD.activeDocument().Name)  #TODO: open file without using Gui
        # self.rob.ToolShape = FreeCAD.activeDocument().circular_saw
        self.rob.ToolShape =  FreeCAD.ActiveDocument.Objects[-1]

        # set transforms
        with open(tooldefjson) as data_file:
            data = json.load(data_file)

        pos = V()
        rot = R()
        mass = 0.001
        massCenterPos = V()
        modelPos = V()
        modelRot = R()

        # tcp translation (in relation to flange)
        p = data.get('pos')
        if p:
            pos = V(p[0], p[1], p[2])
        # tcp rotation (in relation to flange)
        r = data.get('rot')
        if r:
            rot = R(r[0], r[1], r[2], r[3])

        # mass (kg)
        masskg = data.get('mass')
        if masskg:
            mass = masskg
        # mass translation (in relation to flange)
        p = data.get('massCenterPos')
        if p:
            modelPos = V(p[0], p[1], p[2])
        
        # tool translation (in relation to flange)
        p = data.get('modelPos')
        if p:
            modelPos = V(p[0], p[1], p[2])
        # tool rotation (in relation to flange)
        r = data.get('modelRot')
        if r:
            modelRot = R(r[0], r[1], r[2], r[3])

        # assign to robot
        self.rob.Tool.Base = (pos.x, pos.y, pos.z)
        self.rob.Tool.Rotation.Q = (rot.x, rot.y, rot.z, rot.w)
        self.rob.ToolBase.Base = (modelPos.x, modelPos.y, modelPos.z)
        self.rob.ToolBase.Rotation.Q = (modelRot.x, modelRot.y, modelRot.z, modelRot.w)

        # store for later use
        tplace = self.rob.Tool
        p = tplace.Base
        self.tool_pos = V(p[0], p[1], p[2])
        q = tplace.Rotation.Q
        self.tool_rot = R(q[3], q[0], q[1], q[2])
        #inverse
        tplace_inv = self.rob.Tool.inverse()
        pinv = tplace_inv.Base
        self.tool_pos_inv = V(pinv[0], pinv[1], pinv[2])
        qinv = tplace_inv.Rotation.Q
        self.tool_rot_inv = R(qinv[3], qinv[0], qinv[1], qinv[2])

        # update pos so tool end is new tcp
        self.pos = self.pos



    def frame(self, pos, rot=R()):
        """Defines a work object frame (coordinate system)."""
        # x = (origin - xpoint).normalized()
        # y_ = (origin - ypoint).normalized()
        # z = y_.cross(x).normalized()
        # y = x.cross(z)
        # m = euclid.Matrix4.new_rotate_triple_axis(x, y, z)
        # # m.d, m.h, m.l = origin.x, origin.y, origin.z
        self._workframe = Pose(pos, rot)
        self._workframe_inv = self._workframe.inverse()
        # # correct current state
        # self. = self._workframe_inv * self.tcp_pos






    # ###########################################
    # Simbot Animation

    def go(self):
        """Take path and generate animation."""
        last_pose = None
        for target in self._path:
            # target has (pose, dur, linspeed, rotspeed)
            pose = target[0]
            dur = target[1]
            linspeed = target[2]
            rotspeed = target[3]
            if last_pose:
                dist = last_pose.pos.distance(pose.pos)
                if linspeed == 0:
                    dur = 0.0
                else:
                    dur = dist/float(linspeed)
                self.animations.append([last_pose, pose, dur, None])
            last_pose = pose

        # setup timer callback
        self.timer = QtCore.QTimer()
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self._animhandler)
        self.timer.start(40)


    def stop(self):
        self.animations = []
        self.timer.stop()
        QtCore.QObject.disconnect(self.timer, QtCore.SIGNAL("timeout()"), self._animhandler)


    def _animhandler(self):
        """Animation callback, handler of qt timer.

        NOTE: Any bugs in this context are completely silent
              in FreeCAD. To debug, print a message at the end of this
              handler and make sure it will be reached.
        """
        # FreeCAD.Console.PrintMessage('<')
        # end condition
        if len(self.animations) == 0:
            # FreeCAD.Console.PrintMessage('-')
            self.stop()
            return

        last_pose = self.animations[0][0]
        pose = self.animations[0][1]
        dur = self.animations[0][2]
        state = self.animations[0][3]
        if not state:
            # new motion condition
            # FreeCAD.Console.PrintMessage('*')
            self.animations[0][3] = time.time()  # set state
            self.pose(last_pose)
        else:
            # continue motion
            # FreeCAD.Console.PrintMessage('+')
            dt = time.time() - state
            if dur > 0:
                t_pct = dt/dur
            else:
                t_pct = 1.1  # meaning we are already there

            if t_pct > 1:
                # animation done
                t_pct = 1.0
                self.animations.pop(0)

            # linear and SLERP interpolation
            self.pose(Pose.new_interpolate(last_pose, pose, t_pct))

            # this is how to manually compensate for the tool trans
            # r.rob.Tcp = r.path.Waypoints[0].Pos.multiply(r.rob.Tool.inverse())

        # FreeCAD.Console.PrintMessage('>')



    # ###########################################
    # Robot Configuration

    def dynamic_def_files(self, robot_name):
        dh_params = None
        vrml_file = None
        if robot_name == 'kr16':
            # The Denavit-Hartenberg parameters and model for a Kuka KR16.
            dh_params = {
                'axis1':{'a':260.0, 'alpha':-90.0, 'd':675.0, 'theta':0.0, 'rotDir':-1, 'maxAngle':185.0, 'minAngle':-185.0, 'axisVel':156.0},
                'axis2':{'a':680.0, 'alpha':0.0, 'd':0.0, 'theta':0.0, 'rotDir':1, 'maxAngle':35.0, 'minAngle':-155.0, 'axisVel':156.0},
                'axis3':{'a':-35.0, 'alpha':90.0, 'd':0.0, 'theta':-90.0, 'rotDir':1, 'maxAngle':154.0, 'minAngle':-130.0, 'axisVel':156.0},
                'axis4':{'a':0.0, 'alpha':-90.0, 'd':-675.0, 'theta':0.0, 'rotDir':1, 'maxAngle':350.0, 'minAngle':-350.0, 'axisVel':330.0},
                'axis5':{'a':0.0, 'alpha':90.0, 'd':0.0, 'theta':0.0, 'rotDir':1, 'maxAngle':130.0, 'minAngle':-130.0, 'axisVel':330.0},
                'axis6':{'a':0.0, 'alpha':180.0, 'd':-158.0, 'theta':180.0, 'rotDir':1, 'maxAngle':350.0, 'minAngle':-350.0, 'axisVel':615.0},
            }
            vrml_file = FreeCAD.getResourceDir()+"Mod/Robot/Lib/Kuka/kr16.wrl"
        elif robot_name == 'kr500':
            # The Denavit-Hartenberg parameters and model for a Kuka KR500.
            dh_params = {
                'axis1':{'a':500.0, 'alpha':-90.0, 'd':1045.0, 'theta':0.0, 'rotDir':-1, 'maxAngle':185.0, 'minAngle':-185.0, 'axisVel':156.0},
                'axis2':{'a':1300.0, 'alpha':0.0, 'd':0.0, 'theta':0.0, 'rotDir':1, 'maxAngle':35.0, 'minAngle':-155.0, 'axisVel':156.0},
                'axis3':{'a':55.0, 'alpha':90.0, 'd':0.0, 'theta':-90.0, 'rotDir':1, 'maxAngle':154.0, 'minAngle':-130.0, 'axisVel':156.0},
                'axis4':{'a':0.0, 'alpha':-90.0, 'd':-1025.0, 'theta':0.0, 'rotDir':1, 'maxAngle':350.0, 'minAngle':-350.0, 'axisVel':330.0},
                'axis5':{'a':0.0, 'alpha':90.0, 'd':0.0, 'theta':0.0, 'rotDir':1, 'maxAngle':130.0, 'minAngle':-130.0, 'axisVel':330.0},
                'axis6':{'a':0.0, 'alpha':180.0, 'd':-300.0, 'theta':180.0, 'rotDir':1, 'maxAngle':350.0, 'minAngle':-350.0, 'axisVel':615.0},
            }
            vrml_file = FreeCAD.getResourceDir()+"Mod/Robot/Lib/Kuka/kr500_1.wrl"
        elif robot_name == 'irb2600':
            # The Denavit-Hartenberg parameters and model for an ABB IRB2600-20-1.65.
            # it appears to be ok to invert a and alpha simultaneously
            # min/maxAngle is min/max theta
            dh_params = {
                'axis1':{'a':150.0, 'alpha':-90.0, 'd':445.0, 'theta':0.0, 'rotDir':1, 'maxAngle':180.0, 'minAngle':-180.0, 'axisVel':175.0},
                'axis2':{'a':700.0, 'alpha':0.0, 'd':0.0, 'theta':-90.0, 'rotDir':1, 'maxAngle':155.0, 'minAngle':-95.0, 'axisVel':175.0},
                'axis3':{'a':115.0, 'alpha':90.0, 'd':0.0, 'theta':0.0, 'rotDir':1, 'maxAngle':75.0, 'minAngle':-180.0, 'axisVel':175.0},
                'axis4':{'a':0.0, 'alpha':-90.0, 'd':-795.0, 'theta':0.0, 'rotDir':-1, 'maxAngle':400.0, 'minAngle':-400.0, 'axisVel':360.0},
                'axis5':{'a':0.0, 'alpha':90.0, 'd':0.0, 'theta':0.0, 'rotDir':1, 'maxAngle':120.0, 'minAngle':-120.0, 'axisVel':360.0},
                'axis6':{'a':0.0, 'alpha':180.0, 'd':-85.0, 'theta':180.0, 'rotDir':-1, 'maxAngle':400.0, 'minAngle':-400.0, 'axisVel':500.0},
            }
            thislocation = os.path.dirname(os.path.realpath(__file__))
            vrml_file = os.path.join(thislocation, 'irb2600.wrl')
        else:
            print "ERROR: no dh data for robot name"
            return

        axes = ('axis1', 'axis2', 'axis3', 'axis4', 'axis5', 'axis6')
        params = ('a', 'alpha', 'd', 'theta', 'rotDir', 'maxAngle', 'minAngle', 'axisVel')
        csv_string = []
        first = True
        for param in params:
            if first:
                first = False
            else:
                csv_string.append(',   ') 
            csv_string.append(param)
        csv_string.append('\n')
        for axis in axes:
            first = True
            for param in params:
                if first:
                    first = False
                else:
                    csv_string.append(',   ')    
                csv_string.append(str(dh_params[axis][param]))
            csv_string.append('\n')
        csv_string = ''.join(csv_string)
        thislocation = os.path.dirname(os.path.realpath(__file__))
        filename = os.path.join(thislocation, '..', '..', 'temp', 'dh_file.csv')
        with open(filename, 'w') as f:
            f.write(csv_string)
            print("INFO: DH file witten to: %s" % (filename))
        return { 'csv': filename, 'wrl':vrml_file }
