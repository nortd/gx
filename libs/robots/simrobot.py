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

        # Kuka-specific
        # self.rob.Axis2 = -90
        # self.rob.Axis3 = 90
        self.rob.Axis1 = 0
        self.rob.Axis2 = 0
        self.rob.Axis3 = 0
        self.rob.Axis4 = 0
        self.rob.Axis5 = 0
        self.rob.Axis6 = 0

        # tool
        self.tool_pos = euclid.Vector3()
        self.tool_rot = euclid.Quaternion()
        self.tool_pos_inv = euclid.Vector3()
        self.tool_rot_inv = euclid.Quaternion()

        # path
        self.path = None
        self.rot_now = (0,0,0,1)
        self.velocity_now = 100
        self.type_now = "LIN"
        self.name_now = "Pt"
        self.continuity_now = False

        # change to Robot Workbench
        if hasattr(FreeCAD, 'Gui'):
            FreeCAD.Gui.activateWorkbench("RobotWorkbench")

        # show all
        if hasattr(FreeCAD, 'Gui'):
            FreeCAD.Gui.SendMsgToActiveView("ViewFit")
            FreeCAD.Gui.activeDocument().ActiveView.viewFront()
            doc.recompute()

        # animations, this is robot animation chain
        self.animations = []

    # def reset(self):
    #   self.trajectories = []

    def pose(self, pos, orient, robot=1):
        pass

    # def move_linear(self, pos, orient, robot=1):
    #   """Linear move to position and orientation
    #   """
    #   self.oadrobot.setCartesian([pos, orient])


    def move(self, x, y, z, robot=1):
      self.rob.Tcp.move(App.Vector(x, y, z))


    def rot(self, ang_x, ang_y, ang_z):
        """Rotate (absolute) robot orientation to axis-aligned angles.

        ang_x: angle around x-axis in degrees
        ang_y: angle around y-axis in degrees
        ang_z: angle around z-axis in degrees
        """
        qx = euclid.Quaternion.new_rotate_axis(ang_x*TO_RAD, euclid.Vector3(1, 0, 0))
        qy = euclid.Quaternion.new_rotate_axis(ang_y*TO_RAD, euclid.Vector3(0, 1, 0))
        qz = euclid.Quaternion.new_rotate_axis(ang_z*TO_RAD, euclid.Vector3(0, 0, 1))
        self.orient = qx * qy * qz


    # pos property
    @property
    def pos(self):
        # convert from FreeCAD to euclid
        p = self.rob.Tcp.Base
        return euclid.Point3(p[0], p[1], p[2])# - self.tool_pos
    @pos.setter
    def pos(self, p):
        # # convert from euclid to FreeCAD, compensate for tcp transform
        # p_flange = p + self.tool_pos
        self.rob.Tcp.Base = (p[0], p[1], p[2])
        # self.rob.Tcp.Base = (p_flange[0], p_flange[1], p_flange[2])
        # rTcp_ = self.rob.Tcp.Rotation.Q
        # rTcp_inv = euclid.Quaternion(-rTcp_[3], rTcp_[0], rTcp_[1], rTcp_[2])
        # pr = (self.tool_rot_inv*rTcp_inv)*p
        # rTcp = euclid.Quaternion(rTcp_[3], rTcp_[0], rTcp_[1], rTcp_[2])
        # pr = (rTcp*self.tool_rot)*p
        # self.rob.Tcp.Base = (pr[0], pr[1], pr[2])


    # orient property
    @property
    def orient(self):
        # convert from FreeCAD to euclid
        # euclid lib uses (w, x, y, z)
        # FreeCad uses (x, y, z, w)
        q = self.rob.Tcp.Rotation.Q
        return euclid.Quaternion(q[3], q[0], q[1], q[2])#*self.tool_rot
    @orient.setter
    def orient(self, quat):
        # convert form euclid to FreeCAD, compensate for tcp transform
        # q_flange = quat#*setterelf.tool_rot_inv
        self.rob.Tcp.Rotation.Q = (quat.x, quat.y, quat.z, quat.w)
        # self.rob.Tcp.Rotation.Q = (q_flange.x, q_flange.y, q_flange.z, q_flange.w)

    # Axis properties
    # axis1 ... axis6
    @property
    def axis1(self):
        return self.rob.Axis1
    @axis1.setter
    def axis1(self, value):
        self.rob.Axis1 = value
    @property
    def axis2(self):
        return self.rob.Axis2
    @axis2.setter
    def axis2(self, value):
        self.rob.Axis2 = value
    @property
    def axis3(self):
        return self.rob.Axis3
    @axis3.setter
    def axis3(self, value):
        self.rob.Axis3 = value
    @property
    def axis4(self):
        return self.rob.Axis4
    @axis4.setter
    def axis4(self, value):
        self.rob.Axis4 = value
    @property
    def axis5(self):
        return self.rob.Axis5
    @axis5.setter
    def axis5(self, value):
        self.rob.Axis5 = value
    @property
    def axis6(self):
        return self.rob.Axis6
    @axis6.setter
    def axis6(self, value):
        self.rob.Axis6 = value

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



    def tool(self, tool_name, search_path=None):
        """Set the tool of the robot.

        This function adds a tool to the robot based on a tool definition.
        A tool definition is a json file specifying the transforms and a
        vrml file specifying the shape.
        tool_name: Name, also corresponds to tool_name.json and tool_name.wrl
        search_path: The directory to look for tool. By default this function 
                     looks for the tool in libs/robots/tools

        json file format:
        {
            "translation": {"x":0, "y":-50, "z":225},
            "rotation": {"w":1,"x":0,"y":0,"z":1},
            "translation_tcp": {"x":0, "y":0, "z":0},
            "rotation_tcp": {"w":1,"x":0,"y":1,"z":0}
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
            print("ERROR: tool def vrml file not found: %s" % (tooldefwrl))
            return
        
        # open, attach wrl file
        FreeCAD.Gui.insert(tooldefwrl, FreeCAD.activeDocument().Name)  #TODO: open file without using Gui
        # self.rob.ToolShape = FreeCAD.activeDocument().circular_saw
        self.rob.ToolShape =  FreeCAD.ActiveDocument.Objects[-1]
        # set transforms
        with open(tooldefjson) as data_file:
            data = json.load(data_file)

        # tool translation (in relation to flange)
        tTool = data.get('translation')
        if tTool:
            self.rob.ToolBase.Base = (tTool['x'], tTool['y'], tTool['z'])
        # tool rotation (in relation to flange)
        rTool = data.get('rotation')
        if rTool:
            self.rob.ToolBase.Rotation.Q = (rTool['x'], rTool['y'], rTool['z'], rTool['w'])
        # tcp translation (in relation to flange)
        tTcp = data.get('translation_tcp')
        if tTcp:
            self.rob.Tool.Base = (tTcp['x'], tTcp['y'], tTcp['z'])
        # tcp rotation (in relation to flange)
        rTcp = data.get('rotation_tcp')
        if rTcp:
            self.rob.Tool.Rotation.Q = (rTcp['x'], rTcp['y'], rTcp['z'], rTcp['w'])

        # store for later use
        tplace = self.rob.Tool
        p = tplace.Base
        self.tool_pos = euclid.Vector3(p[0], p[1], p[2])
        q = tplace.Rotation.Q
        self.tool_rot = euclid.Quaternion(q[3], q[0], q[1], q[2])
        FreeCAD.Console.PrintMessage(self.tool_rot.get_angle_axis())
        #inverse
        tplace_inv = self.rob.Tool.inverse()
        pinv = tplace_inv.Base
        self.tool_pos_inv = euclid.Vector3(pinv[0], pinv[1], pinv[2])
        qinv = tplace_inv.Rotation.Q
        # self.tool_rot_inv = euclid.Quaternion(qinv[1], qinv[2], qinv[3], qinv[0])
        self.tool_rot_inv = self.tool_rot
        self.tool_rot_inv.w = -self.tool_rot_inv.w
        FreeCAD.Console.PrintMessage(self.tool_rot_inv.get_angle_axis())
        # FreeCAD.Console.PrintMessage(self.tool_rot.inverse().get_angle_axis())




    def waypoint(self, pos, rot=None, 
                 type_=None, name=None, velocity=None, continuity=None):
        """Add a waypoint to the robot's path.

        All parameters but the position are optional. They only need to 
        be supplied when they change.

        point: position, a euclid point
        orient: orientation, a euclid quaternion
        type_: type of movement, defaults to LIN
        name: name of waypoint
        velocity: how fast to get to waypoint
        continuity: pass-by or not
        """
        if rot:
            self.rot_now = rot
        if velocity:
            self.velocity_now = velocity
        if type_:
            self.type_now = type_
        if name:
            self.name_now = name
        if continuity:
            self.continuity_now = continuity

        if not self.path:
            doc = FreeCAD.activeDocument()
            self.path = doc.addObject("Robot::TrajectoryObject","Trajectory").Trajectory

        pose = FreeCAD.Placement(FreeCAD.Vector(pos.x, pos.y, pos.z), 
                                 FreeCAD.Rotation(self.rot_now.x, self.rot_now.y, 
                                                  self.rot_now.z, self.rot_now.w))
        wp = fcRobot.Waypoint(pose, self.type_now, self.name_now, 
                              self.velocity_now, self.continuity_now)
        self.path.insertWaypoints(wp)



    def add_demo_trajectory(self):
        self.trajects = []
        doc = FreeCAD.activeDocument()
        t = doc.addObject("Robot::TrajectoryObject","Trajectory").Trajectory
        # startTcp = self.rob.Tcp # = FreeCAD.Placement(FreeCAD.Vector(500,500,500), FreeCAD.Rotation(1,0,0,1))
        startTcp = self.rob.Tcp = FreeCAD.Placement(FreeCAD.Vector(1177.93, 0.0, 430.462), FreeCAD.Rotation(0,1,0,0))
        t.insertWaypoints(fcRobot.Waypoint(startTcp, "LIN","Pt",4000))
        for i in range(7):
            # orient = euclid.Quaternion.new_rotate_axis(ang_x*TO_RAD, euclid.Vector3(1, 0, 0))
            qx = euclid.Quaternion.new_rotate_axis(-90*TO_RAD, euclid.Vector3(1, 0, 0))
            qy = euclid.Quaternion.new_rotate_axis(i*20*TO_RAD, euclid.Vector3(0, 1, 0))
            orient = qx*qy
            # FreeCAD.Rotation (-0.479426,-0,-0,0.877583)
            fcOrient = FreeCAD.Rotation(orient.x, orient.y, orient.z, orient.w)
            fcPos = FreeCAD.Vector(i*30+800, 500, -i*50+800)
            # t.insertWaypoints(fcRobot.Waypoint(FreeCAD.Placement(fcPos, fcOrient),"LIN","Pt"))
            t.insertWaypoints(fcRobot.Waypoint(FreeCAD.Placement(fcPos, fcOrient),"LIN","Pt",4000))
        t.insertWaypoints(fcRobot.Waypoint(startTcp, "LIN","Pt",4000)) # end point of the trajectory
        # App.activeDocument().Trajectory.Trajectory = t
        self.trajects.append(t)



    ##################################### Simbot Animation

    def add_animation(self, pos1, pos2, rot1, rot2, duration):
        self.animations.append([pos1, pos2, rot1, rot2, duration, None])


    def go(self):
        # add animations from waypoints
        last_pos = None
        last_rot = None
        for wp in self.path.Waypoints:
            pos_ = wp.Pos.Base
            pos = euclid.Point3(pos_.x, pos_.y, pos_.z)
            rot_ = wp.Pos.Rotation.Q
            rot = euclid.Quaternion(rot_[3], rot_[0], rot_[1], rot_[2])
            if last_pos and last_rot:
                vel = wp.Velocity  # mm/s
                dist = last_pos.distance(pos)
                if vel == 0:
                    dur = 0.0
                else:
                    dur = dist/vel
                self.add_animation(last_pos, pos, last_rot, rot, dur)
            last_pos = pos
            last_rot = rot

        # setup timer callback
        self.timer = QtCore.QTimer()
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self._animhandler)
        self.timer.start(40)


    def stop(self):
        self.animations = []
        self.timer.stop()
        QtCore.QObject.disconnect(self.timer, QtCore.SIGNAL("timeout()"), self._animhandler)


    def _animhandler(self):
        # FreeCAD.Console.PrintMessage('<')
        if len(self.animations) == 0:
            # FreeCAD.Console.PrintMessage('-')
            self.stop()
            return

        anim = self.animations[0]
        if not anim[5]:
            # FreeCAD.Console.PrintMessage('*')
            # starting new motion
            anim[5] = time.time()
            self.pos = pos1
            self.orient = rot1
        else:
            # FreeCAD.Console.PrintMessage('+')
            dt = time.time() - anim[5]
            duration = anim[4]
            if duration > 0:
                t_pct = dt/duration
            else:
                t_pct = 1.1  # meaning we are already there

            if t_pct > 1:
                # animation done
                t_pct = 1.0
                self.animations.pop(0)

            # pos, linear interpolation
            p = euclid.Point3.new_interpolate(anim[0], anim[1], t_pct)
            self.pos = p
            # FreeCAD.Console.PrintMessage('!')
            # rot, SLERP interpolation
            q = euclid.Quaternion.new_interpolate(anim[2], anim[3], t_pct)
            self.orient = q

            # TODO: optimize for speed
            pl = FreeCAD.Placement(FreeCAD.Vector(p.x, p.y, p.z),
                                   FreeCAD.Rotation(q.x, q.y, q.z, q.w))
            self.rob.Tcp = pl.multiply(self.rob.Tool.inverse())

            # this is how to manually compensate for the tool trans
            # r.rob.Tcp = r.path.Waypoints[0].Pos.multiply(r.rob.Tool.inverse())

        # FreeCAD.Console.PrintMessage('>')


