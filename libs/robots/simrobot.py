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
from gx.libs.robots.trajectory import Target, Trajectory

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
        self.tcp_pos = euclid.Point3()          # actual TCP pos
        self.tcp_rot = euclid.Quaternion()      # actual TCP rot
        self.tool_pos = euclid.Vector3()
        self.tool_rot = euclid.Quaternion()
        self.tool_pos_inv = euclid.Vector3()
        self.tool_rot_inv = euclid.Quaternion()

        # work frame
        self._workframe = Pose(V(), R())

        # trajectory
        self.trajectory = Trajectory()
        self.rot_now = R()
        self.velocity_now = 100

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

        # init pose
        self.axes = (0,0,0,0,0,0)
        _p = self.rob.Tcp.Base
        self.pos = P(_p.x, _p.y, _p.z)
        # self.toolrotate_to(0, math.pi/2.0, 0)
        forward = aR(pi/2, V(0,1,0))
        self.rot = forward



    # ###########################################
    # Posing

    # rot properties
    @property
    def rot(self):
        return self.tcp_rot  # TODO: invalid after setting axes
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
        return self.tcp_pos  # TODO: invalid after setting axes
    @pos.setter
    def pos(self, p):
        if isinstance(p, P) or isinstance(p, V):
            if isinstance(p, V):  #relative
                p += self.tcp_pos
            self.tcp_pos = p
            _q = self.rob.Tcp.Rotation.Q
            q = euclid.Quaternion(_q[3], _q[0], _q[1], _q[2])
            p_m = p + (q * self.tool_pos_inv)
            self.rob.Tcp.Base = (p_m.x, p_m.y, p_m.z)
        else:
            print "ERROR: invalid type"


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
        # TODO: update self.tcp_pos, self.tcp_rot


    # tool rotation shortcuts
    def rotxyz(self, ang_x, ang_y, ang_z):
        """Set tool rotation by axis-aligned angles (relative).

        ang_x: angle around x-axis in radians
        ang_y: angle around y-axis in radians
        ang_z: angle around z-axis in radians
        """
        qx = aR(ang_x, euclid.Vector3(1, 0, 0))
        qy = aR(ang_y, euclid.Vector3(0, 1, 0))
        qz = aR(ang_z, euclid.Vector3(0, 0, 1))
        self.rot = qx * qy * qz * self.rot


    def rotxyz_to(self, ang_x, ang_y, ang_z):
        """Set tool rotation by axis-aligned angles (absolute).

        ang_x: angle around x-axis in radians
        ang_y: angle around y-axis in radians
        ang_z: angle around z-axis in radians
        """
        qx = euclid.Quaternion.new_rotate_axis(ang_x, euclid.Vector3(1, 0, 0))
        qy = euclid.Quaternion.new_rotate_axis(ang_y, euclid.Vector3(0, 1, 0))
        qz = euclid.Quaternion.new_rotate_axis(ang_z, euclid.Vector3(0, 0, 1))
        self.rot = qx * qy * qz


    def pose(self, pose):
        """Pose the tool by setting position and rotation."""
        self.rot = pose.rot
        self.pos = pose.pos


   
    # ###########################################
    # Trajectory

    def path(self, path):
        """Set path of the robot."""

        for command in path:
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
            elif typ == "axistarget":
                axes = command[1]
                dur = command[2]
            elif typ == "tool":
                name = command[1]
                vals = path.gettool(name)
                pos - vals[0]
                rot = vals[1]
                mass = vals[2]
                massCenterPos = vals[3]
                modelFile = vals[4]
                modelPos = vals[5]
                modelRot = vals[6]
            elif typ == "frame":
                name = command[1]
                vals = path.getframe(name)
                pos = vals[0]
                rot = vals[1]
            elif typ == "speed":
                name = command[1]
                vals = path.getspeed(name)
                lin = vals[0]
                rot = vals[1]
            elif typ == "zone":
                name = command[1]
                vals = path.getzone(name)
                radius = vals[0]
            else:
                raise Exception("invalid command type")



    def generate_freecad_trajectory(self):
        """Generate a FreeCAD trajectory.

        This allows for using the build-in robot simulation as opposed
        to the gx simulation. Native simulation may be useful for
        debugging. To use this, select robot and this trajectory from
        FreeCAD's tree view and press the simulate button.
        """
        doc = FreeCAD.activeDocument()
        traj = doc.addObject("Robot::TrajectoryObject","Trajectory").Trajectory
        for target in self.trajectory:
            pos = target.pos
            rot = target.rot
            velocity = target.linspeed
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
            # rot = euclid.Quaternion.new_rotate_axis(ang_x*TO_RAD, euclid.Vector3(1, 0, 0))
            qx = euclid.Quaternion.new_rotate_axis(-90*TO_RAD, euclid.Vector3(1, 0, 0))
            qy = euclid.Quaternion.new_rotate_axis(i*20*TO_RAD, euclid.Vector3(0, 1, 0))
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

        TODO: mass, center of mass
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
        self.tool_pos = euclid.Point3(p[0], p[1], p[2])
        q = tplace.Rotation.Q
        self.tool_rot = euclid.Quaternion(q[3], q[0], q[1], q[2])
        # FreeCAD.Console.PrintMessage(self.tool_rot.get_angle_axis())
        #inverse
        tplace_inv = self.rob.Tool.inverse()
        pinv = tplace_inv.Base
        self.tool_pos_inv = euclid.Point3(pinv[0], pinv[1], pinv[2])
        qinv = tplace_inv.Rotation.Q
        self.tool_rot_inv = euclid.Quaternion(qinv[3], qinv[0], qinv[1], qinv[2])
        # FreeCAD.Console.PrintMessage(self.tool_rot_inv.get_angle_axis())
        # FreeCAD.Console.PrintMessage(self.tool_rot.inverse().get_angle_axis())

        # update pos so tool end is new tcp
        self.pos = self.pos


    def frame(self, origin, xpoint, ypoint):
        """Defines a work object frame (coordinate system).

        TODO: workframe is not used yet!

        Creates a work object pose in origin with the x-axis pointing
        from origin to xpoint. Then ypoint is used to determin the 
        y-direction (only). The z-direction follows from the right-hand-rule.
        """
        x = (origin - xpoint).normalized()
        z = ypoint.cross(z).normalized()
        y = x.cross(z)
        m = euclis.Matrix4.new_rotate_triple_axis(x, y, z)
        # m.d, m.h, m.l = origin.x, origin.y, origin.z
        self._workframe.pos = origin
        self._workframe.rot = m.get_quaternion()




    # ###########################################
    # Simbot Animation

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
        """Animation callback, handler of qt timer.

        NOTE: Any bugs in this context are completely silent
              in FreeCAD. To debug, print a message at the end of this
              handler and make sure it will be reached.
        """
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
            self.pos = anim[0]
            self.rot = anim[2]
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

            # rot, SLERP interpolation
            q = euclid.Quaternion.new_interpolate(anim[2], anim[3], t_pct)
            self.rot = q
            # pos, linear interpolation
            p = euclid.Point3.new_interpolate(anim[0], anim[1], t_pct)
            self.pos = p

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
