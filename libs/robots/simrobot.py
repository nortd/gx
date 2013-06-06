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

        # transforms
        self._frame = Pose()            # base -> frame
        self._frame_inv = Pose()        # frame -> base
        self._pose = Pose()             # frame -> TCP
        self._toolpose = Pose()         # flange -> TCP
        self._toolpose_inv = Pose()     # TCP -> flange

        # trajectory
        self._path = None
        self.linspeed_curr = None
        self.rotspeed_curr = None

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
        # NOTE: order is backwards
        # the last quat in multiplication compounds is roatated first
        self.UP = aR(0.00001, V(0,0,0))
        self.FRONT = aR(pi/2.00001, V(0,1,0))
        self.DOWN = aR(pi*1.000001, V(0,1,0))
        self.LEFT = self.FRONT * aR(-pi/2.000001, V(1,0,0))
        self.RIGHT = self.FRONT * aR(pi/2.000001, V(1,0,0))

        # init pose
        self.axes = (0,0,0,0,0,0)
        _p = self.rob.Tcp.Base
        self.pos = V(_p.x, _p.y, _p.z)
        # self.toolrotate_to(0, math.pi/2.0, 0)
        self.rot = self.FRONT
        # self.pose = Pose(self.FLOOR, self.DOWN)



    # ###########################################
    # Posing

    @property
    def pose(self):
        """Returns pose: frame -> TCP"""
        return self._pose
    @pose.setter
    def pose(self, pose):
        """Set pose: frame -> TCP"""
        self._pose = pose
        self._set_base_pose(pose)


    def _set_base_pose(self, pose):
        """Take TCP and set flange pose in base frame."""
        flange =  (self._frame * pose) * self._toolpose_inv
        self.rob.Tcp.Base = (flange.pos.x, flange.pos.y, flange.pos.z)
        self.rob.Tcp.Rotation = FreeCAD.Rotation(flange.rot.x, flange.rot.y, 
                                                 flange.rot.z, flange.rot.w)

    def _get_base_pose(self):
        """Get the flange pose in base frame."""
        _p = self.rob.Tcp.Base
        p = V(_p[0], _p[1], _p[2])
        _r = self.rob.Tcp.Rotation.Q
        r = R(_r[3], _r[0], _r[1], _r[2])
        return Pose(p,r)


    # rot properties
    @property
    def rot(self):
        return self.pose.rot
    @rot.setter
    def rot(self, r):
        self.pose = Pose(self.pose.pos, r)


    # pos property
    @property
    def pos(self):
        return self.pose.pos
    @pos.setter
    def pos(self, p):
        self.pose = Pose(p, self.pose.rot)


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
        self.pose = self._frame_inv *  (self._toolpose * self._get_base_pose())


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



   
    # ###########################################
    # Trajectory

    def path(self, path):
        """Set path of the robot.

        Each command in the path list can be one of the following:
        target: inverse kinematics move
        axistarget: forward kinematics move
        output: set gpio output
        input: read gpio input
        tool: setup an end-effector
        frame: setup a work object frame
        speed: linear and rotational speed settings
        zone: target flyby settings
        """
        self._path = []  # reset
        for command in path.commands:
            typ = command[0]
            if typ == "target":
                pos = command[1]
                rot = command[2]
                inter = command[3]
                dur = command[4]
                self._path.append(('target', Pose(pos,rot), dur, 
                                  self.linspeed_curr, self.rotspeed_curr))
            elif typ == "axistarget":
                # axes = command[1]
                # dur = command[2]
                # self._path.append(axes, dur, rotspeed)
                pass
            elif typ == "output":
                # signal = command[1]
                # state = command[2]
                # delay = command[3]
                # wait = command[4]
                # sync = command[5]
                pass
            elif typ == "input":
                pass
            elif typ == "tool":
                toolname = command[1]
                tooldata = path.gettool(toolname)
                # (Pose(pos, rot), mass, Pose(massCenterPos), modelFile, Pose(modelPos, modelRot))
                self._path.append(('tool',) + tooldata)
            elif typ == "frame":
                frame = command[1]
                framedata = path.getframe(frame)
                # (Pose(pos, rot),)
                self._path.append(('frame',) + framedata)
            elif typ == "speed":
                speed = command[1]
                speeddata = path.getspeed(speed)
                self.linspeed_curr = speeddata[0]
                self.rotspeed_curr = speeddata[1]
                pass
            elif typ == "zone":
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
        frame_curr = Pose()
        for target in self._path:
            command = target[0]
            if command == 'target':
                pose = frame_curr * target[1]
                pos = pose.pos
                rot = pose.rot
                velocity = target[3]
                _pose = FreeCAD.Placement(FreeCAD.Vector(pos.x, pos.y, pos.z), 
                                          FreeCAD.Rotation(rot.x, rot.y, rot.z, rot.w))
                wp = fcRobot.Waypoint(_pose, "LIN", "Pt", velocity, False)
                traj.insertWaypoints(wp)
            elif command == 'frame':
                frame_curr = target[1]




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

        self.toolchange(pos, rot, mass, massCenterPos, modelFile, modelPos, modelRot)


    def toolchange(self, pos=P(), rot=R(), mass=0.001, massCenterPos=P(), 
             modelFile=None, modelPos=P(), modelRot=R()):
        """Change the tool of the robot.

        pos,rot: Tool frame transform from robot flange
                 to tool center point and direction.
        mass: mass of tool (kg)
        massCenterPos: Translation from robot flange to 
                       tool center of mass.
        """
        if modelFile:
            # open, attach wrl file
            FreeCAD.Gui.insert(modelFile, FreeCAD.activeDocument().Name)  #TODO: open file without using Gui
            # self.rob.ToolShape = FreeCAD.activeDocument().circular_saw
            self.rob.ToolShape = FreeCAD.ActiveDocument.Objects[-1]

        # assign to robot
        self.rob.Tool.Base = (pos.x, pos.y, pos.z)
        self.rob.Tool.Rotation = FreeCAD.Rotation(rot.x, rot.y, rot.z, rot.w)
        self.rob.ToolBase.Base = (modelPos.x, modelPos.y, modelPos.z)
        self.rob.ToolBase.Rotation = FreeCAD.Rotation(modelRot.x, modelRot.y, modelRot.z, modelRot.w)

        # store for later use
        self._toolpose = Pose(pos, rot)
        self._toolpose_inv = self._toolpose.inverse()

        # update pose so tool end is new tcp
        self.pose = self.pose


    # def frame(self, origin, xpoint, ypoint):
    #     ### calculate pose
    #     vx = (xpoint - origin).normalized()
    #     vy_ = (ypoint - origin).normalized()
    #     vz = vx.cross(vy_).normalized()
    #     vy = vz.cross(vx).normalized()
    #     m = euclid.Matrix4.new_rotate_triple_axis(vx, vy, vz)
    #     pos = origin
    #     rot = m.get_quaternion()
    #     self.framechange(pos, rot)
    #     # visualize
    #     form.line(origin, origin + (1000*vx))
    #     form.line(origin, origin + (1000*vy))
    #     form.line(origin, origin + (1000*vz))

    def framechange(self, pos, rot=R()):
        """Defines a work object frame (coordinate system)."""
        pose_in_base_frame = self._frame * self._pose
        self._frame = Pose(pos, rot)
        self._frame_inv = self._frame.inverse()
        self._pose = self._frame_inv * pose_in_base_frame



    # ###########################################
    # Simbot Animation

    def go(self):
        """Take path and generate animation."""
        last_pose = None
        for target in self._path:
            # target has (command, pose, dur, linspeed, rotspeed)
            command = target[0]
            if command == 'target':
                pose = target[1]
                dur = target[2]
                linspeed = target[3]
                rotspeed = target[4]
                if last_pose:
                    dist = last_pose.pos.distance(pose.pos)
                    if linspeed == 0:
                        dur = 0.0
                    else:
                        dur = dist/float(linspeed)
                    self.animations.append(['target', last_pose, pose, dur, None])
                last_pose = pose
            elif command == 'tool':
                self.animations.append(target)
            elif command == 'frame':
                self.animations.append(target)


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

        anim = self.animations[0]
        command = anim[0]
        if command == 'target':
            last_pose = self.animations[0][1]
            pose = self.animations[0][2]
            dur = self.animations[0][3]
            state = self.animations[0][4]
            if not state:
                # new motion condition
                # FreeCAD.Console.PrintMessage('*')
                self.animations[0][4] = time.time()  # set state
                self.pose = last_pose
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
                self.pose = Pose.new_interpolate(last_pose, pose, t_pct)

                # this is how to manually compensate for the tool trans
                # r.rob.Tcp = r.path.Waypoints[0].Pos.multiply(r.rob.Tool.inverse())
        elif command == 'tool':
            pose = anim[1]
            mass = anim[2]
            massCenterPose = anim[3]
            modelFile = anim[4]
            modelPose = anim[5]
            self.toolchange(pose.pos, pose.rot, mass, massCenterPose.pos, 
                            modelFile, modelPose.pos, modelPose.rot)
            self.animations.pop(0)
        elif command == 'frame':
            pose = anim[1]
            self.framechange(pose.pos, pose.rot)
            self.animations.pop(0)
            FreeCAD.Console.PrintMessage('((frame')

        FreeCAD.Console.PrintMessage('>')



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
