"""Connects to an ABB robot through the open-abb-driver.

The open-abb-driver is a RAPID program that runs on the ABB
controller (IRC5) and opens a socket connection. through
this socket it receives commands that get mapped to RAPID
command pretty much one-to-one.
"""

import os
import sys
import math
from gx.libs import form
from gx.libs.robots import baserobot
from gx.libs.euclid import euclid

# Selecting Implementation (FreeCAD or Rhino)
try:
    import FreeCAD
    import Part
    import Draft
    import Robot as fcRobot
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

        # change to Robot Workbench
        FreeCAD.Gui.activateWorkbench("RobotWorkbench")

        # add tool
        self.add_circular_saw()

        # create a trajectory
        self.trajects = []
        t = doc.addObject("Robot::TrajectoryObject","Trajectory").Trajectory
        # startTcp = self.rob.Tcp # = FreeCAD.Placement(FreeCAD.Vector(500,500,500), FreeCAD.Rotation(1,0,0,1))
        startTcp = self.rob.Tcp = FreeCAD.Placement(FreeCAD.Vector (1177.93, 0.0, 430.462), FreeCAD.Rotation(0,1,0,0))
        (0, 1, 0, 0)
        t.insertWaypoints(startTcp)
        for i in range(7):
            # orient = euclid.Quaternion.new_rotate_axis(ang_x*TO_RAD, euclid.Vector3(1, 0, 0))
            qx = euclid.Quaternion.new_rotate_axis(-90*TO_RAD, euclid.Vector3(1, 0, 0))
            qy = euclid.Quaternion.new_rotate_axis(i*20*TO_RAD, euclid.Vector3(0, 1, 0))
            orient = qx*qy
            # FreeCAD.Rotation (-0.479426,-0,-0,0.877583)
            fcOrient = FreeCAD.Rotation(orient.x, orient.y, orient.z, orient.w)
            fcPos = FreeCAD.Vector(i*30+800, 500, -i*50+800)
            t.insertWaypoints(fcRobot.Waypoint(FreeCAD.Placement(fcPos, fcOrient),"LIN","Pt"))
        t.insertWaypoints(startTcp) # end point of the trajectory
        # App.activeDocument().Trajectory.Trajectory = t
        self.trajects.append(t)

        # show all
        FreeCAD.Gui.SendMsgToActiveView("ViewFit")
        FreeCAD.Gui.activeDocument().ActiveView.viewFront()
        doc.recompute()

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
        return euclid.Point3(p[0], p[1], p[2])
    @pos.setter
    def pos(self, p):
        # convert from euclid to FreeCAD
        self.rob.Tcp.Base = (p[0], p[1], p[2])


    # orient property
    @property
    def orient(self):
        # convert from FreeCAD to euclid
        # euclid lib uses (w, x, y, z)
        # FreeCad uses (x, y, z, w)
        q = self.rob.Tcp.Rotation.Q
        return euclid.Quaternion(q[3], q[0], q[1], q[2])
    @orient.setter
    def orient(self, quat):
        # convert form euclid to FreeCAD
        self.rob.Tcp.Rotation.Q = (quat.x, quat.y, quat.z, quat.w)

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



    def add_circular_saw(self):
        thislocation = os.path.dirname(os.path.realpath(__file__))
        vrml_file = os.path.join(thislocation, 'tools', 'circular_saw.wrl')
        FreeCAD.Gui.insert(vrml_file, FreeCAD.activeDocument().Name)
        self.rob.ToolShape = FreeCAD.activeDocument().circular_saw
        self.rob.Tool.Base = (0,-50,225)
        self.rob.ToolBase.Rotation.Q = (0,1,0,1)
        