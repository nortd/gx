"""Connects to an ABB robot through the open-abb-driver.

The open-abb-driver is a RAPID program that runs on the ABB
controller (IRC5) and opens a socket connection. through
this socket it receives commands that get mapped to RAPID
command pretty much one-to-one.
"""

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
        self.rob.RobotVrmlFile = FreeCAD.getResourceDir()+"Mod/Robot/Lib/Kuka/kr500_1.wrl"
        self.rob.RobotKinematicFile = FreeCAD.getResourceDir()+"Mod/Robot/Lib/Kuka/kr500_1.csv"
        # Kuka-specific
        self.rob.Axis2 = -90
        self.rob.Axis3 = 90

        # create a trajectory
        self.trajects = []
        t = doc.addObject("Robot::TrajectoryObject","Trajectory").Trajectory
        startTcp = self.rob.Tcp
        t.insertWaypoints(startTcp)
        for i in range(7):
            t.insertWaypoints(fcRobot.Waypoint(FreeCAD.Placement(FreeCAD.Vector(2000,1000,i*100+1000),FreeCAD.Vector(0,1,0),i),"LIN","Pt"))
        t.insertWaypoints(startTcp) # end point of the trajectory
        # App.activeDocument().Trajectory.Trajectory = t
        self.trajects.append(t)

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
    def axis1(self, value):
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

