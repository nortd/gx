
from gx.libs.vectormath import *


class Target(object):
    """A robot pose with motion-relevant definitions.

    A robot target is typically a tool postion and rotation 
    (inverse kinematics) but it can also be a pose based on joint 
    angles (forward kinematics). 

    Additionally a Target defines how to get there, at what speed 
    or what time, how to blend into the following trajectory, transforms
    to compensate for tools and work object, and postion of possible 
    external axes.

    inter: the path interpolation of how to get to this target, can be 
           LIN (linear), PTP (joint).
    pose: the position and crotation of the tool
    axes: six values for the robot's axes. 
          This overwrites and pos/rot, LIN/PTP pose.
    linspeed: velocity of tool center point
    rotspeed: reorientation speed in radians/sec. (only used when no lin motion)
    dur: duration of motion (overwrites speed definitions)
    zone: how precise to reach targets dynamically. Valid presets are:
          'fine', 'z0', 'z1', 'z5', 'z10', ... 'z100' which lead to corner
          displacements of none, 0.3mm, 1mm, 5mm, 10mm, ... 100mm
    toolxform: the transform from robot end point to tool center point
    workxform: the transform from world frame to work object frame

    HOW TO USE:
    Most arguments are optional. For example all of the following is valid:
    - Pose(pos=P(0,0,0))
    - Pose(rot=eR(pi/2,0,0))
    - Pose(axes=(0,0,0,0,pi/2,0))

    When a target is composed into a path, previous values are stored and used 
    until changed by a successive targets (state machine). This is to say, 
    omitted args default to the previous values.
    """
    def __init__(self, pos=None, rot=None, axes=None, inter=None, linspeed=None, 
                 rotspeed=None, dur=None, zone=None, toolxfrom=None, workxform=None):
        """Pass args that have changed from previous pose."""

        self.pose = pose    # Pose()
        self.axes = axes    # (0,0,0,0,0,0)
        self.inter = inter  # LIN, PTP
        self.linspeed = linspeed    # (mm/sec)
        self.rotspeed = rotspeed    # (rad/sec)
        self.dur = dur      # seconds
        self.zone = zone    # fine, z0, z1, z5, z10, ...    
        self.toolxform = toolxform  # 'tool0'
        self.workxform = workxform  # 'wobj0', (user and object frame compined)
        ### future stuff
        # self.externalAxes = externalAxes  # 'externalAxis0'
        # self.extlinspeed
        # self.extrotspeed



class Trajectory(object):
    """A trajectory defines a dynamic motion.

    A series of targets (defined both by forward and inverse kinematics)
    compose a trajectory. This includes dynamic properties like speed 
    and motion blend definitions.
    """
    def __init__(self):
        self.targets = []
        self.pose_state = Pose()
        self.axes_state = (0,0,0,0,0,0)
        self.inter_state = 'LIN'  # LIN, PTP
        self.linspeed_state = 200
        self.rotspeed_state = pi/16.0
        self.dur_state = 0
        self.zone_state = "z0"
        self.tool_xform_state = M()
        self.work_xform_state = M()
        # self.externalAxes_state = [9e9, 9e9, 9e9, 9e9, 9e9, 9e9]

        # speed definitions
        self.speeddefs = {}
        v100 = 

        # zone definitions
        self.zonedefs = {}

        # tool definitions
        self.tooldefs = {}

        # work xform definitions
        self.workxformdefs = {}



    def addtarget(self, target):
        """Add a targe to the trajectory.

        All parameters but the position are optional. They only need to 
        be supplied when they change.

        point: position, a euclid point
        orient: orientation, a euclid quaternion
        type_: type of movement, defaults to LIN
        name: name of waypoint
        velocity: how fast to get to waypoint
        continuity: pass-by or not
        """
        if orient:
            self.orient_now = orient
        if velocity:
            self.velocity_now = velocity
        if type_:
            self.type_now = type_
        if name:
            self.name_now = name
        if continuity:
            self.continuity_now = continuity

        self.targets.append(target)


    def add_workxform(self, varname, holdingWork=False, fixed=True, unitExt="",
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

