
import os
import json
from gx.libs.vectormath import *
from gx.libs.euclid import euclid


class Path(object):
    """A path defines a dynamic motion.

    A series of targets (defined both by forward and inverse kinematics)
    compose a path. This includes dynamic properties like speed 
    and motion blend definitions.
    """
    def __init__(self):
        self.commands = []

        # counter for unique name generation
        self.counter = 0

        # tool definitions
        self.tooldefs = ({},{})
        self.toolchange()  # add default

        # work frame definitions
        self.framedefs = ({},{})
        self.frame()  # add default

        # speed definitions
        self.speeddefs = ({},{})
        self.speed()  # add default

        # zone definitions
        self.zonedefs = ({},{})
        self.zone(0.3)  # add default


    def target(self, pos, rot, inter='LIN', dur=None, signal=None, state=1):
        """Add a targe to the path.

        A target is primarily defined by a location and rotation of the
        tool. in addition, the following properties are used from the
        current state: path interpolation, tool settings, work frame
        transform, speed settings, zone settings.

        inter: determines how the path is calculated between targets.
        Setting this to 'LIN' makes the path linear which is appropriate
        for most tool action. The other kind is 'PTP' which does the
        interpolation based on shortest joint movement. This is typically
        used for repositioning the tool quickly. Joint interpolation also 
        often works when a straight line would otherwise go through an area 
        where the robot can't reach.
        dur: Duration of the move (s). This overwrites any speed settings.
        signal: Name of the signal to switch at this target point.
        state: Signal state: On (1), off (0)
        """
        command = ('target', pos, rot, inter, dur, signal, state)
        self.commands.append(command)


    def axistarget(self, axes=None, dur=None):
        """Add a target based on absolute joint positions.

        axes: List of six angles (rad) starting with the joint
              in the robot base.
        dur: Duration (sec) of the move. This will overwrite any
             other speed settings.
        """
        if len(axes) == 6:
            command = ('axistarget', axes, dur)
            self.commands.append(command)
        else:
            raise Exception("invalid axes length")


    def output(self, signal, state=1, delay=0, wait=0, sync=False):
        """Change state of an output signal.

        signal: Name of the signal.
        state: On (1), off (0), or pulse (3)
        delay: in seconds before setting the signal
        wait: in seconds after setting the signal
        sync: wait until the signal is physically set
        """
        command = ('output', signal, state, delay, wait, sync)
        self.commands.append(command)    


    def input():
        print "not implemented yet"    


    def tool(self, name, search_path=None):
        """Loads a tool definition from file.

        This function adds a tool to the robot based on a tool definition.
        A tool definition is a json file specifying tcp pose, mass, center of
        mass pos, model pose. A model file (.wrl) of the same name is used too.
        name: Corresponds to name.json and name.wrl.
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
        tooldefjson = os.path.join(search_path, name + '.json')
        tooldefwrl = os.path.join(search_path, name + '.wrl')
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

        pos = P()
        rot = R()
        mass = 0.001
        massCenterPos = P()
        modelPos = P()
        modelRot = R()

        # tcp translation (in relation to flange)
        p = data.get('pos')
        if p:
            pos = P(p[0], p[1], p[2])
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
            massCenterPos = P(p[0], p[1], p[2])
        
        # tool translation (in relation to flange)
        p = data.get('modelPos')
        if p:
            modelPos = P(p[0], p[1], p[2])
        # tool rotation (in relation to flange)
        r = data.get('modelRot')
        if r:
            modelRot = R(r[0], r[1], r[2], r[3])

        self.toolchange(pos, rot, mass, massCenterPos, modelFile, modelPos, modelRot)


    def toolchange(self, pos=P(), rot=R(), mass=0.001, massCenterPos=P(0,0,0.001), 
             modelFile=None, modelPos=P(), modelRot=R()):
        """Change the tool of the robot.

        pos,rot: Tool frame transform from robot flange
                 to tool center point and direction.
        mass: mass of tool (kg)
        massCenterPos: Translation from robot flange to 
                       tool center of mass.
        """
        defprops = (Pose(pos, rot), mass, Pose(massCenterPos), 
                    modelFile, Pose(modelPos, modelRot))
        varname = self.match_or_add(self.tooldefs, defprops, 'gxtool')
        command = ('tool', varname)
        self.commands.append(command)


    def frame(self, origin=P(), xpoint=P(1000,0,0), ypoint=P(0,1000,0)):
        """Change the work object frame (coordinate system).

        This is a transform on the base frame (which is a transform on 
        the world frame). A common use case is to set the work frame to 
        a fixed point of the work piece. Then write the program in 
        reference to the work piece.

        The work frame transform is defined by the new origin, a point 
        down the new x-axis, and a point down the y-axis. The ypoint is 
        used to determin the rotation around the x-axis (only). The 
        z-direction follows from the right-hand-rule.
        """
        ### calculate pose
        vx = (xpoint - origin).normalized()
        vy_ = (ypoint - origin).normalized()
        vz = vx.cross(vy_).normalized()
        vy = vz.cross(vx).normalized()
        m = euclid.Matrix4.new_rotate_triple_axis(vx, vy, vz)
        pos = origin
        rot = m.get_quaternion()
        ### correlate with framedefs
        defprops = (Pose(pos, rot),)
        varname = self.match_or_add(self.framedefs, defprops, 'gxframe')
        command = ('frame', varname)
        self.commands.append(command)


    def speed(self, lin=100, rot=500):
        """Change speed of the robot.

        lin: linear speed (mm/s)
        rot: rotational speed (deg/s)
        """
        defprops = (lin, rot)
        varname = self.match_or_add(self.speeddefs, defprops, 'gxspeed')
        command = ('speed', varname)
        self.commands.append(command)


    def zone(self, radius):
        """Change motion blending of the robot.

        The zone radius defines how close the robot moves to each
        target. This directly determines how fast the robot can
        move through a target point towards the next.

        radius: allowable distance to target (mm)
        """
        defprops = (radius,)
        varname = self.match_or_add(self.zonedefs, defprops, 'gxzone')
        command = ('zone', varname)
        self.commands.append(command)


    def match_or_add(self, defdict, value, prefix='gxvar'):
        # check existing entries
        matchkey = []
        for v in value:
            matchkey.append(str(v))
        matchkey = ''.join(matchkey)

        if defdict[0].has_key(matchkey):
            namekey = defdict[0][matchkey][0]
            return namekey
        else:
            # add entry
            self.counter += 1
            newnamekey = prefix + str(self.counter)
            defdict[0][matchkey] = (newnamekey,) + value
            defdict[1][newnamekey] = value
            return newnamekey


    def gettool(self, key):
        return self.tooldefs[1][key]

    def getframe(self, key):
        return self.framedefs[1][key]

    def getspeed(self, key):
        return self.speeddefs[1][key]

    def getzone(self, key):
        return self.zonedefs[1][key]