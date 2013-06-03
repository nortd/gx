
import math
from gx.libs.euclid import euclid


__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__all__ = ['pi', 'TO_RAD', 'TO_DEG', 'Pose',
           'P', 'iP', 'V', 'iV', 'R', 'aR', 'eR', 'mR', 'iR',
           'M', 'tM', 'sM', 'aM', 'eM', 'xM', 'yM', 'zM']


pi = math.pi
TO_RAD = math.pi/180.0
TO_DEG = 180.0/math.pi


# point (or position)
P = euclid.Point3                           # x, y, z
iP = euclid.Point3.new_interpolate          # p1, p2, t
# vector (or move)
V = euclid.Vector3                          # x, y, z
iV = euclid.Vector3.new_interpolate         # v1, v2, t
# rotation (or orientation)
R = euclid.Quaternion                       #
aR = euclid.Quaternion.new_rotate_axis      # angle, axis
eR = euclid.Quaternion.new_rotate_euler     # heading, attitude, bank
mR = euclid.Quaternion.new_rotate_matrix    # mat
iR = euclid.Quaternion.new_interpolate      # q1, q2, t
# xR = euclid.Matrix4.new_rotatex             # angle
# yR = euclid.Matrix4.new_rotatey             # angle
# zR = euclid.Matrix4.new_rotatez             # angle
# matrix (transform)
M = euclid.Matrix4                          #
tM = euclid.Matrix4.new_translate           # x, y, z
sM = euclid.Matrix4.new_scale               # x, y, z
aM = euclid.Matrix4.new_rotate_axis         # angle, axis
eM = euclid.Matrix4.new_rotate_euler        # angle_x, angle_y, angle_z
xM = euclid.Matrix4.new_rotatex             # angle
yM = euclid.Matrix4.new_rotatey             # angle
zM = euclid.Matrix4.new_rotatez             # angle
# pose (position, rotation transform)
# Pose = Pose

class Pose(object):
    """A transform of position and rotation.

    This transform acts similar to a Matrix but is limited to
    translation and rotation. It also based on a translation vector
    and a rotation quaternion and allows for smooth linear and
    spherical linear (SLERP) interpolation.

    - (pose * pose) compines transformations
    - (pose =* pose) compines transformations in-place
    - (pose * vector) applies the transformation to a vector
    - (pose * point) applies the transformation to a point
    """

    def __init__(self, pos=V(), rot=R()):
        self.pos = pos
        self.rot = rot


    def __copy__(self):
        return self.__class__(pos.copy(), rot.copy())

    copy = __copy__


    def __repr__(self):
        return 'Pose(%s, %s)' % (str(self.pos), str(self.rot))


    def __mul__(self, other):
        """Multiply pose with other pose, point, or vector.

        (pose * pose2) results in a compound pose
        (pose * vector) results in a vector rotated by pose
        (pose * point) results in a point translated and rotate
        """
        if isinstance(other, Pose):
            # return compound pose
            pos = self.pos + (self.rot * other.pos)
            rot = self.rot * other.rot
            return Pose(pos, rot)
        elif isinstance(other, euclid.Vector3):
            # return point transformed by this pose
            np = self.rot * (self.pos + other)
            return other.__class__(np.x, np.y, np.z)
        # elif isinstance(other, euclid.Vector3):
        #     # return vector transformed by this pose
        #     # Note: vectors are not translated, only rotated
        #     return self.rot * other
        else: 
            assert isinstance(other, Pose) or \
                   isinstance(other, euclid.Vector3)


    def __imul__(self, other):  # pose =* other
        """Multiply pose with other pose in-place.

        Same as (pose = pose * pose2) but optimized
        """
        if isinstance(other, Pose):
            # return compound pose
            self.pos = self.pos + (self.rot * other.pos)
            self.rot = self.rot * other.rot
            return self
        else: 
            assert isinstance(other, euclid.Pose)

    def inverse(self):
        return Pose(-self.pos, 
            euclid.Quaternion(-self.rot.w, self.rot.x, self.rot.y, self.rot.z))

    @classmethod
    def new_translate(cls, x, y, z):
        return cls(euclid.Vector3(x,y,z), euclid.Quaternion())

    @classmethod
    def new_rotate(cls, rot):
        return cls(euclid.Vector3(), rot)

    @classmethod
    def new_rotate_axis(cls, angle, axis):
        return cls(euclid.Vector3(), euclid.Quaternion.new_rotate_axis(angle, axis))

    @classmethod
    def new_rotate_euler(cls, heading, attitude, bank):
        return cls(euclid.Vector3(), euclid.Quaternion.new_rotate_euler(heading, attitude, bank))

    @classmethod
    def new_interpolate(cls, pose1, pose2, t):
        assert isinstance(pose1, Pose) and isinstance(pose2, Pose)
        pos = euclid.Vector3.new_interpolate(pose1.pos, pose2.pos, t)
        rot = euclid.Quaternion.new_interpolate(pose1.rot, pose2.rot, t)
        return cls(pos, rot)



tPose = Pose.new_translate                  # x, y, z
rPose = Pose.new_rotate                     # q
aPose = Pose.new_rotate_axis                # angle, axis
ePose = Pose.new_rotate_euler               # heading, attitude, bank
iPose = Pose.new_interpolate                # pose1, pose2, t