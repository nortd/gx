
from gx.libs.euclid import euclid


__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__all__ = ['P', 'iP', 'V', 'iV', 'R', 'aR', 'eR', 'mR', 'iR',
           'M', 'tM', 'sM', 'aM', 'eM', 'xM', 'yM', 'zM',
           'pose_multiply']



def pose_multiply(pos1, rot1, pos2, rot2):
	pos = pos1 + rot1 * pos2
	rot = rot1 * rot2
	return pos, rot


# ############################################################################
# Aliases and Factories

# point, position
P = euclid.Point3                           # x, y, z
iP = euclid.Point3.new_interpolate          # p1, p2, t
# vector
V = euclid.Vector3                          # x, y, z
iV = euclid.Vector3.new_interpolate         # v1, v2, t
# rotation
R = euclid.Quaternion                       #
aR = euclid.Quaternion.new_rotate_axis      # angle, axis
eR = euclid.Quaternion.new_rotate_euler     # heading, attitude, bank
mR = euclid.Quaternion.new_rotate_matrix    # mat
iR = euclid.Quaternion.new_interpolate      # q1, q2, t
# xR = euclid.Matrix4.new_rotatex             # angle
# yR = euclid.Matrix4.new_rotatey             # angle
# zR = euclid.Matrix4.new_rotatez             # angle
# matrix
M = euclid.Matrix4                          #
tM = euclid.Matrix4.new_translate           # x, y, z
sM = euclid.Matrix4.new_scale               # x, y, z
aM = euclid.Matrix4.new_rotate_axis         # angle, axis
eM = euclid.Matrix4.new_rotate_euler        # angle_x, angle_y, angle_z
xM = euclid.Matrix4.new_rotatex             # angle
yM = euclid.Matrix4.new_rotatey             # angle
zM = euclid.Matrix4.new_rotatez             # angle
