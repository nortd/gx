
import gx
from gx import pi, P, V, R, aR, Pose


def equals(num1, num2):
    return (num1-num2) < 0.0001

gx.clear()

r = gx.simbot()

### frame change
# -> base pose the same
# -> pose adapts to new frame
p1 = V(500,200,800)
r.pos = p1
assert r.pose.pos == p1
basepose_ = r._get_base_pose()
f1 = V(400,100,700)
r.frame(f1)  # set frame
assert basepose_ == r._get_base_pose()
assert r.pos == p1 - f1

### reset frame
r.frame(V(333,222,999))
fpose_before = r._frame
r.pos = V(0,0,0)
pose_before = r.pose
basepose_ = r._get_base_pose()
r.frame(V(0,0,0))  # reset frame
assert basepose_ == r._get_base_pose()
assert r.pos == fpose_before.pos
assert pose_before.rot == r.pose.rot

### add tool
r.frame(V(300,100,900))
r.pos = V(50,50,50)
basepose_ = r._get_base_pose()
pose_before = r.pose.copy()
r.tool("circular_saw")  # mount tool
# assert basepose_ == r._toolpose_inv * r._get_base_pose()
assert pose_before == r.pose

### reset frame
pose_before = r.pose
r.frame(V(0,0,0))  # reset frame
assert pose_before.rot == r.rot

### rotation
r.frame(V(200,200,400))
r.pose = Pose(V(600,50,500), r.UP)
pose_before = r.pose.copy()
basepose_ = r._get_base_pose()
q = aR(pi/2, V(0,1,0))
r.rot = q  # change rotation
r_rel = pose_before.rot.conjugated() * r.pose.rot
print q.get_angle_axis()
print r_rel.get_angle_axis()
print q.dot(r_rel)
assert q == r_rel
assert pose_before.pos == r.pose.pos
assert equals(basepose_.pos.x, r._get_base_pose().pos.x + r._toolpose.pos.z)
assert equals(basepose_.pos.y, r._get_base_pose().pos.y)
assert equals(basepose_.pos.z, r._get_base_pose().pos.z - r._toolpose.pos.z)

### frame rotation
# rotate principal x-axis to y-axis
r.frame(V(400,0,800), aR(pi/2, V(0,0,1)))
r.pose = Pose(V(1000,0,0), r.RIGHT)
r.frame(V(), R())
pinbase = V(400.0,1000.0,800.0)
assert equals(r.pos.x, pinbase.x)
assert equals(r.pos.y, pinbase.y)
assert equals(r.pos.z, pinbase.z)


print "Tests SUCCESSFUL!"
