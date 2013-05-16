
import os
import gx


# some house cleaning
reload(gx)  # when hacking the gx lib
gx.clear()

simbot = gx.simbot()
simbot.tool("circular_saw", search_path=os.path.dirname(__file__))

start_pos = gx.P(1177.93, 0.0, 430.462)
start_rot = gx.Q(0,1,0,0)
simbot.waypoint(start_pos, start_rot)
for i in range(7):
	pos = gx.P(i*30+800, 500, -i*50+800)
    qx = gx.aQ(-90*gx.TO_RAD, gx.V(1, 0, 0))
    qy = gx.aQ(i*20*gx.TO_RAD, gx.V(0, 1, 0))
    rot = qx*qy
    simbot.waypoint(pos, rot)
simbot.waypoint(start_pos, start_rot)


