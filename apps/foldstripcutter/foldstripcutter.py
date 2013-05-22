
import gx
reload(gx)  # when hacking the gx lib
from gx import P, V, aR, pi

# some house cleaning
gx.clear()

simbot = gx.simbot()
# simbot.tool("circular_saw", search_path=os.path.dirname(__file__))
simbot.tool("circular_saw")

start_pos = P(1000, 0, 700)
rotDown = aR(pi, V(0,1,0))
rotLeft = aR(-pi/2, V(0,0,1))
start_rot = rotDown*rotLeft
simbot.waypoint(start_pos, start_rot)
simbot.waypoint(P(1000, 200, 700), start_rot)
simbot.waypoint(P(1000, 200, 700), rotDown * aR(-pi, V(0,0,1)))
simbot.waypoint(P(800, 200, 700), rotDown * aR(-pi, V(0,0,1)))
simbot.waypoint(P(800, 200, 700), rotDown * aR(pi/2, V(0,0,1)))
simbot.waypoint(P(800, 0, 700), rotDown * aR(pi/2, V(0,0,1)))
simbot.waypoint(P(800, 0, 700), rotDown * aR(0, V(0,0,1)))
simbot.waypoint(start_pos, rotDown * aR(0, V(0,0,1)))
simbot.waypoint(start_pos, start_rot)

simbot.go()