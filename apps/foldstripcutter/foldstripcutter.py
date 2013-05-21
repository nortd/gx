
import os
import math
import time
import gx


# some house cleaning
reload(gx)  # when hacking the gx lib
gx.clear()

simbot = gx.simbot()
# simbot.tool("circular_saw", search_path=os.path.dirname(__file__))
simbot.tool("circular_saw")


# default rot is  flange pointing up, along the z-axis
start_pos = gx.P(1000, 0, 700)
rotDown = gx.aQ(math.pi, gx.V(0,1,0))
rotLeft = gx.aQ(-math.pi/2, gx.V(0,0,1))
start_rot = rotDown*rotLeft
simbot.waypoint(start_pos, start_rot)
simbot.waypoint(gx.P(1000, 200, 700), start_rot)
simbot.waypoint(gx.P(1000, 200, 700), rotDown*gx.aQ(-math.pi, gx.V(0,0,1)))
simbot.waypoint(gx.P(800, 200, 700), rotDown*gx.aQ(-math.pi, gx.V(0,0,1)))
simbot.waypoint(gx.P(800, 200, 700), rotDown*gx.aQ(math.pi/2, gx.V(0,0,1)))
simbot.waypoint(gx.P(800, 0, 700), rotDown*gx.aQ(math.pi/2, gx.V(0,0,1)))
simbot.waypoint(gx.P(800, 0, 700), rotDown*gx.aQ(0, gx.V(0,0,1)))
simbot.waypoint(start_pos, rotDown*gx.aQ(0, gx.V(0,0,1)))
simbot.waypoint(start_pos, start_rot)

simbot.go()

# def animate():
#     simbot.pos = gx.P(1000, 0, 700)
#     time.sleep(0.5)
#     simbot.pos = gx.P(1200, 200, 700)
#     time.sleep(0.5)
#     simbot.pos = gx.P(800, 200, 700)
#     time.sleep(0.5)
#     simbot.pos = gx.P(800, 0, 700)
#     time.sleep(0.5)
#     simbot.pos = gx.P(1000, 0, 700)

#     from PyQt4 import QtCore
#     from FreeCAD import Base

#     loop=QtCore.QEventLoop()
#     timer=QtCore.QTimer()
#     timer.setSingleShot(True)
#     QtCore.QObject.connect(timer,QtCore.SIGNAL("timeout()"),loop,QtCore.SLOT("quit()"))

#     duration = 3000 # in ms
#     steps=300
#     ms=duration/steps
#     obj=App.ActiveDocument.ActiveObject
#     start=obj.Placement.Base
#     end=Base.Vector(12,23,34)
#     for i in range(steps):
#         s=float(i)/steps
#         px=start.x * (1-s) + end.x * s
#         py=start.y * (1-s) + end.y * s
#         pz=start.z * (1-s) + end.z * s
#         obj.Placement.Base=Base.Vector(px,py,pz)
#         timer.start(ms)
#         loop.exec_(QtCore.QEventLoop.ExcludeUserInputEvents)

#     obj.Placement.Base=end
