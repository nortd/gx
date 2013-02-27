import robot
import transformations as tr
import numpy as np
r = robot.Robot()
r.setCartesian([[1000,0,1000], [0,0,1,0]]) #quat [0,0,1,0] is tool
flange pointed down
quat_down = [0,0,1,0]
quat_stuff = tr.quaternion_multiply(quat_down,
tr.quaternion_about_axis(np.radians(30), [0,1,0])) #rotate 30 degrees
around the Y axis
r.setCartesian([[1000,0,1000], quat_stuff])