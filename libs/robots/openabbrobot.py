"""Connects to an ABB robot through the open-abb-driver.

The open-abb-driver is a RAPID program that runs on the ABB
controller (IRC5) and opens a socket connection. Through
this socket it receives commands that get mapped to RAPID
command pretty much one-to-one.
"""

from gx.libs import form
from gx.libs.robots import baserobot
from gx.libs.oad import robot as oadrobot
    

ABB_CONTROLLER_IP = "192.168.125.1"


class Robot(baserobot.Robot):

    def __init__(self, ip=ABB_CONTROLLER_IP, port=5000):
        baserobot.Robot.__init__(self)
        self.oadrobot = oadrobot.Robot(IP=ip, PORT=port)

    def connect(self):
        self.oadrobot.connect()

    def close(self):
        self.oadrobot.close()

    def move_linear(self, pos, orient, robot=1):
        """Linear move to position and orientation
        """
        self.oadrobot.setCartesian([pos, orient])

    def set_joints(self, axes=[0,0,0,0,90,0]):
        if len(axes) == 6:
            self.oadrobot.setJoints(axes)
        else:
            print("ERROR: invalid axes parameter")
            