from __future__ import print_function  # need print func in lambda
from __future__ import division  # true division from integers


__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__all__ = ['robot_follow_curve']

from oad.robot import Robot
from libs import euclid


def robot_follow_curve(curve_form):
	if curve_form.is_curve():
		verts = curve_form.tessellate(5)
		quat = euclid.

		# Server: 192.168.126.1   # 192.168.126.10
		robot = Robot(IP='192.168.125.1', PORT=5000, verbose=True)
		robot.connect()
		for vert in verts:
			robot.setCartesian([vert,])
		return True
	else:
		return False



class Trajectory(object):
    def __init__(self):

    # ###########################################
    # implemented in FreeCadApp, and RhinoApp
    
    # Logging
    def error(cls, msg): pass
    def warn(cls, msg): pass
    def log(cls, msg): pass
    def message(cls, msg): pass
    # Document Methods
    def active_view(cls): pass
    def refresh_view(cls): pass
    def view_all(cls): pass
    def view_selection(cls): pass
    def clear(cls): pass
    # Selection
    def clear_selection(cls): pass
    # Cursor
    def wait_cursor(cls): pass    # FreeCAD only
    def normal_cursor(cls): pass  # FreeCAD only