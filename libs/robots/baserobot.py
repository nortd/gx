"""

robot = openabbrobot.Robot()
robot = rapidrobot.Robot()

"""

from gx.libs import form


class Robot(object):

	def __init__(self):
		pass

	def connect(self): pass
	def move_linear(self, pose): pass
	def speed(): pass
	def reset(): pass

	def add_curve(self, curve, num=10, orientation=[0,0,1,0], robot=1):
		# if not isinstance(curve, form.BaseForm):
		# 	print "error, not a form object"
		# 	return

		if not curve.is_curve():
			print "error, not a curve object"
			return

		points = curve.tessellate(num)
		first = True
		for pt in points:
			if first:
				self.move_joint(list(pt), orientation, robot)
				first = False
			else:
				self.move_linear(list(pt), orientation, robot)
