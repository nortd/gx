"""

robot = openabbrobot.Robot()
robot = abbrapidwriter.Robot()

"""



class Robot():


	def connect(self): pass
	def move_linear(self, pose): pass
	def speed(): pass

	def follow_curve(self, curve, num=10, orientation=[0,0,1,0], robot=1):
		if not isinstance(curve, form.BaseForm):
			print "error, not a form object"
			return

		if not curve.is_curve():
			print "error, not a curve object"
			return

		points = curve.tessellate(num)
		for pt in points:
			self.pose([pt,orientation], robot)
