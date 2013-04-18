
import gx
reload(gx)

gx.clear()


def panel_curve(z, num=10, panel_width=1):
	# width direction is z

	c = gx.random_curve(6, xr=(-3,3), yr=(0,1), zr=(z-0.2,z+0.2))
	# c.scale(400)
	# c.translate(700,0,1000)
	length = c.length()
	points = c.tessellate(10)

	panel_length = length/float(num)


	# for i in xrange(num):
	# 	t = i/float(num-1)
	# 	point = c.value_at(t)
	# 	tangent = c.tangent_at(t)
	# 	gx.line(point, (point+tangent.multiply(length/10)))


	t = 0
	t_inc = 1/float(num)
	limit = 100
	while(t < 1 and limit > 0):
		p1 = c.value_at(t)
		tangent = c.tangent_at(t)
		p2 = p1 + tangent.multiply(panel_length)

		# try:
		# 	t = c.closest_curve_point(p2)
		# except:
		# 	gx.warn("failed to find closest point")
		# 	break
		# p3 = c.value_at(t)

		t += t_inc
		p3 = c.value_at(t)
		# gx.line(p1,p2)
		# gx.line(p2,p3)

		# create panel
		# pq1 = p1 + (0, 0, 0.5*panel_width)
		# pq2 = p1 + (0, 0, -0.5*panel_width)
		# pq3 = p2 + (0, 0, -0.5*panel_width)
		# pq4 = p2 + (0, 0, 0.5*panel_width)
		pq1 = [p1[0], p1[1], p1[2] + 0.5*panel_width]
		pq2 = [p1[0], p1[1], p1[2] + -0.5*panel_width]
		pq3 = [p2[0], p2[1], p2[2] + -0.5*panel_width]
		pq4 = [p2[0], p2[1], p2[2] + 0.5*panel_width]
		l1 = gx.line(pq1,pq2)
		l2 = gx.line(pq2,pq3)
		l3 = gx.line(pq3,pq4)
		l4 = gx.line(pq4,pq1)
		wire = Part.Wire(Part.__sortEdges__([l1.obj.Shape.Edge1, l2.obj.Shape.Edge1, l3.obj.Shape.Edge1, l4.obj.Shape.Edge1]))
		face = Part.Face(wire)
		Part.show(face)

		pq1 = [p2[0], p2[1], p2[2] + 0.5*panel_width]
		pq2 = [p2[0], p2[1], p2[2] + -0.5*panel_width]
		pq3 = [p3[0], p3[1], p3[2] + -0.5*panel_width]
		pq4 = [p3[0], p3[1], p3[2] + 0.5*panel_width]
		l1 = gx.line(pq1,pq2)
		l2 = gx.line(pq2,pq3)
		l3 = gx.line(pq3,pq4)
		l4 = gx.line(pq4,pq1)
		wire = Part.Wire(Part.__sortEdges__([l1.obj.Shape.Edge1, l2.obj.Shape.Edge1, l3.obj.Shape.Edge1, l4.obj.Shape.Edge1]))
		face = Part.Face(wire)
		Part.show(face)

		limit -= 1

panel_curve(0)
panel_curve(1)
panel_curve(2)
panel_curve(3)