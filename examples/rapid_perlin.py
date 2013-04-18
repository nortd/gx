
import gx
from gx.libs.noise.perlin import SimplexNoise

pgen = SimplexNoise()
# pgen.noise2(x,y) generates a value between -1 and 1

r = gx.get_rapidrobot()
first = True
orient = [0,0,1,0]
tx = 0
ty = 0
tstep = 0.01
scale = 200
xoffset = 800
yoffset = 0
for i in range(200):
	pos = [pgen.noise2(tx,0)*scale+xoffset, pgen.noise2(0,ty)*scale+yoffset, 1000]
	if first:
		r.move_joint(pos, orient)
		first = False
	r.move_linear(pos, orient)
	tx += tstep
	ty += tstep

import os
thislocation = os.path.dirname(os.path.realpath(__file__))
filepath = os.path.join(thislocation, '..', 'temp', 'T_ROB1.mod')
r.filedump(filepath)

# r.upload()