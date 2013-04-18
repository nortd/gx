
import gx

print gx.settings.VERSION

c = gx.random_curve(12)
c.scale(400)
c.translate(600,0,1000)

r = gx.get_rapidrobot()

r.digital_out("do10_1", True)
r.digital_out("do10_2", True)
r.digital_out("do10_3", True)
r.digital_out("do10_4", True)
r.digital_out("do10_5", True)
r.digital_out("do10_6", True)
r.digital_out("do10_7", True)
r.digital_out("do10_8", True)
r.digital_out("do10_9", True)
r.digital_out("do10_10", True)
r.digital_out("do10_11", True)
r.digital_out("do10_12", True)
r.add_curve(c, 6)
# r.digital_out("do10_1", False)

import os
thislocation = os.path.dirname(os.path.realpath(__file__))
filepath = os.path.join(thislocation, '..', 'temp', 'T_ROB1.mod')
r.filedump(filepath)

r.upload()