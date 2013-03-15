"""This sets the correct python search path.

Since we want to be able to run apps straight out of the
gx file tree we need to make sure python knows where gx
can be found. If it doesn't know we tell it by calculating
it relative to this file.
"""

import os
import sys

thislocation = os.path.dirname(os.path.realpath(__file__))
gxlocation = os.path.join(thislocation, '..', '..')
if not gxlocation in sys.path:
	sys.path.append(os.path.abspath(gxlocation))