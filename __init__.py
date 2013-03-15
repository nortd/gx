
import sys

import settings

def _force_reload_of_gx():
	keys_to_delete = []
	for key in sys.modules:
		if key[:3] == 'gx.':
			keys_to_delete.append(key)
	for key in keys_to_delete:
		del sys.modules[key]

if settings.DEBUG:
	_force_reload_of_gx()

from gx.libs.app import *
from gx.libs.form import *
from gx.libs.robots import *



__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__version__ = '2013.02'
__license__ = 'GPL3'
__docformat__ = 'restructuredtext en'

