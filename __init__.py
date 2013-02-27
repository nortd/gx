
from form import *

# reload dependencies when being reloaded
try:
    reload(form)
    from form import *
except ImportError:
	pass
