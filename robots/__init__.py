


_openabbrobot = None
_rapidrobot = None

def get_openabbrobot():
	global _openabbrobot
	if not _openabbrobot:
		import openabbrobot
		reload(openabbrobot)
		_openabbrobot = openabbrobot.Robot()
	return _openabbrobot

def get_rapidrobot():
	global _rapidrobot
	if not _rapidrobot:
		import rapidrobot
		reload(rapidrobot)
		_rapidrobot = rapidrobot.Robot()
	return _rapidrobot