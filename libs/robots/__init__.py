

_openabbrobot = None
_rapidrobot = None
_simrobot = None

def get_openabbrobot():
	global _openabbrobot
	if not _openabbrobot:
		from gx.libs.robots import openabbrobot
		_openabbrobot = openabbrobot.Robot()
	return _openabbrobot

def get_rapidrobot():
	global _rapidrobot
	if not _rapidrobot:
		from gx.libs.robots import rapidrobot
		_rapidrobot = rapidrobot.Robot()
	return _rapidrobot

def get_simrobot():
	global _simrobot
	if not _simrobot:
		from gx.libs.robots import simrobot
		_simrobot = simrobot.Robot()
	return _simrobot