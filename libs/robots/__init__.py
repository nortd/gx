

_rapidrobot = None
_simrobot = None
_openabbrobot = None
_openabbmultirobot = None


def rapidbot():
	global _rapidrobot
	if not _rapidrobot:
		from gx.libs.robots import rapidrobot
		_rapidrobot = rapidrobot.Robot()
	return _rapidrobot

def simbot():
	global _simrobot
	if not _simrobot:
		from gx.libs.robots import simrobot
		_simrobot = simrobot.Robot()
	return _simrobot


def oadbot():
	global _openabbrobot
	if not _openabbrobot:
		from gx.libs.robots import openabbrobot
		_openabbrobot = openabbrobot.Robot()
	return _openabbrobot

def oadmultibot():
	global _openabbmultirobot
	if not _openabbmultirobot:
		_openabbmultirobot = []
		from gx.libs.robots import openabbrobot
		_openabbmultirobot.append(openabbrobot.Robot(port=5000))
		_openabbmultirobot.append(openabbrobot.Robot(port=6000))
		_openabbmultirobot.append(openabbrobot.Robot(port=7000))
	return _openabbmultirobot