Generative Expressions Library
==============================

gx is a library for generating geometry in FreeCAD and Rhino,
and deriving fabrication instruction for machine control.


Installation
-------------
The library can be installed into python's "dist-packages"
like any other library or used in place. Latter has the advantage
of being unaffected by changes in future changes in gx. For this
copy the gx directory to a convenient location and start adding 
your scripts and apps to the "apps" sub-directory. 

To add gx permanently to your python search path add a "gx.pth" 
file to your "dist-packages" directory. The file should contain 
the path to gx as a single line.


open-abb-driver setup
---------------------

- connect ethernet to ABB controller service port
- configure computer ethernet device to something like:
  - IP: 192.168.125.80
  - Netmask: 255.255.255.0
- ftp-connect to ABB controller (any non-blank user/password)
- upload SERVER.mod/pgf and DummyModule.mod/pgf
- by Teach-Pendant load the SERVER.pgf
- load the dummy file to unused robot in a multi-move system
- put controller in automatic mode (usually key-switch) and turn on motors
- run RAPID program
- connect/control with the robot.py module

