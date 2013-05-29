Generative Expressions Library (alpha)
==============================

gx is a computational design library for generating geometry in FreeCAD and Rhino. It can also control industrial robots for direct fabrication. At the moment this projects is in its early phase and the API is a moving target. It will be a while until we have distilled all the complex concepts down to a super simple interface. If you want to join us in this early phase drop us a line (helloworld at nortd . com).


Howto
-------------
The library can be installed into python's "dist-packages"
like any other library or used in place. Latter has the advantage
of being unaffected by future changes in gx. For this
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
