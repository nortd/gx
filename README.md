Generative Expressions Library
==============================

gx is a library for generating geometry in FreeCAD and Rhino,
and deriving fabrication instruction for machine control.


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
