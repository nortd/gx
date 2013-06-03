
import os
import sys
import math

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

from gx.libs.vectormath import *
from gx.libs.app import *
from gx.libs.form import *
from gx.libs.robots import *



__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__version__ = '2013.02'
__license__ = 'GPL3'
__docformat__ = 'restructuredtext en'

_appcontext = {}


def _find_main(name, appsloc):
    app_to_run = None
    items = os.listdir(appsloc)
    for item in items:
        if item == name+'.py' and os.path.isfile(os.path.join(appsloc, item)):
            print item
            # got a match
            if not app_to_run:
                app_to_run = os.path.join(appsloc, item)
            else:
                print "WARN: multiple apps with this name"
        elif item == name and os.path.isdir(os.path.join(appsloc, item)):
            currentapploc = os.path.join(appsloc, item)
            appitems = os.listdir(currentapploc)
            if name+'.py' in appitems:
                # got a match
                if not app_to_run:
                    app_to_run = os.path.join(currentapploc, name+'.py')
                else:
                    print "WARN: multiple apps with this name"  
            elif 'main.py' in appitems:
                # got a match
                if not app_to_run:
                    app_to_run = os.path.join(currentapploc, 'main.py')
                else:
                    print "WARN: multiple apps with this name"
    return app_to_run


def run(name):
    """Run an app from the apps folder.

    First looks for name+'.py' then for name/name+'.py'
    then for name/main.py

    returns: module vars from the app that are listed
    by _return_. If only one is listed then it is returned
    directly. If many then a dict is returned.
    """
    global _appcontext
    app_to_run = None
    thisloc = os.path.dirname(os.path.realpath(__file__))
    app_to_run = _find_main(name, os.path.join(thisloc, 'apps'))
    if not app_to_run:
        app_to_run = _find_main(name, os.path.join(thisloc, 'examples'))
    if not app_to_run:
        app_to_run = _find_main(name, os.path.join(thisloc, 'tests'))
    if app_to_run:
        print "INFO: running " + app_to_run
        globals_other = {}
        # exec open(app_to_run).read()
        execfile(app_to_run, globals_other)
        _appcontext = globals_other
        # _appcontext = globals_other
        # if globals_other.has_key('_return_'):
        #   retlist = globals_other['_return_']
        #   if len(retlist) == 1:
        #       return globals_other.get(retlist[0])
        #   elif len(retlist) > 1:
        #       return dict((k, globals_other[k]) for k in retlist)
    else:
        print "ERROR: app not found"


def get(name):
    return _appcontext.get(name)