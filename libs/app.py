from __future__ import print_function  # need print func in lambda
from __future__ import division  # true division from integers

import sys

from gx import settings



__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__all__ = ['error', 'warn', 'log', 'message',
           'active_view', 'refresh_view', 'view_all', 'view_selection', 'clear',
           'clear_selection', 'wait_cursor', 'normal_cursor', 'say']

def say():
    print("say7")

# ############################################################################
# General Implementation

class BaseApp(object):
    def __init__(self):
        pass
        
    # ###########################################
    # implemented in FreeCadApp, and RhinoApp
    
    # Logging
    def error(cls, msg): pass
    def warn(cls, msg): pass
    def log(cls, msg): pass
    def message(cls, msg): pass
    # Document Methods
    def active_view(cls): pass
    def refresh_view(cls): pass
    def view_all(cls): pass
    def view_selection(cls): pass
    def clear(cls): pass
    # Selection
    def clear_selection(cls): pass
    # Cursor
    def wait_cursor(cls): pass    # FreeCAD only
    def normal_cursor(cls): pass  # FreeCAD only




# ############################################################################
# FreeCAD Implementation

class FreeCadApp(BaseApp):
    def __init(self):
        BaseApp.__init__(self)

    # ###########################################
    # Logging
    @classmethod
    def error(cls, msg):
        FreeCAD.Console.PrintError(msg)

    @classmethod
    def warn(cls, msg):
        FreeCAD.Console.PrintWarning(msg)

    @classmethod
    def log(cls, msg):
        FreeCAD.Console.PrintLog(msg)

    @classmethod
    def message(cls, msg):
        FreeCAD.Console.PrintMessage(msg)


    # ###########################################
    # Document Methods

    @classmethod
    def get_active_document(cls):
        return FreeCAD.ActiveDocument

    @classmethod
    def get_object(cls, name):
        return FreeCAD.ActiveDocument.getObject(name)

    @classmethod
    def get_view_object(cls, name):
        """Get the view representation part of the object."""
        if hasattr(FreeCAD, 'Gui'):
            return FreeCAD.ActiveDocument.getObject(name).ViewObject
        else:
            return None

    @classmethod
    def active_view(cls):
        if hasattr(FreeCAD, 'Gui'):
            return FreeCAD.Gui.ActiveDocument.ActiveView
        else:
            return None

    @classmethod
    def refresh_view(cls):
        if hasattr(FreeCAD, 'Gui'):
            FreeCAD.ActiveDocument.recompute()

    @classmethod
    def view_all(cls):
        FreeCAD.Gui.SendMsgToActiveView("ViewFit")

    @classmethod
    def view_selection(cls):
        if hasattr(FreeCAD, 'Gui'):
            FreeCAD.Gui.SendMsgToActiveView("ViewSelection")

    @classmethod
    def clear(cls):
        doc = FreeCAD.ActiveDocument
        if doc:
            for obj in FreeCAD.ActiveDocument.Objects:
                FreeCAD.ActiveDocument.removeObject(obj.Name)



    # ###########################################
    # Selection

    @classmethod
    def clear_selection(cls):
        if hasattr(FreeCAD, 'Gui'):
            FreeCAD.Gui.Selection.clearSelection()

    # ###########################################
    # Cursor

    @classmethod
    def wait_cursor(cls):
        if hasattr(FreeCAD, 'Gui'):
            from PyQt4 import QtCore,QtGui
            QtGui.qApp.setOverrideCursor(QtCore.Qt.WaitCursor)
    @classmethod
    def normal_cursor(cls):
        if hasattr(FreeCAD, 'Gui'):
            from PyQt4 import QtCore,QtGui
            QtGui.qApp.restoreOverrideCursor()



# ############################################################################
# Rhino Implementation

class RhinoApp(BaseApp):
    def __init__(self):
        BaseApp.__init__(self)

    # ###########################################
    # Logging
    @classmethod
    def error(cls, msg):
        print("ERROR: " + msg)

    @classmethod
    def warn(cls, msg):
        print("WARNING: " + msg)

    @classmethod
    def log(cls, msg):
        print("LOG: " + msg)

    @classmethod
    def message(cls, msg):
        print("MESSAGE: " + msg)

    # ###########################################
    # Document Methods

    @classmethod
    def active_view(cls):
        return rs.CurrentView()

    @classmethod
    def refresh_view(cls):
        rs.Redraw()

    @classmethod
    def view_all(cls):
        rs.ZoomExtents()

    @classmethod
    def view_selection(cls):
        rs.ZoomSelected()

    @classmethod
    def clear(cls):
        count = rs.UnselectAllObjects()
        object_ids = rs.InvertSelectedObjects()
        rs.DeleteObjects(object_ids)


    # ###########################################
    # Selection

    @classmethod
    def clear_selection(cls):
        rs.UnselectAllObjects()




# ############################################################################
# Selecting Implementation (FreeCAD or Rhino)
try:
    import FreeCAD
    import Part
    App = FreeCadApp
    if hasattr(FreeCAD, 'Gui'):
        print("INFO: using FreeCAD")
    else:
        print("INFO: using embedded FreeCAD")
except ImportError:
    # try to embed FreeCAD without GUI
    try:
        sys.path.append(settings.FREECAD_DYLIB_PATH)
        import FreeCAD
        import Part
        App = FreeCadApp
        print("INFO: using embedded FreeCAD")
    except ValueError:
        try:
            import rhinoscript
            import rhinoscriptsyntax as rs
            App = RhinoApp
            print ("INFO: using Rhino")
        except ImportError:
            print("\nError: wrong context, run in FreeCAD or Rhino\n")
            # setup a dummy
            App = BaseApp



# ############################################################################
# Aliases
# Logging
error = App.error
warn = App.warn
log = App.log
message = App.message
# Document
active_view = App.active_view
refresh_view = App.refresh_view
view_all = App.view_all
view_selection = App.view_selection
clear = App.clear
# Selection
clear_selection = App.clear_selection
# Cursor
wait_cursor = App.wait_cursor
normal_cursor = App.normal_cursor
