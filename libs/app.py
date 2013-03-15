from __future__ import print_function  # need print func in lambda
from __future__ import division  # true division from integers

import sys

from gx import settings



__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__all__ = ['active_view', 'refresh_view', 'view_all', 'view_selection',
           'clear_selection', 'wait_cursor', 'normal_cursor', 'say']

def say():
    print("say7")

# ############################################################################
# General Implementation

class BaseApp():
    def __init__(self):
        BaseApp.__init__(self)

    # ###########################################
    # implemented in FreeCadApp, and RhinoApp
    # Document Methods
    def active_view(cls): pass
    def refresh_view(cls): pass
    def view_all(cls): pass
    def view_selection(cls): pass
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
# App-level
active_view = App.active_view
refresh_view = App.refresh_view
view_all = App.view_all
view_selection = App.view_selection
# Selection
clear_selection = App.clear_selection
# Cursor
wait_cursor = App.wait_cursor
normal_cursor = App.normal_cursor
