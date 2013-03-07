from __future__ import print_function  # need print func in lambda
from __future__ import division  # true division from integers


__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__all__ = ['active_view', 'refresh_view', 'view_all', 'view_selection',
           'clear_selection', 'wait_cursor', 'normal_cursor']



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
        return FreeCAD.ActiveDocument.getObject(name).ViewObject

    @classmethod
    def active_view(cls):
        return FreeCAD.Gui.ActiveDocument.ActiveView

    @classmethod
    def refresh_view(cls):
        FreeCAD.ActiveDocument.recompute()

    @classmethod
    def view_all(cls):
        FreeCAD.Gui.SendMsgToActiveView("ViewFit")

    @classmethod
    def view_selection(cls):
        FreeCAD.Gui.SendMsgToActiveView("ViewSelection")

    # ###########################################
    # Selection

    @classmethod
    def clear_selection(cls):
        FreeCAD.Gui.Selection.clearSelection()

    # ###########################################
    # Cursor

    @classmethod
    def wait_cursor(cls): 
        from PyQt4 import QtCore,QtGui
        QtGui.qApp.setOverrideCursor(QtCore.Qt.WaitCursor)
    @classmethod
    def normal_cursor(cls):
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
except ImportError:
    try:
        import rhinoscript
        import rhinoscriptsyntax as rs
        App = RhinoApp
    except ImportError:
        print("Error: wrong context, run in FreeCAD or Rhino")



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
