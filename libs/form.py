from __future__ import print_function  # need print func in lambda
from __future__ import division  # true division from integers

"""Generative Expressions Library

gx is a library for generating geometry in FreeCAD and Rhino,
and deriving fabrication instruction for machine control.

[...]

Adding python files to the search path
---------------------------------------
* FreeCAD
  * import sys
  * sys.path.append("path/to/scripts")
* Rhino
  * import rhinoscriptsyntax as rs
  * rs.AddSearchPath("C:\\My Python Scripts")

Running FreeCAD standalone in Ubuntu
------------------------------------
* import sys
* sys.path.append("/usr/lib/freecad/lib/")
* import FreeCAD

Resources, example scripts
---------------------------
* /usr/lib/freecad/Mod

"""

import sys
import math
import random

from gx import settings
from .euclid import euclid
from .vectormath import *



__author__  = 'Stefan Hechenberger <stefan@nortd.com>'
__all__ = ['selected', 'point', 'line', 'circle', 'interpolation_curve', 'random_curve',
            'translate', 'scale', 'rotate']



# ############################################################################
# General Implementation

class BaseForm(object):

    def __init__(self):
        self.obj = None

    # ###########################################
    # implemented in FreeCadForm, and RhinoForm
    # Factories
    def selected(cls): pass
    def point(cls, p): pass
    def line(cls, p1, p2): pass
    def circle(cls, p1, p2): pass
    def interpolation_curve(cls, pts): pass
    def random_curve(cls, nPts=4, xr=(0,1), yr=(0,1), zr=(0,0), xsigma=0.5): pass
    # Selection
    def select(self, clearFirst=False): pass
    def unselect(self): pass
    def is_selected(self): pass
    # Geometry Classification
    def is_curve(self): pass
    def is_line_curve(self): pass
    def is_planar_curve(self): pass  # TODO: implement in FreeCAD
    def is_closed_curve(self): pass
    # Curve Methods
    def length(self): pass
    def value_at(self, t, paramNormalized=True): pass
    def tangent_at(self, t, paramNormalized=True): pass
    def curvature_at(self, t, paramNormalized=True): pass
    def center_of_curvature_at(self, t, paramNormalized=True): pass
    def derivative1_at(self, t, paramNormalized=True): pass
    def derivative2_at(self, t, paramNormalized=True): pass
    def derivative3_at(self, t, paramNormalized=True): pass
    def closest_curve_point(self, pt, paramNormalized=True): pass
    def tessellate(self, num): pass
    # Surface Methods
    # def normal_at(self, u, v, paramNormalized=True): pass
    # General Geometry Methods

    # Transformations
    # def transform_shape(self, mat): pass
    def transform(self, mat): pass
    def move(self, x, y, z):
        self.transform(tM(x, y, z))
    def scale(self, x, y=None, z=None, center=(0,0,0)):
        if not y or not z:
            y = x
            z = x
        if center == V():
            mat = sM(x,y,z)
        else:
            mat = (tM(center[0], center[1], center[2]) * 
                   sM(x,y,z) * 
                   tM(-center[0], -center[1], -center[2]))
        self.transform(mat)
    def rotate(self, quaternion, center=(0,0,0)):
        if center == V():
            mat = quaternion.get_matrix()
        else:
            mat = (tM(center[0], center[1], center[2]) * 
                   quaternion.get_matrix() * 
                   tM(-center[0], -center[1], -center[2]))
        self.transform(mat)
    def rotatexyz(self, x, y, z, center=(0,0,0)):
        if center == V():
            mat = rM(y, z, x)
        else:
            mat = (tM(center[0], center[1], center[2]) * 
                   rM(y, z, x) * 
                   tM(-center[0], -center[1], -center[2]))
        self.transform(mat)
    def rotatex(self, angle, center=(0,0,0)):
        if center == V():
            mat = xM(angle)
        else:
            mat = (tM(center[0], center[1], center[2]) * 
                   xM(angle) * 
                   tM(-center[0], -center[1], -center[2]))
        self.transform(mat)
    def rotatey(self, angle, center=(0,0,0)):
        if center == V():
            mat = yM(angle)
        else:
            mat = (tM(center[0], center[1], center[2]) * 
                   yM(angle) * 
                   tM(-center[0], -center[1], -center[2]))
        self.transform(mat)
    def rotatez(self, angle, center=(0,0,0)):
        if center == V():
            mat = zM(angle)
        else:
            mat = (tM(center[0], center[1], center[2]) * 
                   zM(angle) * 
                   tM(-center[0], -center[1], -center[2]))
        self.transform(mat)
    def rotatea(self, angle, axis, center=(0,0,0)):
        if center == V():
            mat = aM(angle, axis)
        else:
            mat = (tM(center[0], center[1], center[2]) * 
                   aM(angle, axis) * 
                   tM(-center[0], -center[1], -center[2]))
        self.transform(mat)



# ############################################################################
# FreeCAD Implementation

class FreeCadForm(BaseForm):
    def __init__(self):
        BaseForm.__init__(self)
        if FreeCAD.ActiveDocument == None:
            FreeCAD.newDocument()


    # ###########################################
    # Factories

    @classmethod
    def selected(cls):
        objs = FreeCAD.Gui.Selection.getSelection()
        if objs:
            self = cls()
            self.obj = objs[0]
            return self
        else:
            return None

    @classmethod
    def point(cls, p):
        self = cls()
        # self.obj = FreeCAD.ActiveDocument.addObject("Part::Feature","gxPoint")
        # shape = Draft.makePoint(p[0], p[1], p[2])
        # self.obj.Shape = shape
        self.obj = Draft.makePoint(p[0], p[1], p[2])
        self.obj.Label = u'gxPoint'
        return self

    @classmethod
    def line(cls, p1, p2):
        self = cls()
        self.obj = FreeCAD.ActiveDocument.addObject("Part::Feature","gxLine")
        shape = Part.makeLine(tuple(p1), tuple(p2))  # Part.TopoShape
        self.obj.Shape = shape
        # self.obj = Part.makeLine(p1, p2)
        # Part.show(self.obj)
        # FreeCAD.Console.PrintMessage("line made")
        return self

    @classmethod
    def circle(cls, r):
        self = cls()
        self.obj = FreeCAD.ActiveDocument.addObject("Part::Feature","gxCircle")
        circ = Part.Circle()  # Part.GeomCircle
        circ.Radius = float(r)
        self.obj.Shape = circ.toShape()
        # shape = Part.makeCircle(r)
        # obj.Shape = shape
        return self

    @classmethod
    def interpolation_curve(cls, pts):
        self = cls()
        self.obj = FreeCAD.ActiveDocument.addObject("Part::Feature","gxCurve")
        crv = Part.BSplineCurve()  # Part.GeomCircle
        pts_tuples = []  # make sure points are tuples
        for pt in pts:
            pts_tuples.append(tuple(pt))
        crv.interpolate(pts_tuples)
        self.obj.Shape = crv.toShape()
        return self

    @classmethod
    def random_curve(cls, nPts=4, xr=(0,1), yr=(0,1), zr=(0,0), xsigma=0.5):
        nPts = int(nPts)
        if nPts == 0: return None
        self = cls()
        self.obj = FreeCAD.ActiveDocument.addObject("Part::Feature","gxCurve")
        crv = Part.BSplineCurve()
        pts = []
        step = float(xr[1]-xr[0])/nPts
        for i in range(nPts):
            pts.append((random.gauss(xr[0]+i*step, xsigma*step),
                        random.uniform(yr[0],yr[1]),
                        random.uniform(zr[0],zr[1])))
        crv.interpolate(pts)
        self.obj.Shape = crv.toShape()
        return self


    # ###########################################
    # Selection

    def select(self, clearFirst=False):
        if clearFirst:
            clear_selection()
        FreeCAD.Gui.Selection.addSelection(self.obj)

    def unselect(self):
        FreeCAD.Gui.Selection.removeSelection(self.obj)

    def is_selected(self):
        return FreeCAD.Gui.Selection.isSelected(self.obj)


    # ###########################################
    # Geometry Classification

    def is_curve(self):
        """Is this a single curve."""
        return hasattr(self.obj.Shape, 'Curve')
        # return self.obj.isDerivedFrom("Part::Feature") and \
        #        self.obj.Shape and self.obj.Shape.isValid() and \
        #        len(self.obj.Shape.Edges) == 1

    def is_line_curve(self):
        return self.is_curve and type(self.obj.Shape.Curve) == Part.Line
        # return self.is_curve and (len(self.obj.Shape.Edges[0].Vertexes) == 2)

    def is_planar_curve(self):
        self.error("not implemented")

    def is_closed_curve(self):
        return self.is_curve and self.obj.Shape.isClosed()


    # ###########################################
    # Curve Methods

    def length(self):
        if self.is_curve():
            return self.obj.Shape.Length
        else:
            self.error("not a curve")
            return None

    def value_at(self, t, paramNormalized=True):
        """param: 0 to 1.0"""
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.Shape.Edges[0].valueAt(t)
        else:
            self.error("not a curve")
            return None

    def tangent_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.Shape.Edges[0].tangentAt(t)
        else:
            self.error("not a curve")
            return None

    def curvature_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.Shape.Edges[0].curvatureAt(t)
        else:
            self.error("not a curve")
            return None

    def center_of_curvature_at(self, t, paramNormalized=True):
        # TODO: getting an exception
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.Shape.Edges[0].centerOfCurvatureAt(t)
        else:
            self.error("not a curve")
            return None

    def derivative1_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.Shape.Edges[0].derivative1At(t)
        else:
            self.error("not a curve")
            return None

    def derivative2_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.Shape.Edges[0].derivative2At(t)
        else:
            self.error("not a curve")
            return None

    def derivative3_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.Shape.Edges[0].derivative3At(t)
        else:
            self.error("not a curve")
            return None

    def closest_curve_point(self, pt, paramNormalized=True):
        if self.is_curve():
            t = self.obj.Shape.Curve.parameter(FreeCAD.Vector(pt[0],pt[1],pt[2]))
            if paramNormalized: t = self._param_to_normalized(t)
            return t
        else:
            self.error("not a curve")
            return None

    def tessellate(self, num):
        """Return a list of points (vectors).
        param: number of points
               if under 1 then parameter t increment
        """
        if self.is_curve():
            return self.obj.Shape.Curve.discretize(num)
        else:
            self.error("not a curve")
            return None

    def _param_from_normalized(self, t):
        # same as 0-self.obj.Shape.Length ?
        return (self.obj.Shape.FirstParameter + 
               t*(self.obj.Shape.LastParameter-self.obj.Shape.FirstParameter))

    def _param_to_normalized(self, t):
        return ((t - self.obj.Shape.FirstParameter)/
               (self.obj.Shape.LastParameter-self.obj.Shape.FirstParameter))



    # ###########################################
    # Surface Methods

    # def normal_at(self, u, v, paramNormalized=True):
    #     # TODO: getting an exception
    #     if self.is_curve():
    #         if paramNormalized: t = self._param_from_normalized(t)
    #         return self.obj.Shape.Edges[0].normalAt(t)
    #     else:
    #         self.error("not a curve")
    #         return None


    # ###########################################
    # Transformations

    # def transform_shape(self, matrix):
    #     self.obj.transformShape()

    def transform(self, mat):
        # fmat = FreeCAD.Matrix(mat[0][0],mat[0][1], mat[0][2],mat[0][3],
        #                       mat[1][0],mat[1][1], mat[1][2],mat[1][3],
        #                       mat[2][0],mat[2][1], mat[2][2],mat[2][3],
        #                       mat[3][0],mat[3][1], mat[3][2],mat[3][3])
        fmat = FreeCAD.Matrix(mat[0],mat[4], mat[8],mat[12],
                              mat[1],mat[5], mat[9],mat[13],
                              mat[2],mat[6], mat[10],mat[14],
                              mat[3],mat[7], mat[11],mat[15])
        self.obj.Shape = self.obj.Shape.transformGeometry(fmat)



# ############################################################################
# Rhino Implementation

class RhinoForm(BaseForm):
    def __init__(self):
        BaseForm.__init__(self)

    # ###########################################
    # Factories

    @classmethod
    def selected(cls):
        objs = rs.SelectedObjects()
        if objs:
            self = cls()
            self.obj = objs[0]  # get the first
            return self
        else:
            return None

    @classmethod
    def point(cls, p):
        self = cls()
        self.obj = rs.AddPoint(p)
        return self

    @classmethod
    def line(cls, p1, p2):
        self = cls()
        self.obj = rs.AddLine(p1, p2)
        return self

    @classmethod
    def circle(cls, r):
        self = cls()
        self.obj = rs.AddCircle(rs.WorldXYPlane(), r)
        return self

    @classmethod
    def interpolation_curve(cls, pts):
        self = cls()
        self.obj = rs.AddInterpCurve(pts)
        return self

    @classmethod
    def random_curve(cls, nPts=4, xr=(0,1), yr=(0,1), zr=(0,0), xsigma=0.5):
        nPts = int(nPts)
        if nPts == 0: return None
        self = cls()
        pts = []
        step = float(xr[1]-xr[0])/nPts
        for i in range(nPts):
            pts.append((random.gauss(xr[0]+i*step, xsigma*step),
                        random.uniform(yr[0],yr[1]),
                        random.uniform(zr[0],zr[1])))
        self.obj = rs.AddInterpCurve(pts)
        return self


    # ###########################################
    # Selection

    def select(self, clearFirst=False):
        if clearFirst:
            clear_selection()
        rs.SelectObjects([self.obj])

    def unselect(self):
        rs.UnselectObjects([self.obj])

    def is_selected(self):
        return rs.IsObjectSelected(self.obj)


    # ###########################################
    # Geometry Classification

    def is_curve(self):
        return rs.IsCurve(self.obj)

    def is_line_curve(self):
        return rs.IsLine(self.obj)

    def is_planar_curve(self):
        return rs.IsCurvePlanar(self.obj)

    def is_closed_curve(self):
        return rs.IsCurveClosed(self.obj)

    # ###########################################
    # Curve Methods

    def length(self):
        if self.is_curve():
            return rs.CurveLength(self.obj)
        else:
            self.error("not a curve")
            return None

    def value_at(self, t, paramNormalized=True):
        """Return point at paramter t in space of the curve.

        FYI: While parameter space is evenly distributed for some curves,
        this is not true for NURBS. They are more widely spaced towards
        the ends, and more closely spaced at areas of dense control
        points and more weighted CPs.
        """
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return rs.EvaluateCurve(t)   
        else:
            self.error("not a curve")
            return None

    def tangent_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return rs.CurveTangent(self.obj, t)
        else:
            self.error("not a curve")
            return None

    def curvature_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            curvatureObj = rs.CurveCurvature(self.obj, t)
            if curvatureObj:
                return curvatureObj[3]
                # curvatureObj[0]  ... point
                # curvatureObj[1]  ... tangent
                # curvatureObj[2]  ... center of circle
                # curvatureObj[4]  ... curvature vector
            else:
                return None
        else:
            self.error("not a curve")
            return None

    def center_of_curvature_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            curvatureObj = rs.CurveCurvature(self.obj, t)
            if curvatureObj:
                return curvatureObj[2]
            else:
                return None
        else:
            self.error("not a curve")
            return None

    def derivative1_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.derivative1At(param)
        else:
            self.error("not a curve")
            return None

    def derivative2_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.derivative2At(param)
        else:
            self.error("not a curve")
            return None

    def derivative3_at(self, t, paramNormalized=True):
        if self.is_curve():
            if paramNormalized: t = self._param_from_normalized(t)
            return self.obj.derivative3At(param)
        else:
            self.error("not a curve")
            return None

    def closest_curve_point(self, pt, paramNormalized=True):
        if self.is_curve():
            t = rs.CurveClosestPoint(self.obj, pt)
            if paramNormalized: t = self._param_to_normalized(t)
            return t
        else:
            self.error("not a curve")
            return None

    def tessellate(self, num):
        """Tessellate curve.
        num: number of points
             if <= 1 points with num distance in nomalized t"""
        if self.is_curve():
            if num <= 1.0:
                seglen = num*rs.CurveLength(self.obj)
                return rs.DivideCurveLength(self.obj, seglen)
            else:
                return rs.DivideCurve(self.obj, num-1)
        else:
            self.error("not a curve")
            return None

    def _param_from_normalized(self, t):
        domain = rs.CurveDomain(self.obj)
        return domain[0] + t*(domain[1]-domain[0])

    def _param_to_normalized(self, t):
        domain = rs.CurveDomain(self.obj)
        return (t - domain[0])/(domain[1]-domain[0])



    # ###########################################
    # Surface Methods


    # ###########################################
    # Transformations

    # def transform_shape(self, matrix):
    #     self.obj.transformShape()

    def transform(self, mat):
        rs.TransformObject(self.obj, 
            [mat[:4],mat[4:8],mat[8:12],mat[12:16]])


# ############################################################################
# Selecting Implementation (FreeCAD or Rhino)
try:
    import FreeCAD
    import Part
    import Draft
    Form = FreeCadForm
except ImportError:
    # try to embed FreeCAD without GUI
    try:
        sys.path.append(settings.FREECAD_DYLIB_PATH)
        import FreeCAD
        import Part
        import Draft
        Form = FreeCadForm
    except ValueError:
        try:
            import rhinoscript
            import rhinoscriptsyntax as rs
            Form = RhinoForm
        except ImportError:
            print("\nError: wrong context, run in FreeCAD or Rhino\n")
            # setup a dummy
            Form = BaseForm


# ############################################################################
# Aliases
# Form Factories
selected = Form.selected
point = Form.point
line = Form.line
circle = Form.circle
interpolation_curve = Form.interpolation_curve
random_curve = Form.random_curve
# Transformation Forwarders
def translate(x, y, z):
    form = Form.selected()
    if form:
        form.translate(x,y,z)
        form.select()
def scale(x, y=None, z=None):
    form = Form.selected()
    if form:
        form.scale(x,y,z)
        form.select()
def rotate(x, y, z):
    form = Form.selected()
    if form:
        form.rotate(x,y,z)
        form.select()
