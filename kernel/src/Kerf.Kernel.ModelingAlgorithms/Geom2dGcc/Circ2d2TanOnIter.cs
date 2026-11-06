// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints;

//=========================================================================
//   Creation d un cercle tangent a deux elements : Droite.               +
//                                                  Cercle.               +
//                                                  Point.                +
//                                                  Courbes.              +
//                        centre sur un troisieme : Droite.               +
//                                                  Cercle.               +
//                                                  Courbes.              +
//=========================================================================

/// <summary>
/// One circle tangent to two arguments (a qualified curve, circle or line, or a point) whose centre lies on a line, a
/// circle or a curve, found iteratively (math_FunctionSetRoot) from starting parameters on the arguments and the
/// centre's curve.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). OCCT's accessors return 0 for the parameters
/// (Tangency1, Tangency2, CenterOn3): the port does the same (occt-issues.md). The constructors with a point
/// (QCurve, point, line/circle/curve) give math_FunctionSetRoot 3-vectors for a 4-variable function: native OCCT and
/// the port throw Standard_OutOfRange there.
/// </remarks>
internal sealed class Circ2d2TanOnIter
{
    private bool _wellDone;
    private Circle2D _cirsol = new();
    private Qualifier _qualifier1;
    private Qualifier _qualifier2;
    private bool _theSame1;
    // OCCT's TheSame2 is only ever set to false and never read (IsTheSame2 returns false): left out.
    private Point2D _pnttg1sol = new();
    private Point2D _pnttg2sol = new();
    private Point2D _pntcen = new();
    private double _par1sol;
    private double _par2sol;
    private double _pararg1;
    private double _pararg2;
    private double _parcen3;

    public Circ2d2TanOnIter(in QualifiedLin qualified1, in QCurve qualified2, in Line2D onLine, double param1, double param2, double param3, double tolang)
    {

        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        double Tol = Math.Abs(tolang);
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Direction2D            dirx = new(Direction2D.D.X);
        Line2D            l1  = qualified1.Qualified();
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = RealFirst();
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = RealFirst();
        umin[4]         = 0.0;
        umax[1]         = RealLast();
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = RealLast();
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = param3;
        tol[1]          = 1.0e-15;
        tol[2]          = ConstraintCurveTool.EpsX(cu2, tolang);
        tol[3]          = tol[1];
        tol[4]          = tol[1];
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(param1, l1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(param3, onLine);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(l1, cu2, onLine, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1,Nor2;
            Vector2D tan1, tan2;
            global::Kerf.Engine.Geometry.ElCLib.D1(ufirst[1], l1, out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            Vector2D tan3 = new(onLine.Direction.XY());
            Point2D point3new = new(onLine.Location.XY() + ufirst[3] * tan3.XY());
            double   dist1 = point3new.DistanceTo(point1);
            double   dist2 = point3new.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, point3new);
                Vector2D vec2 = new(point2, point3new);
                double   normevec2 = vec2.Magnitude;
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double pscal = point3new.XY().Dot(new Xy(-l1.Direction.Y, l1.Direction.X));
                if (qualified1.IsUnqualified || (qualified1.IsOutside && pscal <= 0.0)
                        || (qualified1.IsEnclosed && pscal >= 0.0))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pnttg1sol  = point1;
                        _pararg1    = ufirst[1];
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pnttg2sol  = point2;
                        _pararg2    = ufirst[2];
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3new;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QCurve qualified1, in QCurve qualified2, in Line2D onLine, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D cu1 = qualified1.Qualified();
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = RealFirst();
        umin[4]         = 0.0;
        umax[1]         = ConstraintCurveTool.LastParameter(cu1);
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = RealLast();
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = param3;
        tol[1]          = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = 1.0e-15;
        tol[4]          = Tol / 10.0;
        Point2D point1 = ConstraintCurveTool.Value(cu1, param1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(param3, onLine);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(cu1, cu2, onLine, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            Vector2D tan1, tan2;
            ConstraintCurveTool.D1(cu1, ufirst[1], out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            Vector2D tan3 = new(onLine.Direction.XY());
            Point2D point3new = new(onLine.Location.XY() + ufirst[3] * tan3.XY());
            double   dist1 = point3new.DistanceTo(point1);
            double   dist2 = point3new.DistanceTo(point2);
            if ((dist1 + dist2) / 2.0 < Tol)
            {
                return;
            }
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, point3new);
                Vector2D vec2 = new(point2, point3new);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   angle1, angle2;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = vec1.AngleTo(tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                        || (qualified1.IsOutside && angle1 >= 0.0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pnttg1sol  = point1;
                        _pararg1    = ufirst[1];
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pnttg2sol  = point2;
                        _pararg2    = ufirst[2];
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3new;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QCurve qualified1, in Point2D point2, in Line2D onLine, double param1, double param2, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D cu1 = qualified1.Qualified();
        math_Vector         umin = new(1, 3);
        math_Vector         umax = new(1, 3);
        math_Vector         ufirst = new(1, 3);
        math_Vector         tol = new(1, 3);
        umin[1]         = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]         = RealFirst();
        umin[3]         = 0.0;
        umax[1]         = ConstraintCurveTool.LastParameter(cu1);
        umax[2]         = RealLast();
        umax[3]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        tol[1]          = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[2]          = 1.0e-15;
        tol[3]          = Tol / 10.0;
        Point2D point1 = ConstraintCurveTool.Value(cu1, param1);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(param2, onLine);
        ufirst[3]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(cu1, point2, onLine, Max(ufirst[3], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            Point2D point1new, point3new;
            Vector2D tan1, tan3;
            ConstraintCurveTool.D1(cu1, ufirst[1], out point1new, out tan1);
            global::Kerf.Engine.Geometry.ElCLib.D1(ufirst[2], onLine, out point3new, out tan3);
            double dist1 = point3new.DistanceTo(point1new);
            double dist2 = point3new.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = tan1.Magnitude;
                Vector2D vec1 = new(point1new, point3new);
                double   normevec1 = vec1.Magnitude;
                double   angle1;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = vec1.AngleTo(tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                        || (qualified1.IsOutside && angle1 >= 0.0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                {
                    _qualifier1 = qualified1.Qualifier();
                    _qualifier2 = Qualifier.Noqualifier;
                    _pnttg1sol  = point1new;
                    _pararg1    = ufirst[1];
                    _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                    _pnttg2sol  = point2;
                    _pararg2    = 0.0;
                    _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                    _pntcen     = point3new;
                    _parcen3    = ufirst[2];
                    _wellDone   = true;
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QualifiedCirc qualified1, in QCurve qualified2, in Line2D onLine, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Circle2D           c1  = qualified1.Qualified();
        double              r1  = c1.Radius;
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = RealFirst();
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = RealFirst();
        umin[4]         = 0.0;
        umax[1]         = RealLast();
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = RealLast();
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = param3;
        tol[1]          = 2.0e-15 * M_PI;
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = 1.0e-15;
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(param1, c1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(param3, onLine);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(c1, cu2, onLine, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1,Nor2;
            Vector2D tan1, tan2, nor2;
            global::Kerf.Engine.Geometry.ElCLib.D2(ufirst[1], c1, out point1, out tan1, out nor2);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            onLine.Direction.XY(); // (OCCT_DEBUG makes a vector of it)
            point3       = global::Kerf.Engine.Geometry.ElCLib.Value(ufirst[3], onLine);
            double dist1 = point3.DistanceTo(point1);
            double dist2 = point3.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, point3);
                Vector2D vec2 = new(point2, point3);
                double   normevec2 = vec2.Magnitude;
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double dist = c1.Location.DistanceTo(point3);
                double rsol = _cirsol.Radius;
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && rsol >= r1 && dist <= rsol)
                        || (qualified1.IsOutside && dist >= rsol)
                        || (qualified1.IsEnclosed && rsol <= r1 && dist <= rsol))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pnttg1sol  = point1;
                        _pararg1    = ufirst[1];
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pnttg2sol  = point2;
                        _pararg2    = ufirst[2];
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QualifiedCirc qualified1, in QCurve qualified2, in Circle2D onCirc, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Circle2D           c1  = qualified1.Qualified();
        double              r1  = c1.Radius;
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = RealFirst();
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = RealFirst();
        umin[4]         = 0.0;
        umax[1]         = RealLast();
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = RealLast();
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = param3;
        tol[1]          = 2.0e-15 * M_PI;
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = 2.0e-15 * M_PI;
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(param1, c1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(param3, onCirc);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(c1, cu2, onCirc, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1;
            Vector2D tan1, tan2;
            global::Kerf.Engine.Geometry.ElCLib.D1(ufirst[1], c1, out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            point3       = global::Kerf.Engine.Geometry.ElCLib.Value(ufirst[3], onCirc);
            double dist1 = point3.DistanceTo(point1);
            double dist2 = point3.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, point3);
                Vector2D vec2 = new(point2, point3);
                double   normevec2 = vec2.Magnitude;
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double dist = c1.Location.DistanceTo(point3);
                double rsol = _cirsol.Radius;
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && rsol >= r1 && dist <= rsol)
                        || (qualified1.IsOutside && dist >= rsol)
                        || (qualified1.IsEnclosed && rsol <= r1 && dist <= rsol))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pnttg1sol  = point1;
                        _pararg1    = ufirst[1];
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pnttg2sol  = point2;
                        _pararg2    = ufirst[2];
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QualifiedLin qualified1, in QCurve qualified2, in Circle2D onCirc, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Line2D            l1  = qualified1.Qualified();
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = RealFirst();
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = RealFirst();
        umin[4]         = 0.0;
        umax[1]         = RealLast();
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = RealLast();
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = param3;
        tol[1]          = 1.0e-15;
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = 2.0e-15 * M_PI;
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(param1, l1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(param3, onCirc);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(l1, cu2, onCirc, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            Point2D point1new, point2new;
            Vector2D tan1, tan2;
            global::Kerf.Engine.Geometry.ElCLib.D1(ufirst[1], l1, out point1new, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2new, out tan2);
            point3       = global::Kerf.Engine.Geometry.ElCLib.Value(ufirst[3], onCirc);
            double dist1 = point3.DistanceTo(point1new);
            double dist2 = point3.DistanceTo(point2new);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1new, point3);
                Vector2D vec2 = new(point2new, point3);
                double   normevec2 = vec2.Magnitude;
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double pscal = point3.XY().Dot(new Xy(-l1.Direction.Y, l1.Direction.X));
                if (qualified1.IsUnqualified || (qualified1.IsOutside && pscal <= 0.0)
                        || (qualified1.IsEnclosed && pscal >= 0.0))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pnttg1sol  = point1new;
                        _pararg1    = ufirst[1];
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pnttg2sol  = point2new;
                        _pararg2    = ufirst[2];
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QCurve qualified1, in QCurve qualified2, in Circle2D onCirc, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D cu1 = qualified1.Qualified();
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = RealFirst();
        umin[4]         = 0.0;
        umax[1]         = ConstraintCurveTool.LastParameter(cu1);
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = RealLast();
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = param3;
        tol[1]          = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = 2.0e-15 * M_PI;
        tol[4]          = Tol / 10.0;
        Point2D point1 = ConstraintCurveTool.Value(cu1, param1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        double   r1     = onCirc.Radius;
        Point2D point3 = new(onCirc.Location.XY() + r1 * new Xy(Math.Cos(param3), Math.Sin(param3)));
        ufirst[4] = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(cu1, cu2, onCirc, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1;
            Vector2D tan1, tan2;
            ConstraintCurveTool.D1(cu1, ufirst[1], out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            point3 =
                new Point2D(onCirc.Location.XY() + r1 * new Xy(Math.Cos(ufirst[3]), Math.Sin(ufirst[3])));
            double dist1 = point3.DistanceTo(point1);
            double dist2 = point3.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, point3);
                Vector2D vec2 = new(point2, point3);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   angle1, angle2;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = vec1.AngleTo(tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                        || (qualified1.IsOutside && angle1 >= 0.0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pararg1    = ufirst[1];
                        _pnttg1sol  = point1;
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pararg2    = ufirst[2];
                        _pnttg2sol  = point2;
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QCurve qualified1, in Point2D point2, in Circle2D onCirc, double param1, double param2, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D cu1 = qualified1.Qualified();
        math_Vector         umin = new(1, 3);
        math_Vector         umax = new(1, 3);
        math_Vector         ufirst = new(1, 3);
        math_Vector         tol = new(1, 3);
        umin[1]         = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]         = RealFirst();
        umin[3]         = 0.0;
        umax[1]         = ConstraintCurveTool.LastParameter(cu1);
        umax[2]         = RealLast();
        umax[3]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        tol[1]          = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[2]          = 2.0e-15 * M_PI;
        tol[3]          = Tol / 10.0;
        Point2D point1 = ConstraintCurveTool.Value(cu1, param1);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(param2, onCirc);
        ufirst[3]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(cu1, point2, onCirc, Max(ufirst[3], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            Point2D point1new, point3new;
            Vector2D tan1, tan3;
            ConstraintCurveTool.D1(cu1, ufirst[1], out point1new, out tan1);
            global::Kerf.Engine.Geometry.ElCLib.D1(ufirst[2], onCirc, out point3new, out tan3);
            double dist1 = point3new.DistanceTo(point1new);
            double dist2 = point3new.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = tan1.Magnitude;
                Vector2D vec1 = new(point1new, point3new);
                double   normevec1 = vec1.Magnitude;
                double   angle1;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = vec1.AngleTo(tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                        || (qualified1.IsOutside && angle1 >= 0.0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                {
                    _qualifier1 = qualified1.Qualifier();
                    _qualifier2 = Qualifier.Noqualifier;
                    _pnttg1sol  = point1new;
                    _pararg1    = ufirst[1];
                    _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                    _pnttg2sol  = point2;
                    _pararg2    = 0.0;
                    _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                    _pntcen     = point3new;
                    _parcen3    = ufirst[2];
                    _wellDone   = true;
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QCurve qualified1, in QCurve qualified2, CurveAdaptor2D onCurv, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        double Tol = Math.Abs(tolerance);
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D cu1 = qualified1.Qualified();
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = ConstraintCurveTool.FirstParameter(onCurv);
        umin[4]         = 0.0;
        umax[1]         = ConstraintCurveTool.LastParameter(cu1);
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = ConstraintCurveTool.LastParameter(onCurv);
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = param3;
        tol[1]          = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = ConstraintCurveTool.EpsX(onCurv, Math.Abs(tolerance));
        tol[4]          = Tol / 10.0;
        Point2D point1 = ConstraintCurveTool.Value(cu1, param1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = ConstraintCurveTool.Value(onCurv, param3);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(cu1, cu2, onCurv, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            Vector2D tan1, tan2, tan3;
            ConstraintCurveTool.D1(cu1, ufirst[1], out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            ConstraintCurveTool.D1(onCurv, ufirst[3], out point3, out tan3);
            double dist1 = point3.DistanceTo(point1);
            double dist2 = point3.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, point3);
                Vector2D vec2 = new(point2, point3);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   angle1, angle2;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = vec1.AngleTo(tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                        || (qualified1.IsOutside && angle1 >= 0.0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pararg1    = ufirst[1];
                        _pnttg1sol  = point1;
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pararg2    = ufirst[2];
                        _pnttg2sol  = point2;
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QualifiedCirc qualified1, in QCurve qualified2, CurveAdaptor2D onCurv, double param1, double param2, double paramOn, double tolerance)
    {

        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Circle2D           c1  = qualified1.Qualified();
        double              r1  = c1.Radius;
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = RealFirst();
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = ConstraintCurveTool.FirstParameter(onCurv);
        umin[4]         = 0.0;
        umax[1]         = RealLast();
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = ConstraintCurveTool.LastParameter(onCurv);
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = paramOn;
        tol[1]          = 2.0e-15 * M_PI;
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = ConstraintCurveTool.EpsX(onCurv, Math.Abs(tolerance));
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(param1, c1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = ConstraintCurveTool.Value(onCurv, paramOn);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(c1, cu2, onCurv, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            Vector2D tan1, tan2, tan3;
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            ConstraintCurveTool.D1(onCurv, ufirst[3], out point3, out tan3);
            global::Kerf.Engine.Geometry.ElCLib.D1(ufirst[1], c1, out point1, out tan1);
            double dist1 = point3.DistanceTo(point1);
            double dist2 = point3.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                Direction2D dirx = new(Direction2D.D.X);
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(new Point2D(point1.XY()), new Point2D(point3.XY()));
                Vector2D vec2 = new(new Point2D(point2.XY()), new Point2D(point3.XY()));
                double   normevec2 = vec2.Magnitude;
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double dist = c1.Location.DistanceTo(point3);
                double rsol = _cirsol.Radius;
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && rsol >= r1 && dist <= rsol)
                        || (qualified1.IsOutside && dist >= rsol)
                        || (qualified1.IsEnclosed && rsol <= r1 && dist <= rsol))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pnttg1sol  = point1;
                        _pararg1    = ufirst[1];
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pnttg2sol  = point2;
                        _pararg2    = ufirst[2];
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QualifiedLin qualified1, in QCurve qualified2, CurveAdaptor2D onCurv, double param1, double param2, double paramOn, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Line2D            l1  = qualified1.Qualified();
        CurveAdaptor2D cu2 = qualified2.Qualified();
        math_Vector         umin = new(1, 4);
        math_Vector         umax = new(1, 4);
        math_Vector         ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        umin[1]         = RealFirst();
        umin[2]         = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]         = ConstraintCurveTool.FirstParameter(onCurv);
        umin[4]         = 0.0;
        umax[1]         = RealLast();
        umax[2]         = ConstraintCurveTool.LastParameter(cu2);
        umax[3]         = ConstraintCurveTool.LastParameter(onCurv);
        umax[4]         = RealLast();
        ufirst[1]       = param1;
        ufirst[2]       = param2;
        ufirst[3]       = paramOn;
        tol[1]          = 1.0e-15;
        tol[2]          = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]          = ConstraintCurveTool.EpsX(onCurv, Math.Abs(tolerance));
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(param1, l1);
        Point2D point2 = ConstraintCurveTool.Value(cu2, param2);
        Point2D point3 = ConstraintCurveTool.Value(onCurv, paramOn);
        ufirst[4]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(l1, cu2, onCurv, Max(ufirst[4], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            Vector2D tan1, tan2, tan3;
            global::Kerf.Engine.Geometry.ElCLib.D1(ufirst[1], l1, out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            ConstraintCurveTool.D1(onCurv, ufirst[3], out point3, out tan3);
            double dist1 = point3.DistanceTo(point1);
            double dist2 = point3.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, point3);
                Vector2D vec2 = new(point2, point3);
                double   normevec2 = vec2.Magnitude;
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = vec2.AngleTo(tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double pscal = point3.XY().Dot(new Xy(-l1.Direction.Y, l1.Direction.X));
                if (qualified1.IsUnqualified || (qualified1.IsOutside && pscal <= 0.0)
                        || (qualified1.IsEnclosed && pscal >= 0.0))
                {
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle2 <= 0.0)
                            || (qualified2.IsOutside && angle2 >= 0) || (qualified2.IsEnclosed && angle2 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pnttg1sol  = point1;
                        _pararg1    = ufirst[1];
                        _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                        _pnttg2sol  = point2;
                        _pararg2    = ufirst[2];
                        _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                        _pntcen     = point3;
                        _parcen3    = ufirst[3];
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d2TanOnIter(in QCurve qualified1, in Point2D point2, CurveAdaptor2D onCurv, double param1, double paramOn, double tolerance)
    {
        _theSame1 = false;
        _par1sol  = 0.0;
        _par2sol  = 0.0;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _parcen3  = 0.0;

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double              Tol = Math.Abs(tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D cu1 = qualified1.Qualified();
        math_Vector         umin = new(1, 3);
        math_Vector         umax = new(1, 3);
        math_Vector         ufirst = new(1, 3);
        math_Vector         tol = new(1, 3);
        umin[1]         = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]         = RealFirst();
        umin[3]         = ConstraintCurveTool.FirstParameter(onCurv);
        umax[1]         = ConstraintCurveTool.LastParameter(cu1);
        umax[2]         = RealLast();
        umax[3]         = ConstraintCurveTool.LastParameter(onCurv);
        ufirst[1]       = param1;
        ufirst[2]       = paramOn;
        tol[1]          = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[2]          = ConstraintCurveTool.EpsX(onCurv, Math.Abs(tolerance));
        tol[3]          = Tol / 10.0;
        Point2D point1 = ConstraintCurveTool.Value(cu1, param1);
        Point2D point3 = ConstraintCurveTool.Value(onCurv, paramOn);
        ufirst[3]       = (point3.DistanceTo(point2) + point3.DistanceTo(point1)) / 2.0;
        FunctionTanCuCuOnCu func = new(cu1, point2, onCurv, Max(ufirst[3], Tol));
        FunctionSetRoot          root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        func.Value(ufirst, umin);
        if (root.IsDone)
        {
            root.Root(ufirst);
            //    gp_Vec2d Tan1,Tan2,Tan3;
            Vector2D tan1, tan3;
            ConstraintCurveTool.D1(cu1, ufirst[1], out point1, out tan1);
            ConstraintCurveTool.D1(onCurv, ufirst[3], out point3, out tan3);
            double dist1 = point3.DistanceTo(point1);
            double dist2 = point3.DistanceTo(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                _cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = tan1.Magnitude;
                Vector2D vec1 = new(point1, point3);
                double   normevec1 = vec1.Magnitude;
                double   angle1;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = vec1.AngleTo(tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                        || (qualified1.IsOutside && angle1 >= 0.0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                {
                    _qualifier1 = qualified1.Qualifier();
                    _qualifier2 = Qualifier.Noqualifier;
                    _pnttg1sol  = point1;
                    _pararg1    = ufirst[1];
                    _par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol);
                    _pnttg2sol  = point2;
                    _pararg2    = 0.0;
                    _par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol);
                    _pntcen     = point3;
                    _parcen3    = ufirst[2];
                    _wellDone   = true;
                }
            }
        }
    }

    /// <summary>True if the construction converged to a solution.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The solution (not checked: an undone construction returns the default circle).</summary>
    public Circle2D ThisSolution()
    {
        return _cirsol;
    }

    /// <summary>The qualifiers of the solution relative to the two arguments.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void WhichQualifier(out Qualifier qualif1, out Qualifier qualif2)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            qualif1 = _qualifier1;
            qualif2 = _qualifier2;
        }
    }

    /// <summary>The tangency point on the first argument (OCCT returned 0 for the parameters: Kerf decision 1545).</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    public void Tangency1(out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            if (!_theSame1)
            {
                parSol = _par1sol;
                parArg = _pararg1;
                pntSol = _pnttg1sol;
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The tangency point on the second argument (OCCT returned 0 for the parameters: Kerf decision 1545).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void Tangency2(out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            parSol = _par2sol;
            parArg = _pararg2;
            pntSol = _pnttg2sol;
        }
    }

    /// <summary>The centre of the solution (OCCT returned 0 for its parameter: Kerf decision 1545).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void CenterOn3(out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            parArg = _parcen3;
            pntSol = _pntcen;
        }
    }

    /// <summary>True if the solution is the first argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame1()
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if (!_theSame1)
        {
            return false;
        }
        return true;
    }

    /// <summary>Always false: the second argument is never the solution.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame2
    {
        get
        {
            if (!_wellDone)
            {
                throw new NotDoneException();
            }
            return false;
        }
    }
}
