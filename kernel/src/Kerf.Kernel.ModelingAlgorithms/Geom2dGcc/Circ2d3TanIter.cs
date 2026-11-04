// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3TanIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3TanIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// A circle tangent to three arguments (qualified curves, circles or lines, or points), at least one of them a curve,
/// found iteratively from initial parameters on the arguments: math_FunctionSetRoot on Geom2dGcc_FunctionTanCuCuCu,
/// then the circle through the three points found (GccAna_Circ2d3Tan), kept if it is tangent and its qualifiers hold.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); one solution at most. A point argument is a circle
/// of radius 0 for the function.
/// </remarks>
internal sealed class Circ2d3TanIter
{
    private bool _wellDone;
    private Circle2D _cirsol = new();
    private Qualifier _qualifier1;
    private Qualifier _qualifier2;
    private Qualifier _qualifier3;
    private bool _theSame1;
    // OCCT's TheSame2 is only ever false, as IsTheSame2's answer is: left out.
    private bool _theSame3;
    private Point2D _pnttg1sol = new();
    private Point2D _pnttg2sol = new();
    private Point2D _pnttg3sol = new();
    // The parameters of the tangency points on the solution are computed when asked (OCCT stored 0, or distances
    // between the points: Kerf decision 17107).
    private double _pararg1;
    private double _pararg2;
    private double _pararg3;


    public Circ2d3TanIter(in QCurve qualified1, in QCurve qualified2, in QCurve qualified3, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified)
                || !(qualified3.IsEnclosed || qualified3.IsEnclosing || qualified3.IsOutside
                          || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        CurveAdaptor2D         cu1 = qualified1.Qualified();
        CurveAdaptor2D         cu2 = qualified2.Qualified();
        CurveAdaptor2D         cu3 = qualified3.Qualified();
        FunctionTanCuCuCu func = new(cu1, cu2, cu3);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]   = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]   = ConstraintCurveTool.FirstParameter(cu3);
        umax[1]   = ConstraintCurveTool.LastParameter(cu1);
        umax[2]   = ConstraintCurveTool.LastParameter(cu2);
        umax[3]   = ConstraintCurveTool.LastParameter(cu3);
        ufirst[1] = param1;
        ufirst[2] = param2;
        ufirst[3] = param3;
        tol[1]    = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[2]    = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]    = ConstraintCurveTool.EpsX(cu3, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D point1 = new(), point2 = new(), point3 = new();
            Vector2D tan1 = new(), tan2 = new(), tan3 = new();
            ConstraintCurveTool.D1(cu1, ufirst[1], out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            ConstraintCurveTool.D1(cu3, ufirst[3], out point3, out tan3);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol             = circ.ThisSolution(1);
                Point2D centre    = _cirsol.Location;
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double angle1 = vec1.AngleTo(tan1);
                    if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                            || (qualified1.IsOutside && angle1 >= 0.0)
                            || (qualified1.IsEnclosed && angle1 <= 0.0))
                    {
                        angle1 = vec2.AngleTo(tan2);
                        if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle1 <= 0.0)
                                || (qualified2.IsOutside && angle1 >= 0)
                                || (qualified2.IsEnclosed && angle1 <= 0.0))
                        {
                            angle1 = vec3.AngleTo(tan3);
                            if (qualified3.IsUnqualified || (qualified3.IsEnclosing && angle1 <= 0.0)
                                    || (qualified3.IsOutside && angle1 >= 0)
                                    || (qualified3.IsEnclosed && angle1 <= 0.0))
                            {
                                _qualifier1 = qualified1.Qualifier();
                                _qualifier2 = qualified2.Qualifier();
                                _qualifier3 = qualified3.Qualifier();
                                _pararg1    = ufirst[1];
                                _pnttg1sol  = point1;
                                _pararg2    = ufirst[2];
                                _pnttg2sol  = point2;
                                _pnttg3sol  = point3;
                                _pararg3    = ufirst[3];
                                _wellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QualifiedCirc qualified1, in QCurve qualified2, in QCurve qualified3, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified)
                || !(qualified3.IsEnclosed || qualified3.IsEnclosing || qualified3.IsOutside
                          || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D                   c1  = qualified1.Qualified();
        CurveAdaptor2D         cu2 = qualified2.Qualified();
        CurveAdaptor2D         cu3 = qualified3.Qualified();
        FunctionTanCuCuCu func = new(c1, cu2, cu3);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = 0.0;
        umin[2]   = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]   = ConstraintCurveTool.FirstParameter(cu3);
        umax[1]   = 2 * M_PI;
        umax[2]   = ConstraintCurveTool.LastParameter(cu2);
        umax[3]   = ConstraintCurveTool.LastParameter(cu3);
        ufirst[1] = param1;
        ufirst[2] = param2;
        ufirst[3] = param3;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]    = ConstraintCurveTool.EpsX(cu3, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D centre1 = c1.Location;
            double   r1 = c1.Radius;
            Point2D point1 = new(centre1.XY() + r1 * new Xy(Math.Cos(ufirst[1]), Math.Sin(ufirst[1])));
            Vector2D tan1 = new(new Xy(-Math.Sin(ufirst[1]), Math.Cos(ufirst[1])));
            Point2D point2 = new(), point3 = new();
            //     gp_Vec2d Tan2,Tan3,Nor2,Nor3;
            Vector2D tan2 = new(), tan3 = new();
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            ConstraintCurveTool.D1(cu3, ufirst[3], out point3, out tan3);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                double   dist      = centre1.DistanceTo(centre);
                double   rsol      = _cirsol.Radius;
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (qualified1.IsUnqualified || (qualified1.IsEnclosing && rsol >= r1 && dist <= rsol)
                            || (qualified1.IsOutside && dist >= rsol)
                            || (qualified1.IsEnclosed && rsol <= r1 && dist <= rsol))
                    {
                        double angle1 = vec2.AngleTo(tan2);
                        if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle1 <= 0.0)
                                || (qualified2.IsOutside && angle1 >= 0)
                                || (qualified2.IsEnclosed && angle1 <= 0.0))
                        {
                            angle1 = vec3.AngleTo(tan3);
                            if (qualified3.IsUnqualified || (qualified3.IsEnclosing && angle1 <= 0.0)
                                    || (qualified3.IsOutside && angle1 >= 0)
                                    || (qualified3.IsEnclosed && angle1 <= 0.0))
                            {
                                _qualifier1 = qualified1.Qualifier();
                                _qualifier2 = qualified2.Qualifier();
                                _qualifier3 = qualified3.Qualifier();
                                _pararg1    = ufirst[1];
                                _pnttg1sol  = point1;
                                _pararg2    = ufirst[2];
                                _pnttg2sol  = point2;
                                _pararg3    = ufirst[3];
                                _pnttg3sol  = point3;
                                _wellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QualifiedCirc qualified1, in QualifiedCirc qualified2, in QCurve qualified3, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified)
                || !(qualified3.IsEnclosed || qualified3.IsEnclosing || qualified3.IsOutside
                          || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D                   c1  = qualified1.Qualified();
        Circle2D                   c2  = qualified2.Qualified();
        CurveAdaptor2D         cu3 = qualified3.Qualified();
        FunctionTanCuCuCu func = new(c1, c2, cu3);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = 0.0;
        umin[2]   = 0.0;
        umin[3]   = ConstraintCurveTool.FirstParameter(cu3);
        umax[1]   = 2 * M_PI;
        umax[2]   = 2 * M_PI;
        umax[3]   = ConstraintCurveTool.LastParameter(cu3);
        ufirst[1] = param1;
        ufirst[2] = param2;
        ufirst[3] = param3;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 2.0e-15 * M_PI;
        tol[3]    = ConstraintCurveTool.EpsX(cu3, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D centre1 = c1.Location;
            double   r1 = c1.Radius;
            Point2D point1 = new(centre1.XY() + r1 * new Xy(Math.Cos(ufirst[1]), Math.Sin(ufirst[1])));
            Vector2D tan1 = new(new Xy(-Math.Sin(ufirst[1]), Math.Cos(ufirst[1])));
            Point2D centre2 = c2.Location;
            double   r2 = c2.Radius;
            Point2D point2 = new(centre2.XY() + r2 * new Xy(Math.Cos(ufirst[2]), Math.Sin(ufirst[2])));
            Vector2D tan2 = new(new Xy(-Math.Sin(ufirst[2]), Math.Cos(ufirst[2])));
            Point2D point3 = new();
            Vector2D tan3 = new();
            ConstraintCurveTool.D1(cu3, ufirst[3], out point3, out tan3);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                double   dist      = centre1.DistanceTo(centre);
                double   rsol      = _cirsol.Radius;
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (qualified1.IsUnqualified || (qualified1.IsEnclosing && rsol >= r1 && dist <= rsol)
                            || (qualified1.IsOutside && dist >= rsol)
                            || (qualified1.IsEnclosed && rsol <= r1 && dist <= rsol))
                    {
                        // the second circle with its own qualifier (OCCT tested Qualified1 again: Kerf decision 17107)
                        dist = centre2.DistanceTo(centre);
                        if (qualified2.IsUnqualified || (qualified2.IsEnclosing && rsol >= r2 && dist <= rsol)
                                || (qualified2.IsOutside && dist >= rsol)
                                || (qualified2.IsEnclosed && rsol <= r2 && dist <= rsol))
                        {
                            Vector2D vec = new(point3, centre);
                            double   angle1 = vec.AngleTo(tan3);
                            if (qualified3.IsUnqualified || (qualified3.IsEnclosing && angle1 <= 0.0)
                                    || (qualified3.IsOutside && angle1 >= 0)
                                    || (qualified3.IsEnclosed && angle1 <= 0.0))
                            {
                                _qualifier1 = qualified1.Qualifier();
                                _qualifier2 = qualified2.Qualifier();
                                _qualifier3 = qualified3.Qualifier();
                                _pararg1    = ufirst[1];
                                _pnttg1sol  = point1;
                                _pararg2    = ufirst[2];
                                _pnttg2sol  = point2;
                                _pararg3    = ufirst[3];
                                _pnttg3sol  = point3;
                                _wellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QualifiedLin qualified1, in QCurve qualified2, in QCurve qualified3, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified)
                || !(qualified3.IsEnclosed || qualified3.IsEnclosing || qualified3.IsOutside
                          || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D                    l1  = qualified1.Qualified();
        CurveAdaptor2D         cu2 = qualified2.Qualified();
        CurveAdaptor2D         cu3 = qualified3.Qualified();
        FunctionTanCuCuCu func = new(l1, cu2, cu3);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = RealFirst();
        umin[2]   = ConstraintCurveTool.FirstParameter(cu2);
        umin[3]   = ConstraintCurveTool.FirstParameter(cu3);
        umax[1]   = RealLast();
        umax[2]   = ConstraintCurveTool.LastParameter(cu2);
        umax[3]   = ConstraintCurveTool.LastParameter(cu3);
        ufirst[1] = param1;
        ufirst[2] = param2;
        ufirst[3] = param3;
        tol[1]    = 1.0e-15;
        tol[2]    = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        tol[3]    = ConstraintCurveTool.EpsX(cu3, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D centre1 = l1.Location;
            Point2D point1 = new(centre1.XY() + ufirst[1] * l1.Direction.XY());
            Point2D point2 = new(), point3 = new();
            Vector2D tan2 = new(), tan3 = new();
            ConstraintCurveTool.D1(cu2, ufirst[2], out point2, out tan2);
            ConstraintCurveTool.D1(cu3, ufirst[3], out point3, out tan3);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;

                // creation vaariables intermediaires pour WNT
                Xy  dummy1 = centre.XY() - l1.Location.XY();
                Xy  dummy2 = new(-l1.Direction.Y, l1.Direction.X);
                double pscal = dummy1.Dot(dummy2);

                Vector2D tan1 = new(l1.Direction.XY());
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (qualified1.IsUnqualified || (qualified1.IsOutside && pscal <= 0.0)
                            || (qualified1.IsEnclosed && pscal >= 0.0))
                    {
                        Vector2D vec = new(point2, centre);
                        double   angle1 = vec.AngleTo(tan2);
                        if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle1 <= 0.0)
                                || (qualified2.IsOutside && angle1 >= 0)
                                || (qualified2.IsEnclosed && angle1 <= 0.0))
                        {
                            vec    = new Vector2D(point3, centre);
                            angle1 = vec.AngleTo(tan3);
                            if (qualified3.IsUnqualified || (qualified3.IsEnclosing && angle1 <= 0.0)
                                    || (qualified3.IsOutside && angle1 >= 0)
                                    || (qualified3.IsEnclosed && angle1 <= 0.0))
                            {
                                _qualifier1 = qualified1.Qualifier();
                                _qualifier2 = qualified2.Qualifier();
                                _qualifier3 = qualified3.Qualifier();
                                _pararg1    = ufirst[1];
                                _pnttg1sol  = point1;
                                _pararg2    = ufirst[2];
                                _pnttg2sol  = point2;
                                _pararg3    = ufirst[3];
                                _pnttg3sol  = point3;
                                _wellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QualifiedLin qualified1, in QualifiedLin qualified2, in QCurve qualified3, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified)
                || !(qualified3.IsEnclosed || qualified3.IsEnclosing || qualified3.IsOutside
                          || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D                    l1  = qualified1.Qualified();
        Line2D                    l2  = qualified2.Qualified();
        CurveAdaptor2D         cu3 = qualified3.Qualified();
        FunctionTanCuCuCu func = new(l1, l2, cu3);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = RealFirst();
        umin[2]   = RealFirst();
        umin[3]   = ConstraintCurveTool.FirstParameter(cu3);
        umax[1]   = RealLast();
        umax[2]   = RealLast();
        umax[3]   = ConstraintCurveTool.LastParameter(cu3);
        ufirst[1] = param1;
        ufirst[2] = param2;
        ufirst[3] = param3;
        tol[1]    = 1.0e-15;
        tol[2]    = 1.0e-15;
        tol[3]    = ConstraintCurveTool.EpsX(cu3, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D centre1 = l1.Location;
            Point2D point1 = new(centre1.XY() + ufirst[1] * l1.Direction.XY());
            Point2D centre2 = l2.Location;
            Point2D point2 = new(centre2.XY() + ufirst[2] * l2.Direction.XY());
            Point2D point3 = new();
            Vector2D tan3 = new();
            ConstraintCurveTool.D1(cu3, ufirst[3], out point3, out tan3);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                // the side of L1 from a point of it (OCCT measured from the origin: Kerf decision 17107)
                double   pscal = (centre.XY() - l1.Location.XY()).Dot(new Xy(-l1.Direction.Y, l1.Direction.X));
                if (qualified1.IsUnqualified || (qualified1.IsOutside && pscal <= 0.0)
                        || (qualified1.IsEnclosed && pscal >= 0.0))
                {
                    Vector2D tan1 = new(l1.Direction.XY());
                    Vector2D tan2 = new(l2.Direction.XY());
                    double   normetan1 = tan1.Magnitude;
                    double   normetan2 = tan2.Magnitude;
                    double   normetan3 = tan3.Magnitude;
                    Vector2D vec1 = new(point1, centre);
                    Vector2D vec2 = new(point2, centre);
                    Vector2D vec3 = new(point3, centre);
                    double   normevec1 = vec1.Magnitude;
                    double   normevec2 = vec2.Magnitude;
                    double   normevec3 = vec3.Magnitude;
                    double   dot1, dot2, dot3;
                    if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                    {
                        dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                    }
                    else
                    {
                        dot1 = 0.0;
                    }
                    if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                    {
                        dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                    }
                    else
                    {
                        dot2 = 0.0;
                    }
                    if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                    {
                        dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                    }
                    else
                    {
                        dot3 = 0.0;
                    }
                    Tol = 1.0e-12;
                    if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                    {
                        if (qualified2.IsUnqualified || (qualified2.IsOutside && pscal <= 0.0)
                                || (qualified2.IsEnclosed && pscal >= 0.0))
                        {
                            double angle1 = vec3.AngleTo(tan3);
                            if (qualified3.IsUnqualified || (qualified3.IsEnclosing && angle1 <= 0.0)
                                    || (qualified3.IsOutside && angle1 >= 0)
                                    || (qualified3.IsEnclosed && angle1 <= 0.0))
                            {
                                _qualifier1 = qualified1.Qualifier();
                                _qualifier2 = qualified2.Qualifier();
                                _qualifier3 = qualified3.Qualifier();
                                _pararg1    = ufirst[1];
                                _pnttg1sol  = point1;
                                _pararg2    = ufirst[2];
                                _pnttg2sol  = point2;
                                _pararg3    = ufirst[3];
                                _pnttg3sol  = point3;
                                _wellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QCurve qualified1, in QCurve qualified2, in Point2D point3, double param1, double param2, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D                   c1 = new(new Axis2D(point3, new Direction2D(Direction2D.D.X)), 0.0);
        CurveAdaptor2D         cu1 = qualified1.Qualified();
        CurveAdaptor2D         cu2 = qualified2.Qualified();
        FunctionTanCuCuCu func = new(c1, cu1, cu2);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = 0.0;
        umin[2]   = ConstraintCurveTool.FirstParameter(cu1);
        umin[3]   = ConstraintCurveTool.FirstParameter(cu2);
        umax[1]   = 2 * M_PI;
        umax[2]   = ConstraintCurveTool.LastParameter(cu1);
        umax[3]   = ConstraintCurveTool.LastParameter(cu2);
        ufirst[1] = M_PI;
        ufirst[2] = param1;
        ufirst[3] = param2;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        tol[3]    = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D point1 = new(), point2 = new();
            //     gp_Vec2d Tan1,Tan2,Nor1,Nor2;
            Vector2D tan1 = new(), tan2 = new();
            ConstraintCurveTool.D1(cu1, ufirst[2], out point1, out tan1);
            ConstraintCurveTool.D1(cu2, ufirst[3], out point2, out tan2);
            AnalyticCirc2d3Tan circ = new(point3, point1, point2, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                Vector2D tan3 = new(-Math.Sin(ufirst[1]), Math.Cos(ufirst[1]));
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double angle1 = vec1.AngleTo(tan1);
                    if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                            || (qualified1.IsOutside && angle1 >= 0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                    {
                        // the second curve with its own qualifier (OCCT tested Qualified1 again: Kerf decision 17107)
                        angle1 = vec2.AngleTo(tan2);
                        if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle1 <= 0.0)
                                || (qualified2.IsOutside && angle1 >= 0)
                                || (qualified2.IsEnclosed && angle1 <= 0.0))
                        {
                            _qualifier1 = qualified1.Qualifier();
                            _qualifier2 = qualified2.Qualifier();
                            _qualifier3 = Qualifier.Noqualifier;
                            _pararg1    = ufirst[2];
                            _pnttg1sol  = point1;
                            _pararg2    = ufirst[3];
                            _pnttg2sol  = point2;
                            _pararg3    = 0.0;
                            _pnttg3sol  = point3;
                            _wellDone   = true;
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QCurve qualified1, in Point2D point2, in Point2D Point3, double param1, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D                   c1 = new(new Axis2D(point2, new Direction2D(Direction2D.D.X)), 0.0);
        Circle2D                   c2 = new(new Axis2D(Point3, new Direction2D(Direction2D.D.X)), 0.0);
        CurveAdaptor2D         cu1 = qualified1.Qualified();
        FunctionTanCuCuCu func = new(c1, c2, cu1);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = 0.0;
        umin[2]   = 0.0;
        umin[3]   = ConstraintCurveTool.FirstParameter(cu1);
        umax[1]   = 2 * M_PI;
        umax[2]   = 2 * M_PI;
        umax[3]   = ConstraintCurveTool.LastParameter(cu1);
        ufirst[1] = M_PI;
        ufirst[2] = M_PI;
        ufirst[3] = param1;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 2.0e-15 * M_PI;
        tol[3]    = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D point3 = new();
            //     gp_Vec2d Tan3,Nor3;
            Vector2D tan3 = new();
            ConstraintCurveTool.D1(cu1, ufirst[3], out point3, out tan3);
            AnalyticCirc2d3Tan circ = new(point2, Point3, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                Vector2D tan2 = new(-Math.Sin(ufirst[2]), Math.Cos(ufirst[2]));
                Vector2D tan1 = new(-Math.Sin(ufirst[1]), Math.Cos(ufirst[1]));
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point2, centre);
                Vector2D vec2 = new(Point3, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double angle1 = vec1.AngleTo(tan1);
                    if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 <= 0.0)
                            || (qualified1.IsOutside && angle1 >= 0) || (qualified1.IsEnclosed && angle1 <= 0.0))
                    {
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = Qualifier.Noqualifier;
                        _qualifier3 = Qualifier.Noqualifier;
                        _pararg1    = ufirst[3];
                        _pnttg1sol  = point3;
                        _pararg2    = 0.0;
                        _pnttg2sol  = point2;
                        _pararg3    = 0.0;
                        _pnttg3sol  = Point3;
                        _wellDone   = true;
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QualifiedLin qualified1, in QCurve qualified2, in Point2D point3, double param1, double param2, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D                    l1  = qualified1.Qualified();
        CurveAdaptor2D         cu2 = qualified2.Qualified();
        Circle2D                   c3 = new(new Axis2D(point3, new Direction2D(Direction2D.D.X)), 0.0);
        FunctionTanCuCuCu func = new(c3, l1, cu2);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[2]   = RealFirst();
        umin[3]   = ConstraintCurveTool.FirstParameter(cu2);
        umin[1]   = 0.0;
        umax[2]   = RealLast();
        umax[3]   = ConstraintCurveTool.LastParameter(cu2);
        umax[1]   = 2 * M_PI;
        ufirst[2] = param1;
        ufirst[3] = param2;
        ufirst[1] = M_PI;
        tol[1]    = 2.0e-15;
        tol[2]    = 1.0e-15;
        tol[3]    = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D centre1 = l1.Location;
            Point2D point1 = new(centre1.XY() + ufirst[2] * l1.Direction.XY());
            Point2D point2 = new();
            Vector2D tan2 = new();
            // Cu2's parameter is Ufirst(3) (OCCT took Ufirst(2), the one on L1: Kerf decision 17107)
            ConstraintCurveTool.D1(cu2, ufirst[3], out point2, out tan2);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                // the side of L1 from a point of it (OCCT measured from the origin: Kerf decision 17107)
                double   pscal = (centre.XY() - l1.Location.XY()).Dot(new Xy(-l1.Direction.Y, l1.Direction.X));
                Vector2D tan1 = new(l1.Direction.XY());
                Vector2D tan3 = new(-Math.Sin(ufirst[1]), Math.Cos(ufirst[1]));
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (qualified1.IsUnqualified || (qualified1.IsOutside && pscal <= 0.0)
                            || (qualified1.IsEnclosed && pscal >= 0.0))
                    {
                        double angle1 = vec2.AngleTo(tan2);
                        if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle1 <= 0.0)
                                || (qualified2.IsOutside && angle1 >= 0)
                                || (qualified2.IsEnclosed && angle1 <= 0.0))
                        {
                            _qualifier1 = qualified1.Qualifier();
                            _qualifier2 = qualified2.Qualifier();
                            _qualifier3 = Qualifier.Noqualifier;
                            _pararg1    = ufirst[2];
                            _pnttg1sol  = point1;
                            _pararg2    = ufirst[3];
                            _pnttg2sol  = point2;
                            _pararg3    = 0.0;
                            _pnttg3sol  = point3;
                            _wellDone   = true;
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QualifiedCirc qualified1, in QualifiedLin qualified2, in QCurve qualified3, double param1, double param2, double param3, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified)
                || !(qualified3.IsEnclosed || qualified3.IsEnclosing || qualified3.IsOutside
                          || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D                   c1  = qualified1.Qualified();
        Line2D                    l2  = qualified2.Qualified();
        CurveAdaptor2D         cu3 = qualified3.Qualified();
        FunctionTanCuCuCu func = new(c1, l2, cu3);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = 0.0;
        umin[2]   = RealFirst();
        umin[3]   = ConstraintCurveTool.FirstParameter(cu3);
        umax[1]   = 2 * M_PI;
        umax[2]   = RealLast();
        umax[3]   = ConstraintCurveTool.LastParameter(cu3);
        ufirst[1] = param1;
        ufirst[2] = param2;
        ufirst[3] = param3;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 1.0e-15;
        tol[3]    = ConstraintCurveTool.EpsX(cu3, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            func.Value(ufirst, umin);
            root.Root(ufirst);
            Point2D centre1 = c1.Location;
            double   r1 = c1.Radius;
            Point2D point1 = new(centre1.XY() + r1 * new Xy(Math.Cos(ufirst[1]), Math.Sin(ufirst[1])));
            Point2D centre2 = l2.Location;
            Point2D point2 = new(centre2.XY() + ufirst[2] * l2.Direction.XY());
            Point2D point3 = new();
            Vector2D tan3 = new();
            ConstraintCurveTool.D1(cu3, ufirst[3], out point3, out tan3);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                Vector2D tan1 = new(-Math.Sin(ufirst[1]), Math.Cos(ufirst[1]));
                Vector2D tan2 = new(l2.Direction.XY());
                double   normetan1 = tan1.Magnitude;
                double   normetan2 = tan2.Magnitude;
                double   normetan3 = tan3.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double dist = centre1.DistanceTo(centre);
                    double rsol = _cirsol.Radius;
                    if (qualified1.IsUnqualified || (qualified1.IsEnclosing && rsol >= r1 && dist <= rsol)
                            || (qualified1.IsOutside && dist >= rsol)
                            || (qualified1.IsEnclosed && rsol <= r1 && dist <= rsol))
                    {
                        // the side of L2 from a point of it (OCCT measured from the origin: Kerf decision 17107)
                        double pscal = (centre.XY() - l2.Location.XY()).Dot(new Xy(-l2.Direction.Y, l2.Direction.X));
                        if (qualified2.IsUnqualified || (qualified2.IsOutside && pscal <= 0.0)
                                || (qualified2.IsEnclosed && pscal >= 0.0))
                        {
                            double angle1 = vec3.AngleTo(tan3);
                            if (qualified3.IsUnqualified || (qualified3.IsEnclosing && angle1 <= 0.0)
                                    || (qualified3.IsOutside && angle1 >= 0)
                                    || (qualified3.IsEnclosed && angle1 <= 0.0))
                            {
                                _qualifier1 = qualified1.Qualifier();
                                _qualifier2 = qualified2.Qualifier();
                                _qualifier3 = qualified3.Qualifier();
                                _pararg1    = ufirst[1];
                                _pnttg1sol  = point1;
                                _pararg2    = ufirst[2];
                                _pnttg2sol  = point2;
                                _pararg3    = ufirst[3];
                                _pnttg3sol  = point3;
                                _wellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Circ2d3TanIter(in QualifiedCirc qualified1, in QCurve qualified2, in Point2D point3, double param1, double param2, double tolerance)
    {
        _theSame1 = false;
        _theSame3 = false;
        _pararg1  = 0.0;
        _pararg2  = 0.0;
        _pararg3  = 0.0;

        double Tol = Math.Abs(tolerance);
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        _qualifier3 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D                   c1  = qualified1.Qualified();
        CurveAdaptor2D         cu2 = qualified2.Qualified();
        Circle2D                   c3 = new(new Axis2D(point3, new Direction2D(Direction2D.D.X)), 0.0);
        FunctionTanCuCuCu func = new(c1, c3, cu2);
        math_Vector                 umin = new(1, 3);
        math_Vector                 umax = new(1, 3);
        math_Vector                 ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        umin[1]   = 0.0;
        umin[3]   = ConstraintCurveTool.FirstParameter(cu2);
        umin[2]   = 0.0;
        umax[1]   = 2 * M_PI;
        umax[3]   = ConstraintCurveTool.LastParameter(cu2);
        umax[2]   = 2 * M_PI;
        ufirst[1] = param1;
        ufirst[2] = M_PI;
        ufirst[3] = param2;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 2.0e-15 * M_PI;
        tol[3]    = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolerance));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            func.Value(ufirst, umin);
            Point2D centre1 = c1.Location;
            double   r1 = c1.Radius;
            Point2D point1 = new(centre1.XY() + r1 * new Xy(Math.Cos(ufirst[1]), Math.Sin(ufirst[1])));
            Point2D point2 = new();
            //     gp_Vec2d Tan2,Nor2;
            Vector2D tan2 = new();
            // Cu2's parameter is Ufirst(3) (OCCT took Ufirst(2), the one on the point's circle: Kerf decision 17107)
            ConstraintCurveTool.D1(cu2, ufirst[3], out point2, out tan2);
            AnalyticCirc2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone)
            {
                _cirsol = circ.ThisSolution(1);
                Point2D centre = _cirsol.Location;
                Vector2D tan1 = new(-Math.Sin(ufirst[1]), Math.Cos(ufirst[1]));
                Vector2D tan3 = new(-Math.Sin(ufirst[2]), Math.Cos(ufirst[2]));
                double   normetan2 = tan2.Magnitude;
                Vector2D vec1 = new(point1, centre);
                Vector2D vec2 = new(point2, centre);
                Vector2D vec3 = new(point3, centre);
                double   normevec1 = vec1.Magnitude;
                double   normevec2 = vec2.Magnitude;
                double   normevec3 = vec3.Magnitude;
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution())
                {
                    dot1 = vec1.Dot(tan1) / (normevec1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = vec2.Dot(tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution())
                {
                    dot3 = vec3.Dot(tan3) / (normevec3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double dist = centre1.DistanceTo(centre);
                    double rsol = _cirsol.Radius;
                    if (qualified1.IsUnqualified || (qualified1.IsEnclosing && rsol >= r1 && dist <= rsol)
                            || (qualified1.IsOutside && dist >= rsol)
                            || (qualified1.IsEnclosed && rsol <= r1 && dist <= rsol))
                    {
                        double angle1 = vec2.AngleTo(tan2);
                        if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle1 <= 0.0)
                                || (qualified2.IsOutside && angle1 >= 0)
                                || (qualified2.IsEnclosed && angle1 <= 0.0))
                        {
                            _qualifier1 = qualified1.Qualifier();
                            _qualifier2 = qualified2.Qualifier();
                            _qualifier3 = Qualifier.Noqualifier;
                            _pararg1    = ufirst[1];
                            _pnttg1sol  = point1;
                            _pararg2    = ufirst[2];
                            _pnttg2sol  = point2;
                            _pararg3    = 0.0;
                            _pnttg3sol  = point3;
                            _wellDone   = true;
                        }
                    }
                }
            }
        }
    }

    /// <summary>True if the construction found a solution.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The solution (not checked: an unset circle if the construction failed).</summary>
    public Circle2D ThisSolution()
    {
        return _cirsol;
    }

    /// <summary>The qualifiers of the arguments for the solution.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void WhichQualifier(out Qualifier qualif1, out Qualifier qualif2, out Qualifier qualif3)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            qualif1 = _qualifier1;
            qualif2 = _qualifier2;
            qualif3 = _qualifier3;
        }
    }

    /// <summary>The tangency point on the first argument: its parameter on the solution and on the argument.</summary>
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
                parSol = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg1sol); // OCCT stored 0 (Kerf decision 17107)
                parArg = _pararg1;
                pntSol = _pnttg1sol;
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The tangency point on the second argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void Tangency2(out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            parSol = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg2sol); // OCCT stored 0 (Kerf decision 17107)
            parArg = _pararg2;
            pntSol = _pnttg2sol;
        }
    }

    /// <summary>The tangency point on the third argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void Tangency3(out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            parSol = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol, _pnttg3sol); // OCCT stored 0 (Kerf decision 17107)
            parArg = _pararg3;
            pntSol = _pnttg3sol;
        }
    }

    /// <summary>True if the solution is the first argument (never, as OCCT sets it).</summary>
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

    /// <summary>True if the solution is the second argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame2()
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        // OCCT tested TheSame3; the iterative solution is never an argument (Kerf decision 17107)
        return false;
    }

    /// <summary>
    /// True if the solution is the third argument (OCCT answered true whenever the construction was done, so
    /// Geom2dGcc_Circ2d3Tan's Tangency3 threw for every iterative solution: Kerf decision 17107).
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame3
    {
        get
        {
            if (!_wellDone)
            {
                throw new NotDoneException();
            }

            return _theSame3;
        }
    }
}
