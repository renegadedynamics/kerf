// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

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
public sealed class Geom2dGcc_Circ2d2TanOnIter
{
    private bool WellDone;
    private Circle2D cirsol = new();
    private GccEnt_Position qualifier1;
    private GccEnt_Position qualifier2;
    private bool TheSame1;
    // OCCT's TheSame2 is only ever set to false and never read (IsTheSame2 returns false): left out.
    private Point2D pnttg1sol = new();
    private Point2D pnttg2sol = new();
    private Point2D pntcen = new();
    private double par1sol;
    private double par2sol;
    private double pararg1;
    private double pararg2;
    private double parcen3;

    public Geom2dGcc_Circ2d2TanOnIter(in GccEnt_QualifiedLin Qualified1, in Geom2dGcc_QCurve Qualified2, in Line2D OnLine, double Param1, double Param2, double Param3, double Tolang)
    {

        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        double Tol = Math.Abs(Tolang);
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Direction2D            dirx = new(Direction2D.D.X);
        Line2D            L1  = Qualified1.Qualified();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = RealFirst();
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = RealFirst();
        Umin[4]         = 0.0;
        Umax[1]         = RealLast();
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = RealLast();
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = Param3;
        tol[1]          = 1.0e-15;
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Tolang);
        tol[3]          = tol[1];
        tol[4]          = tol[1];
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(Param1, L1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(Param3, OnLine);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(L1, Cu2, OnLine, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1,Nor2;
            Vector2D Tan1, Tan2;
            global::Kerf.Engine.Geometry.ElCLib.D1(Ufirst[1], L1, out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Vector2D Tan3 = new(OnLine.Direction().XY());
            Point2D point3new = new(OnLine.Location().XY() + Ufirst[3] * Tan3.XY());
            double   dist1 = point3new.Distance(point1);
            double   dist2 = point3new.Distance(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, point3new);
                Vector2D Vec2 = new(point2, point3new);
                double   normevec2 = Vec2.Magnitude();
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double pscal = point3new.XY().Dot(new Xy(-L1.Direction().Y(), L1.Direction().X()));
                if (Qualified1.IsUnqualified() || (Qualified1.IsOutside() && pscal <= 0.0)
                        || (Qualified1.IsEnclosed() && pscal >= 0.0))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pnttg1sol  = point1;
                        pararg1    = Ufirst[1];
                        par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                        pnttg2sol  = point2;
                        pararg2    = Ufirst[2];
                        par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                        pntcen     = point3new;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in Geom2dGcc_QCurve Qualified1, in Geom2dGcc_QCurve Qualified2, in Line2D OnLine, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = RealFirst();
        Umin[4]         = 0.0;
        Umax[1]         = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = RealLast();
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = Param3;
        tol[1]          = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = 1.0e-15;
        tol[4]          = Tol / 10.0;
        Point2D point1 = Geom2dGcc_CurveTool.Value(Cu1, Param1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(Param3, OnLine);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(Cu1, Cu2, OnLine, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Vector2D Tan1, Tan2;
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[1], out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Vector2D Tan3 = new(OnLine.Direction().XY());
            Point2D point3new = new(OnLine.Location().XY() + Ufirst[3] * Tan3.XY());
            double   dist1 = point3new.Distance(point1);
            double   dist2 = point3new.Distance(point2);
            if ((dist1 + dist2) / 2.0 < Tol)
            {
                return;
            }
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, point3new);
                Vector2D Vec2 = new(point2, point3new);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   angle1, angle2;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = Vec1.Angle(Tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && angle1 <= 0.0)
                        || (Qualified1.IsOutside() && angle1 >= 0.0) || (Qualified1.IsEnclosed() && angle1 <= 0.0))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pnttg1sol  = point1;
                        pararg1    = Ufirst[1];
                        par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                        pnttg2sol  = point2;
                        pararg2    = Ufirst[2];
                        par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                        pntcen     = point3new;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in Geom2dGcc_QCurve Qualified1, in Point2D Point2, in Line2D OnLine, double Param1, double Param2, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        math_Vector         Umin = new(1, 3);
        math_Vector         Umax = new(1, 3);
        math_Vector         Ufirst = new(1, 3);
        math_Vector         tol = new(1, 3);
        Umin[1]         = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]         = RealFirst();
        Umin[3]         = 0.0;
        Umax[1]         = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]         = RealLast();
        Umax[3]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        tol[1]          = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[2]          = 1.0e-15;
        tol[3]          = Tol / 10.0;
        Point2D point1 = Geom2dGcc_CurveTool.Value(Cu1, Param1);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(Param2, OnLine);
        Ufirst[3]       = (point3.Distance(Point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(Cu1, Point2, OnLine, Max(Ufirst[3], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Point2D point1new, point3new;
            Vector2D Tan1, Tan3;
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[1], out point1new, out Tan1);
            global::Kerf.Engine.Geometry.ElCLib.D1(Ufirst[2], OnLine, out point3new, out Tan3);
            double dist1 = point3new.Distance(point1new);
            double dist2 = point3new.Distance(Point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = Tan1.Magnitude();
                Vector2D Vec1 = new(point1new, point3new);
                double   normevec1 = Vec1.Magnitude();
                double   angle1;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = Vec1.Angle(Tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && angle1 <= 0.0)
                        || (Qualified1.IsOutside() && angle1 >= 0.0) || (Qualified1.IsEnclosed() && angle1 <= 0.0))
                {
                    qualifier1 = Qualified1.Qualifier();
                    qualifier2 = GccEnt_noqualifier;
                    pnttg1sol  = point1new;
                    pararg1    = Ufirst[1];
                    par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                    pnttg2sol  = Point2;
                    pararg2    = Ufirst[2]; // OCCT: the line parameter of the centre, not a parameter on the point
                    par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                    pntcen     = point3new;
                    parcen3    = Ufirst[3]; // OCCT: Ufirst(3) is the radius here, not the centre's parameter
                    WellDone   = true;
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in GccEnt_QualifiedCirc Qualified1, in Geom2dGcc_QCurve Qualified2, in Line2D OnLine, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Circle2D           C1  = Qualified1.Qualified();
        double              R1  = C1.Radius();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = RealFirst();
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = RealFirst();
        Umin[4]         = 0.0;
        Umax[1]         = RealLast();
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = RealLast();
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = Param3;
        tol[1]          = 2.0e-15 * M_PI;
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = 1.0e-15;
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(Param1, C1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(Param3, OnLine);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(C1, Cu2, OnLine, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1,Nor2;
            Vector2D Tan1, Tan2, Nor2;
            global::Kerf.Engine.Geometry.ElCLib.D2(Ufirst[1], C1, out point1, out Tan1, out Nor2);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            OnLine.Direction().XY(); // (OCCT_DEBUG makes a vector of it)
            point3       = global::Kerf.Engine.Geometry.ElCLib.Value(Ufirst[1], OnLine); // OCCT: Ufirst(1) is the parameter on C1, not the centre's on OnLine (Ufirst(3))
            double dist1 = point3.Distance(point1);
            double dist2 = point3.Distance(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, point3);
                Vector2D Vec2 = new(point2, point3);
                double   normevec2 = Vec2.Magnitude();
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double dist = C1.Location().Distance(point3);
                double Rsol = cirsol.Radius();
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R1 && dist <= Rsol)
                        || (Qualified1.IsOutside() && dist >= Rsol)
                        || (Qualified1.IsEnclosed() && Rsol <= R1 && dist <= Rsol))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pnttg1sol  = point1;
                        pararg1    = Ufirst[1];
                        par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                        pnttg2sol  = point2;
                        pararg2    = Ufirst[2];
                        par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                        pntcen     = point3;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in GccEnt_QualifiedCirc Qualified1, in Geom2dGcc_QCurve Qualified2, in Circle2D OnCirc, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Circle2D           C1  = Qualified1.Qualified();
        double              R1  = C1.Radius();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = RealFirst();
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = RealFirst();
        Umin[4]         = 0.0;
        Umax[1]         = RealLast();
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = RealLast();
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = Param3;
        tol[1]          = 2.0e-15 * M_PI;
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = 2.0e-15 * M_PI;
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(Param1, C1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(Param3, OnCirc);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(C1, Cu2, OnCirc, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1;
            Vector2D Tan1, Tan2;
            global::Kerf.Engine.Geometry.ElCLib.D1(Ufirst[1], C1, out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            point3       = global::Kerf.Engine.Geometry.ElCLib.Value(Ufirst[3], OnCirc);
            double dist1 = point3.Distance(point1);
            double dist2 = point3.Distance(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, point3);
                Vector2D Vec2 = new(point2, point3);
                double   normevec2 = Vec2.Magnitude();
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double dist = C1.Location().Distance(point3);
                double Rsol = cirsol.Radius();
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R1 && dist <= Rsol)
                        || (Qualified1.IsOutside() && dist >= Rsol)
                        || (Qualified1.IsEnclosed() && Rsol <= R1 && dist <= Rsol))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pnttg1sol  = point1;
                        pararg1    = Ufirst[1];
                        par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                        pnttg2sol  = point2;
                        pararg2    = Ufirst[2];
                        par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                        pntcen     = point3;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in GccEnt_QualifiedLin Qualified1, in Geom2dGcc_QCurve Qualified2, in Circle2D OnCirc, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Line2D            L1  = Qualified1.Qualified();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = RealFirst();
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = RealFirst();
        Umin[4]         = 0.0;
        Umax[1]         = RealLast();
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = RealLast();
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = Param3;
        tol[1]          = 1.0e-15;
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = 2.0e-15 * M_PI;
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(Param1, L1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(Param3, OnCirc);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(L1, Cu2, OnCirc, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Point2D point1new, point2new;
            Vector2D Tan1, Tan2;
            global::Kerf.Engine.Geometry.ElCLib.D1(Ufirst[1], L1, out point1new, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2new, out Tan2);
            point3       = global::Kerf.Engine.Geometry.ElCLib.Value(Ufirst[3], OnCirc);
            double dist1 = point3.Distance(point1new);
            double dist2 = point3.Distance(point2new);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1new, point3);
                Vector2D Vec2 = new(point2new, point3);
                double   normevec2 = Vec2.Magnitude();
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double pscal = point3.XY().Dot(new Xy(-L1.Direction().Y(), L1.Direction().X()));
                if (Qualified1.IsUnqualified() || (Qualified1.IsOutside() && pscal <= 0.0)
                        || (Qualified1.IsEnclosed() && pscal >= 0.0))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pnttg1sol  = point1new;
                        pararg1    = Ufirst[1];
                        par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                        pnttg2sol  = point2new;
                        pararg2    = Ufirst[2];
                        par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                        pntcen     = point3;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in Geom2dGcc_QCurve Qualified1, in Geom2dGcc_QCurve Qualified2, in Circle2D OnCirc, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = RealFirst();
        Umin[4]         = 0.0;
        Umax[1]         = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = RealLast();
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = Param3;
        tol[1]          = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = 2.0e-15 * M_PI;
        tol[4]          = Tol / 10.0;
        Point2D point1 = Geom2dGcc_CurveTool.Value(Cu1, Param1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        double   R1     = OnCirc.Radius();
        Point2D point3 = new(OnCirc.Location().XY() + R1 * new Xy(Math.Cos(Param3), Math.Sin(Param3)));
        Ufirst[4] = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(Cu1, Cu2, OnCirc, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            //     gp_Vec2d Tan1,Tan2,Nor1;
            Vector2D Tan1, Tan2;
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[1], out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            point3 =
                new Point2D(OnCirc.Location().XY() + R1 * new Xy(Math.Cos(Ufirst[3]), Math.Sin(Ufirst[3])));
            double dist1 = point3.Distance(point1);
            double dist2 = point3.Distance(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, point3);
                Vector2D Vec2 = new(point2, point3);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   angle1, angle2;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = Vec1.Angle(Tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && angle1 <= 0.0)
                        || (Qualified1.IsOutside() && angle1 >= 0.0) || (Qualified1.IsEnclosed() && angle1 <= 0.0))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pararg1    = Ufirst[1];
                        par1sol    = 0.0; // OCCT: 0, not the parameter of the tangency point on the solution
                        pnttg1sol  = point1;
                        pararg2    = Ufirst[2];
                        pnttg2sol  = point2;
                        par2sol    = pnttg2sol.Distance(pnttg1sol); // OCCT: the distance between the tangency points, not a parameter on the solution
                        pntcen     = point3;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in Geom2dGcc_QCurve Qualified1, in Point2D Point2, in Circle2D OnCirc, double Param1, double Param2, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        math_Vector         Umin = new(1, 3);
        math_Vector         Umax = new(1, 3);
        math_Vector         Ufirst = new(1, 3);
        math_Vector         tol = new(1, 3);
        Umin[1]         = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]         = RealFirst();
        Umin[3]         = 0.0;
        Umax[1]         = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]         = RealLast();
        Umax[3]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        tol[1]          = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[2]          = 2.0e-15 * M_PI;
        tol[3]          = Tol / 10.0;
        Point2D point1 = Geom2dGcc_CurveTool.Value(Cu1, Param1);
        Point2D point3 = global::Kerf.Engine.Geometry.ElCLib.Value(Param2, OnCirc);
        Ufirst[3]       = (point3.Distance(Point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(Cu1, Point2, OnCirc, Max(Ufirst[3], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Point2D point1new, point3new;
            Vector2D Tan1, Tan3;
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[1], out point1new, out Tan1);
            global::Kerf.Engine.Geometry.ElCLib.D1(Ufirst[2], OnCirc, out point3new, out Tan3);
            double dist1 = point3new.Distance(point1new);
            double dist2 = point3new.Distance(Point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3new, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = Tan1.Magnitude();
                Vector2D Vec1 = new(point1new, point3new);
                double   normevec1 = Vec1.Magnitude();
                double   angle1;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = Vec1.Angle(Tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && angle1 <= 0.0)
                        || (Qualified1.IsOutside() && angle1 >= 0.0) || (Qualified1.IsEnclosed() && angle1 <= 0.0))
                {
                    qualifier1 = Qualified1.Qualifier();
                    qualifier2 = GccEnt_noqualifier;
                    pnttg1sol  = point1new;
                    pararg1    = Ufirst[1];
                    par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                    pnttg2sol  = Point2;
                    pararg2    = 0.0;
                    par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                    pntcen     = point3new;
                    parcen3    = Ufirst[3]; // OCCT: Ufirst(3) is the radius here, not the centre's parameter
                    WellDone   = true;
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in Geom2dGcc_QCurve Qualified1, in Geom2dGcc_QCurve Qualified2, CurveAdaptor2D OnCurv, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        double Tol = Math.Abs(Tolerance);
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = Geom2dGcc_CurveTool.FirstParameter(OnCurv);
        Umin[4]         = 0.0;
        Umax[1]         = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = Geom2dGcc_CurveTool.LastParameter(OnCurv);
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = Param3;
        tol[1]          = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = Geom2dGcc_CurveTool.EpsX(OnCurv, Math.Abs(Tolerance));
        tol[4]          = Tol / 10.0;
        Point2D point1 = Geom2dGcc_CurveTool.Value(Cu1, Param1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = Geom2dGcc_CurveTool.Value(OnCurv, Param3);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(Cu1, Cu2, OnCurv, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Vector2D Tan1, Tan2, Tan3;
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[1], out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Geom2dGcc_CurveTool.D1(OnCurv, Ufirst[3], out point3, out Tan3);
            double dist1 = point3.Distance(point1);
            double dist2 = point3.Distance(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, point3);
                Vector2D Vec2 = new(point2, point3);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   angle1, angle2;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = Vec1.Angle(Tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && angle1 <= 0.0)
                        || (Qualified1.IsOutside() && angle1 >= 0.0) || (Qualified1.IsEnclosed() && angle1 <= 0.0))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pararg1    = Ufirst[1];
                        par1sol    = 0.0; // OCCT: 0, not the parameter of the tangency point on the solution
                        pnttg1sol  = point1;
                        pararg2    = Ufirst[2];
                        pnttg2sol  = point2;
                        par2sol    = pnttg2sol.Distance(pnttg1sol); // OCCT: the distance between the tangency points, not a parameter on the solution
                        pntcen     = point3;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in GccEnt_QualifiedCirc Qualified1, in Geom2dGcc_QCurve Qualified2, CurveAdaptor2D OnCurv, double Param1, double Param2, double ParamOn, double Tolerance)
    {

        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Circle2D           C1  = Qualified1.Qualified();
        double              R1  = C1.Radius();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = RealFirst();
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = Geom2dGcc_CurveTool.FirstParameter(OnCurv);
        Umin[4]         = 0.0;
        Umax[1]         = RealLast();
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = Geom2dGcc_CurveTool.LastParameter(OnCurv);
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = ParamOn;
        tol[1]          = 2.0e-15 * M_PI;
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = Geom2dGcc_CurveTool.EpsX(OnCurv, Math.Abs(Tolerance));
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(Param1, C1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = Geom2dGcc_CurveTool.Value(OnCurv, ParamOn);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(C1, Cu2, OnCurv, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Vector2D Tan1, Tan2, Tan3;
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Geom2dGcc_CurveTool.D1(OnCurv, Ufirst[3], out point3, out Tan3);
            global::Kerf.Engine.Geometry.ElCLib.D1(Ufirst[1], C1, out point1, out Tan1);
            double dist1 = point3.Distance(point1);
            double dist2 = point3.Distance(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                Direction2D dirx = new(Direction2D.D.X);
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(new Point2D(point1.XY()), new Point2D(point3.XY()));
                Vector2D Vec2 = new(new Point2D(point2.XY()), new Point2D(point3.XY()));
                double   normevec2 = Vec2.Magnitude();
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double dist = C1.Location().Distance(point3);
                double Rsol = cirsol.Radius();
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R1 && dist <= Rsol)
                        || (Qualified1.IsOutside() && dist >= Rsol)
                        || (Qualified1.IsEnclosed() && Rsol <= R1 && dist <= Rsol))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pnttg1sol  = point1;
                        pararg1    = Ufirst[1];
                        par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                        pnttg2sol  = point2;
                        pararg2    = Ufirst[2];
                        par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                        pntcen     = point3;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in GccEnt_QualifiedLin Qualified1, in Geom2dGcc_QCurve Qualified2, CurveAdaptor2D OnCurv, double Param1, double Param2, double ParamOn, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        Line2D            L1  = Qualified1.Qualified();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        math_Vector         Umin = new(1, 4);
        math_Vector         Umax = new(1, 4);
        math_Vector         Ufirst = new(1, 4);
        math_Vector         tol = new(1, 4);
        Umin[1]         = RealFirst();
        Umin[2]         = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]         = Geom2dGcc_CurveTool.FirstParameter(OnCurv);
        Umin[4]         = 0.0;
        Umax[1]         = RealLast();
        Umax[2]         = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]         = Geom2dGcc_CurveTool.LastParameter(OnCurv);
        Umax[4]         = RealLast();
        Ufirst[1]       = Param1;
        Ufirst[2]       = Param2;
        Ufirst[3]       = ParamOn;
        tol[1]          = 1.0e-15;
        tol[2]          = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]          = Geom2dGcc_CurveTool.EpsX(OnCurv, Math.Abs(Tolerance));
        tol[4]          = Tol / 10.0;
        Point2D point1 = global::Kerf.Engine.Geometry.ElCLib.Value(Param1, L1);
        Point2D point2 = Geom2dGcc_CurveTool.Value(Cu2, Param2);
        Point2D point3 = Geom2dGcc_CurveTool.Value(OnCurv, ParamOn);
        Ufirst[4]       = (point3.Distance(point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(L1, Cu2, OnCurv, Max(Ufirst[4], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Vector2D Tan1, Tan2, Tan3;
            global::Kerf.Engine.Geometry.ElCLib.D1(Ufirst[1], L1, out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Geom2dGcc_CurveTool.D1(OnCurv, Ufirst[3], out point3, out Tan3);
            double dist1 = point3.Distance(point1);
            double dist2 = point3.Distance(point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, point3);
                Vector2D Vec2 = new(point2, point3);
                double   normevec2 = Vec2.Magnitude();
                double   angle2;
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    angle2 = Vec2.Angle(Tan2);
                }
                else
                {
                    angle2 = 0.0;
                }
                double pscal = point3.XY().Dot(new Xy(-L1.Direction().Y(), L1.Direction().X()));
                if (Qualified1.IsUnqualified() || (Qualified1.IsOutside() && pscal <= 0.0)
                        || (Qualified1.IsEnclosed() && pscal >= 0.0))
                {
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && angle2 <= 0.0)
                            || (Qualified2.IsOutside() && angle2 >= 0) || (Qualified2.IsEnclosed() && angle2 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pnttg1sol  = point1;
                        pararg1    = Ufirst[1];
                        par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                        pnttg2sol  = point2;
                        pararg2    = Ufirst[2];
                        par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                        pntcen     = point3;
                        parcen3    = Ufirst[3];
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d2TanOnIter(in Geom2dGcc_QCurve Qualified1, in Point2D Point2, CurveAdaptor2D OnCurv, double Param1, double ParamOn, double Tolerance)
    {
        TheSame1 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        parcen3  = 0.0;

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double              Tol = Math.Abs(Tolerance);
        Direction2D            dirx = new(Direction2D.D.X);
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        math_Vector         Umin = new(1, 3);
        math_Vector         Umax = new(1, 3);
        math_Vector         Ufirst = new(1, 3);
        math_Vector         tol = new(1, 3);
        Umin[1]         = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]         = RealFirst();
        Umin[3]         = Geom2dGcc_CurveTool.FirstParameter(OnCurv);
        Umax[1]         = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]         = RealLast();
        Umax[3]         = Geom2dGcc_CurveTool.LastParameter(OnCurv);
        Ufirst[1]       = Param1;
        Ufirst[2]       = ParamOn;
        tol[1]          = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[2]          = Geom2dGcc_CurveTool.EpsX(OnCurv, Math.Abs(Tolerance));
        tol[3]          = Tol / 10.0;
        Point2D point1 = Geom2dGcc_CurveTool.Value(Cu1, Param1);
        Point2D point3 = Geom2dGcc_CurveTool.Value(OnCurv, ParamOn);
        Ufirst[3]       = (point3.Distance(Point2) + point3.Distance(point1)) / 2.0;
        Geom2dGcc_FunctionTanCuCuOnCu Func = new(Cu1, Point2, OnCurv, Max(Ufirst[3], Tol));
        FunctionSetRoot          Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        Func.Value(Ufirst, Umin);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            //    gp_Vec2d Tan1,Tan2,Tan3;
            Vector2D Tan1, Tan3;
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[1], out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(OnCurv, Ufirst[3], out point3, out Tan3);
            double dist1 = point3.Distance(point1);
            double dist2 = point3.Distance(Point2);
            if (Math.Abs(dist1 - dist2) / 2.0 <= Tol)
            {
                cirsol             = new Circle2D(new Axis2D(point3, dirx), (dist1 + dist2) / 2.0);
                double   normetan1 = Tan1.Magnitude();
                Vector2D Vec1 = new(point1, point3);
                double   normevec1 = Vec1.Magnitude();
                double   angle1;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    angle1 = Vec1.Angle(Tan1);
                }
                else
                {
                    angle1 = 0.0;
                }
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && angle1 <= 0.0)
                        || (Qualified1.IsOutside() && angle1 >= 0.0) || (Qualified1.IsEnclosed() && angle1 <= 0.0))
                {
                    qualifier1 = Qualified1.Qualifier();
                    qualifier2 = GccEnt_noqualifier;
                    pnttg1sol  = point1;
                    pararg1    = Ufirst[1];
                    par1sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg1sol);
                    pnttg2sol  = Point2;
                    pararg2    = 0.0;
                    par2sol    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol, pnttg2sol);
                    pntcen     = point3;
                    parcen3    = Ufirst[3]; // OCCT: Ufirst(3) is the radius here, not the centre's parameter
                    WellDone   = true;
                }
            }
        }
    }

    /// <summary>True if the construction converged to a solution.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The solution (not checked: an undone construction returns the default circle).</summary>
    public Circle2D ThisSolution()
    {
        return cirsol;
    }

    /// <summary>The qualifiers of the solution relative to the two arguments.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void WhichQualifier(out GccEnt_Position Qualif1, out GccEnt_Position Qualif2)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            Qualif1 = qualifier1;
            Qualif2 = qualifier2;
        }
    }

    /// <summary>The tangency point on the first argument (the parameters are always 0, as in OCCT).</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    public void Tangency1(out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            if (!TheSame1) // OCCT: TheSame1 == 0
            {
                ParSol = 0; // OCCT: returns 0, not par1sol
                ParArg = 0; // OCCT: returns 0, not pararg1
                PntSol = pnttg1sol;
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The tangency point on the second argument (the parameters are always 0, as in OCCT).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void Tangency2(out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            ParSol = 0; // OCCT: returns 0, not par2sol
            ParArg = 0; // OCCT: returns 0, not pararg2
            PntSol = pnttg2sol;
        }
    }

    /// <summary>The centre of the solution (its parameter is always 0, as in OCCT).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void CenterOn3(out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            ParArg = 0; // OCCT: returns 0, not parcen3
            PntSol = pntcen;
        }
    }

    /// <summary>True if the solution is the first argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame1()
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }

        if (!TheSame1)
        {
            return false;
        }
        return true;
    }

    /// <summary>Always false: the second argument is never the solution.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame2()
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        return false;
    }
}
