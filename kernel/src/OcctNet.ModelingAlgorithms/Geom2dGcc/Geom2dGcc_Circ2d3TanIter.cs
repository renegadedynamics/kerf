// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3TanIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3TanIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// A circle tangent to three arguments (qualified curves, circles or lines, or points), at least one of them a curve,
/// found iteratively from initial parameters on the arguments: math_FunctionSetRoot on Geom2dGcc_FunctionTanCuCuCu,
/// then the circle through the three points found (GccAna_Circ2d3Tan), kept if it is tangent and its qualifiers hold.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); one solution at most. A point argument is a circle
/// of radius 0 for the function.
/// </remarks>
public sealed class Geom2dGcc_Circ2d3TanIter
{
    private bool WellDone;
    private Circle2D cirsol = new();
    private GccEnt_Position qualifier1;
    private GccEnt_Position qualifier2;
    private GccEnt_Position qualifier3;
    private bool TheSame1;
    // OCCT's TheSame2 is written but never read (IsTheSame2 reads TheSame3): left out.
    private bool TheSame3;
    private Point2D pnttg1sol = new();
    private Point2D pnttg2sol = new();
    private Point2D pnttg3sol = new();
    private double par1sol;
    private double par2sol;
    private double par3sol;
    private double pararg1;
    private double pararg2;
    private double pararg3;


    public Geom2dGcc_Circ2d3TanIter(in Geom2dGcc_QCurve Qualified1, in Geom2dGcc_QCurve Qualified2, in Geom2dGcc_QCurve Qualified3, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified())
                || !(Qualified3.IsEnclosed() || Qualified3.IsEnclosing() || Qualified3.IsOutside()
                          || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        CurveAdaptor2D         Cu1 = Qualified1.Qualified();
        CurveAdaptor2D         Cu2 = Qualified2.Qualified();
        CurveAdaptor2D         Cu3 = Qualified3.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(Cu1, Cu2, Cu3);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu3);
        Umax[1]   = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu3);
        Ufirst[1] = Param1;
        Ufirst[2] = Param2;
        Ufirst[3] = Param3;
        tol[1]    = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[2]    = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu3, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D point1 = new(), point2 = new(), point3 = new();
            Vector2D Tan1 = new(), Tan2 = new(), Tan3 = new();
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[1], out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Geom2dGcc_CurveTool.D1(Cu3, Ufirst[3], out point3, out Tan3);
            GccAna_Circ2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone())
            {
                cirsol             = circ.ThisSolution(1);
                Point2D centre    = cirsol.Location();
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double Angle1 = Vec1.Angle(Tan1);
                    if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Angle1 <= 0.0)
                            || (Qualified1.IsOutside() && Angle1 >= 0.0)
                            || (Qualified1.IsEnclosed() && Angle1 <= 0.0))
                    {
                        Angle1 = Vec2.Angle(Tan2);
                        if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && Angle1 <= 0.0)
                                || (Qualified2.IsOutside() && Angle1 >= 0)
                                || (Qualified2.IsEnclosed() && Angle1 <= 0.0))
                        {
                            Angle1 = Vec3.Angle(Tan3);
                            if (Qualified3.IsUnqualified() || (Qualified3.IsEnclosing() && Angle1 <= 0.0)
                                    || (Qualified3.IsOutside() && Angle1 >= 0)
                                    || (Qualified3.IsEnclosed() && Angle1 <= 0.0))
                            {
                                qualifier1 = Qualified1.Qualifier();
                                qualifier2 = Qualified2.Qualifier();
                                qualifier3 = Qualified3.Qualifier();
                                pararg1    = Ufirst[1];
                                par1sol    = 0.0;
                                pnttg1sol  = point1;
                                pararg2    = Ufirst[2];
                                pnttg2sol  = point2;
                                par2sol    = pnttg2sol.Distance(pnttg1sol);
                                pnttg3sol  = point3;
                                pararg3    = Ufirst[3];
                                par3sol    = pnttg3sol.Distance(pnttg1sol);
                                WellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in GccEnt_QualifiedCirc Qualified1, in Geom2dGcc_QCurve Qualified2, in Geom2dGcc_QCurve Qualified3, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified())
                || !(Qualified3.IsEnclosed() || Qualified3.IsEnclosing() || Qualified3.IsOutside()
                          || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D                   C1  = Qualified1.Qualified();
        CurveAdaptor2D         Cu2 = Qualified2.Qualified();
        CurveAdaptor2D         Cu3 = Qualified3.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(C1, Cu2, Cu3);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = 0.0;
        Umin[2]   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu3);
        Umax[1]   = 2 * M_PI;
        Umax[2]   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu3);
        Ufirst[1] = Param1;
        Ufirst[2] = Param2;
        Ufirst[3] = Param3;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu3, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D centre1 = C1.Location();
            double   R1 = C1.Radius();
            Point2D point1 = new(centre1.XY() + R1 * new Xy(Math.Cos(Ufirst[1]), Math.Sin(Ufirst[1])));
            Vector2D Tan1 = new(new Xy(-Math.Sin(Ufirst[1]), Math.Cos(Ufirst[1])));
            Point2D point2 = new(), point3 = new();
            //     gp_Vec2d Tan2,Tan3,Nor2,Nor3;
            Vector2D Tan2 = new(), Tan3 = new();
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Geom2dGcc_CurveTool.D1(Cu3, Ufirst[3], out point3, out Tan3);
            GccAna_Circ2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                double   dist      = centre1.Distance(centre);
                double   Rsol      = cirsol.Radius();
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R1 && dist <= Rsol)
                            || (Qualified1.IsOutside() && dist >= Rsol)
                            || (Qualified1.IsEnclosed() && Rsol <= R1 && dist <= Rsol))
                    {
                        double Angle1 = Vec2.Angle(Tan2);
                        if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && Angle1 <= 0.0)
                                || (Qualified2.IsOutside() && Angle1 >= 0)
                                || (Qualified2.IsEnclosed() && Angle1 <= 0.0))
                        {
                            Angle1 = Vec3.Angle(Tan3);
                            if (Qualified3.IsUnqualified() || (Qualified3.IsEnclosing() && Angle1 <= 0.0)
                                    || (Qualified3.IsOutside() && Angle1 >= 0)
                                    || (Qualified3.IsEnclosed() && Angle1 <= 0.0))
                            {
                                qualifier1 = Qualified1.Qualifier();
                                qualifier2 = Qualified2.Qualifier();
                                qualifier3 = Qualified3.Qualifier();
                                pararg1    = Ufirst[1];
                                par1sol    = 0.0;
                                pnttg1sol  = point1;
                                pararg2    = Ufirst[2];
                                pnttg2sol  = point2;
                                par2sol    = 0.0;
                                pararg3    = Ufirst[3];
                                pnttg3sol  = point3;
                                par3sol    = 0.0;
                                WellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedCirc Qualified2, in Geom2dGcc_QCurve Qualified3, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified())
                || !(Qualified3.IsEnclosed() || Qualified3.IsEnclosing() || Qualified3.IsOutside()
                          || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D                   C1  = Qualified1.Qualified();
        Circle2D                   C2  = Qualified2.Qualified();
        CurveAdaptor2D         Cu3 = Qualified3.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(C1, C2, Cu3);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = 0.0;
        Umin[2]   = 0.0;
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu3);
        Umax[1]   = 2 * M_PI;
        Umax[2]   = 2 * M_PI;
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu3);
        Ufirst[1] = Param1;
        Ufirst[2] = Param2;
        Ufirst[3] = Param3;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 2.0e-15 * M_PI;
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu3, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D centre1 = C1.Location();
            double   R1 = C1.Radius();
            Point2D point1 = new(centre1.XY() + R1 * new Xy(Math.Cos(Ufirst[1]), Math.Sin(Ufirst[1])));
            Vector2D Tan1 = new(new Xy(-Math.Sin(Ufirst[1]), Math.Cos(Ufirst[1])));
            Point2D centre2 = C2.Location();
            double   R2 = C2.Radius();
            Point2D point2 = new(centre2.XY() + R2 * new Xy(Math.Cos(Ufirst[2]), Math.Sin(Ufirst[2])));
            Vector2D Tan2 = new(new Xy(-Math.Sin(Ufirst[2]), Math.Cos(Ufirst[2])));
            Point2D point3 = new();
            Vector2D Tan3 = new();
            Geom2dGcc_CurveTool.D1(Cu3, Ufirst[3], out point3, out Tan3);
            GccAna_Circ2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                double   dist      = centre1.Distance(centre);
                double   Rsol      = cirsol.Radius();
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R1 && dist <= Rsol)
                            || (Qualified1.IsOutside() && dist >= Rsol)
                            || (Qualified1.IsEnclosed() && Rsol <= R1 && dist <= Rsol))
                    {
                        dist = centre2.Distance(centre); // OCCT: the second circle is tested with Qualified1, not Qualified2
                        if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R2 && dist <= Rsol)
                                || (Qualified1.IsOutside() && dist >= Rsol)
                                || (Qualified1.IsEnclosed() && Rsol <= R2 && dist <= Rsol))
                        {
                            Vector2D Vec = new(point3, centre);
                            double   Angle1 = Vec.Angle(Tan3);
                            if (Qualified3.IsUnqualified() || (Qualified3.IsEnclosing() && Angle1 <= 0.0)
                                    || (Qualified3.IsOutside() && Angle1 >= 0)
                                    || (Qualified3.IsEnclosed() && Angle1 <= 0.0))
                            {
                                qualifier1 = Qualified1.Qualifier();
                                qualifier2 = Qualified2.Qualifier();
                                qualifier3 = Qualified3.Qualifier();
                                pararg1    = Ufirst[1];
                                par1sol    = 0.0;
                                pnttg1sol  = point1;
                                pararg2    = Ufirst[2];
                                pnttg2sol  = point2;
                                par2sol    = 0.0;
                                pararg3    = Ufirst[3];
                                pnttg3sol  = point3;
                                par3sol    = 0.0;
                                WellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in GccEnt_QualifiedLin Qualified1, in Geom2dGcc_QCurve Qualified2, in Geom2dGcc_QCurve Qualified3, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified())
                || !(Qualified3.IsEnclosed() || Qualified3.IsEnclosing() || Qualified3.IsOutside()
                          || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Line2D                    L1  = Qualified1.Qualified();
        CurveAdaptor2D         Cu2 = Qualified2.Qualified();
        CurveAdaptor2D         Cu3 = Qualified3.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(L1, Cu2, Cu3);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = RealFirst();
        Umin[2]   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu3);
        Umax[1]   = RealLast();
        Umax[2]   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu3);
        Ufirst[1] = Param1;
        Ufirst[2] = Param2;
        Ufirst[3] = Param3;
        tol[1]    = 1.0e-15;
        tol[2]    = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu3, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D centre1 = L1.Location();
            Point2D point1 = new(centre1.XY() + Ufirst[1] * L1.Direction().XY());
            Point2D point2 = new(), point3 = new();
            Vector2D Tan2 = new(), Tan3 = new();
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2);
            Geom2dGcc_CurveTool.D1(Cu3, Ufirst[3], out point3, out Tan3);
            GccAna_Circ2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();

                // creation vaariables intermediaires pour WNT
                Xy  dummy1 = centre.XY() - L1.Location().XY();
                Xy  dummy2 = new(-L1.Direction().Y(), L1.Direction().X());
                double pscal = dummy1.Dot(dummy2);

                Vector2D Tan1 = new(L1.Direction().XY());
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (Qualified1.IsUnqualified() || (Qualified1.IsOutside() && pscal <= 0.0)
                            || (Qualified1.IsEnclosed() && pscal >= 0.0))
                    {
                        Vector2D Vec = new(point2, centre);
                        double   Angle1 = Vec.Angle(Tan2);
                        if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && Angle1 <= 0.0)
                                || (Qualified2.IsOutside() && Angle1 >= 0)
                                || (Qualified2.IsEnclosed() && Angle1 <= 0.0))
                        {
                            Vec    = new Vector2D(point3, centre);
                            Angle1 = Vec.Angle(Tan3);
                            if (Qualified3.IsUnqualified() || (Qualified3.IsEnclosing() && Angle1 <= 0.0)
                                    || (Qualified3.IsOutside() && Angle1 >= 0)
                                    || (Qualified3.IsEnclosed() && Angle1 <= 0.0))
                            {
                                qualifier1 = Qualified1.Qualifier();
                                qualifier2 = Qualified2.Qualifier();
                                qualifier3 = Qualified3.Qualifier();
                                pararg1    = Ufirst[1];
                                par1sol    = 0.0;
                                pnttg1sol  = point1;
                                pararg2    = Ufirst[2];
                                pnttg2sol  = point2;
                                par2sol    = 0.0;
                                pararg3    = Ufirst[3];
                                pnttg3sol  = point3;
                                par3sol    = 0.0;
                                WellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in GccEnt_QualifiedLin Qualified1, in GccEnt_QualifiedLin Qualified2, in Geom2dGcc_QCurve Qualified3, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified())
                || !(Qualified3.IsEnclosed() || Qualified3.IsEnclosing() || Qualified3.IsOutside()
                          || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Line2D                    L1  = Qualified1.Qualified();
        Line2D                    L2  = Qualified2.Qualified();
        CurveAdaptor2D         Cu3 = Qualified3.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(L1, L2, Cu3);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = RealFirst();
        Umin[2]   = RealFirst();
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu3);
        Umax[1]   = RealLast();
        Umax[2]   = RealLast();
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu3);
        Ufirst[1] = Param1;
        Ufirst[2] = Param2;
        Ufirst[3] = Param3;
        tol[1]    = 1.0e-15;
        tol[2]    = 1.0e-15;
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu3, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D centre1 = L1.Location();
            Point2D point1 = new(centre1.XY() + Ufirst[1] * L1.Direction().XY());
            Point2D centre2 = L2.Location();
            Point2D point2 = new(centre2.XY() + Ufirst[2] * L2.Direction().XY());
            Point2D point3 = new();
            Vector2D Tan3 = new();
            Geom2dGcc_CurveTool.D1(Cu3, Ufirst[3], out point3, out Tan3);
            GccAna_Circ2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                double   pscal = centre.XY().Dot(new Xy(-L1.Direction().Y(), L1.Direction().X())); // OCCT: the side of L1 from the origin, not from a point of L1 (the next test likewise for L2)
                if (Qualified1.IsUnqualified() || (Qualified1.IsOutside() && pscal <= 0.0)
                        || (Qualified1.IsEnclosed() && pscal >= 0.0))
                {
                    Vector2D Tan1 = new(L1.Direction().XY());
                    Vector2D Tan2 = new(L2.Direction().XY());
                    double   normetan1 = Tan1.Magnitude();
                    double   normetan2 = Tan2.Magnitude();
                    double   normetan3 = Tan3.Magnitude();
                    Vector2D Vec1 = new(point1, centre);
                    Vector2D Vec2 = new(point2, centre);
                    Vector2D Vec3 = new(point3, centre);
                    double   normevec1 = Vec1.Magnitude();
                    double   normevec2 = Vec2.Magnitude();
                    double   normevec3 = Vec3.Magnitude();
                    double   dot1, dot2, dot3;
                    if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                    {
                        dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                    }
                    else
                    {
                        dot1 = 0.0;
                    }
                    if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                    {
                        dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                    }
                    else
                    {
                        dot2 = 0.0;
                    }
                    if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                    {
                        dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                    }
                    else
                    {
                        dot3 = 0.0;
                    }
                    Tol = 1.0e-12;
                    if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                    {
                        if (Qualified2.IsUnqualified() || (Qualified2.IsOutside() && pscal <= 0.0)
                                || (Qualified2.IsEnclosed() && pscal >= 0.0))
                        {
                            double Angle1 = Vec3.Angle(Tan3);
                            if (Qualified3.IsUnqualified() || (Qualified3.IsEnclosing() && Angle1 <= 0.0)
                                    || (Qualified3.IsOutside() && Angle1 >= 0)
                                    || (Qualified3.IsEnclosed() && Angle1 <= 0.0))
                            {
                                qualifier1 = Qualified1.Qualifier();
                                qualifier2 = Qualified2.Qualifier();
                                qualifier3 = Qualified3.Qualifier();
                                pararg1    = Ufirst[1];
                                par1sol    = 0.0;
                                pnttg1sol  = point1;
                                pararg2    = Ufirst[2];
                                pnttg2sol  = point2;
                                par2sol    = 0.0;
                                pararg3    = Ufirst[3];
                                pnttg3sol  = point3;
                                par3sol    = 0.0;
                                WellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in Geom2dGcc_QCurve Qualified1, in Geom2dGcc_QCurve Qualified2, in Point2D Point3, double Param1, double Param2, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D                   C1 = new(new Axis2D(Point3, new Direction2D(Direction2D.D.X)), 0.0);
        CurveAdaptor2D         Cu1 = Qualified1.Qualified();
        CurveAdaptor2D         Cu2 = Qualified2.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(C1, Cu1, Cu2);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = 0.0;
        Umin[2]   = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umax[1]   = 2 * M_PI;
        Umax[2]   = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Ufirst[1] = M_PI;
        Ufirst[2] = Param1;
        Ufirst[3] = Param2;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D point1 = new(), point2 = new();
            //     gp_Vec2d Tan1,Tan2,Nor1,Nor2;
            Vector2D Tan1 = new(), Tan2 = new();
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[2], out point1, out Tan1);
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[3], out point2, out Tan2);
            GccAna_Circ2d3Tan circ = new(Point3, point1, point2, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                Vector2D Tan3 = new(-Math.Sin(Ufirst[1]), Math.Cos(Ufirst[1]));
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(Point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double Angle1 = Vec1.Angle(Tan1);
                    if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Angle1 <= 0.0)
                            || (Qualified1.IsOutside() && Angle1 >= 0) || (Qualified1.IsEnclosed() && Angle1 <= 0.0))
                    {
                        Angle1 = Vec2.Angle(Tan2); // OCCT: the second test below checks Qualified1 again, not Qualified2
                        if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Angle1 <= 0.0)
                                || (Qualified1.IsOutside() && Angle1 >= 0)
                                || (Qualified1.IsEnclosed() && Angle1 <= 0.0))
                        {
                            qualifier1 = Qualified1.Qualifier();
                            qualifier2 = Qualified2.Qualifier();
                            qualifier3 = GccEnt_noqualifier;
                            pararg1    = Ufirst[2];
                            par1sol    = 0.0;
                            pnttg1sol  = point1;
                            pararg2    = Ufirst[3];
                            pnttg2sol  = point2;
                            par2sol    = 0.0;
                            pararg3    = 0.0;
                            pnttg3sol  = Point3;
                            par3sol    = 0.0;
                            WellDone   = true;
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in Geom2dGcc_QCurve Qualified1, in Point2D Point2, in Point2D Point3, double Param1, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D                   C1 = new(new Axis2D(Point2, new Direction2D(Direction2D.D.X)), 0.0);
        Circle2D                   C2 = new(new Axis2D(Point3, new Direction2D(Direction2D.D.X)), 0.0);
        CurveAdaptor2D         Cu1 = Qualified1.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(C1, C2, Cu1);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = 0.0;
        Umin[2]   = 0.0;
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umax[1]   = 2 * M_PI;
        Umax[2]   = 2 * M_PI;
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Ufirst[1] = M_PI;
        Ufirst[2] = M_PI;
        Ufirst[3] = Param1;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 2.0e-15 * M_PI;
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D point3 = new();
            //     gp_Vec2d Tan3,Nor3;
            Vector2D Tan3 = new();
            Geom2dGcc_CurveTool.D1(Cu1, Ufirst[3], out point3, out Tan3);
            GccAna_Circ2d3Tan circ = new(Point2, Point3, point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                Vector2D Tan2 = new(-Math.Sin(Ufirst[2]), Math.Cos(Ufirst[2]));
                Vector2D Tan1 = new(-Math.Sin(Ufirst[1]), Math.Cos(Ufirst[1]));
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(Point2, centre);
                Vector2D Vec2 = new(Point3, centre);
                Vector2D Vec3 = new(point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double Angle1 = Vec1.Angle(Tan1);
                    if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Angle1 <= 0.0)
                            || (Qualified1.IsOutside() && Angle1 >= 0) || (Qualified1.IsEnclosed() && Angle1 <= 0.0))
                    {
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = GccEnt_noqualifier;
                        qualifier3 = GccEnt_noqualifier;
                        pararg1    = Ufirst[3];
                        par1sol    = 0.0;
                        pnttg1sol  = point3;
                        pararg2    = 0.0;
                        pnttg2sol  = Point2;
                        par2sol    = 0.0;
                        pararg3    = 0.0;
                        pnttg3sol  = Point3;
                        par3sol    = 0.0;
                        WellDone   = true;
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in GccEnt_QualifiedLin Qualified1, in Geom2dGcc_QCurve Qualified2, in Point2D Point3, double Param1, double Param2, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Line2D                    L1  = Qualified1.Qualified();
        CurveAdaptor2D         Cu2 = Qualified2.Qualified();
        Circle2D                   C3 = new(new Axis2D(Point3, new Direction2D(Direction2D.D.X)), 0.0);
        Geom2dGcc_FunctionTanCuCuCu Func = new(C3, L1, Cu2);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[2]   = RealFirst();
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[1]   = 0.0;
        Umax[2]   = RealLast();
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[1]   = 2 * M_PI;
        Ufirst[2] = Param1;
        Ufirst[3] = Param2;
        Ufirst[1] = M_PI;
        tol[1]    = 2.0e-15;
        tol[2]    = 1.0e-15;
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D centre1 = L1.Location();
            Point2D point1 = new(centre1.XY() + Ufirst[2] * L1.Direction().XY());
            Point2D point2 = new();
            Vector2D Tan2 = new();
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2); // OCCT: Cu2's parameter is Ufirst(3); Ufirst(2) is the one on L1
            GccAna_Circ2d3Tan circ = new(point1, point2, Point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                double   pscal = centre.XY().Dot(new Xy(-L1.Direction().Y(), L1.Direction().X())); // OCCT: the side of L1 from the origin, not from a point of L1
                Vector2D Tan1 = new(L1.Direction().XY());
                Vector2D Tan3 = new(-Math.Sin(Ufirst[1]), Math.Cos(Ufirst[1]));
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(Point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    if (Qualified1.IsUnqualified() || (Qualified1.IsOutside() && pscal <= 0.0)
                            || (Qualified1.IsEnclosed() && pscal >= 0.0))
                    {
                        double Angle1 = Vec2.Angle(Tan2);
                        if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && Angle1 <= 0.0)
                                || (Qualified2.IsOutside() && Angle1 >= 0)
                                || (Qualified2.IsEnclosed() && Angle1 <= 0.0))
                        {
                            qualifier1 = Qualified1.Qualifier();
                            qualifier2 = Qualified2.Qualifier();
                            qualifier3 = GccEnt_noqualifier;
                            pararg1    = Ufirst[2];
                            par1sol    = 0.0;
                            pnttg1sol  = point1;
                            pararg2    = Ufirst[3];
                            pnttg2sol  = point2;
                            par2sol    = 0.0;
                            pararg3    = 0.0;
                            pnttg3sol  = Point3;
                            par3sol    = 0.0;
                            WellDone   = true;
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedLin Qualified2, in Geom2dGcc_QCurve Qualified3, double Param1, double Param2, double Param3, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified())
                || !(Qualified3.IsEnclosed() || Qualified3.IsEnclosing() || Qualified3.IsOutside()
                          || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D                   C1  = Qualified1.Qualified();
        Line2D                    L2  = Qualified2.Qualified();
        CurveAdaptor2D         Cu3 = Qualified3.Qualified();
        Geom2dGcc_FunctionTanCuCuCu Func = new(C1, L2, Cu3);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = 0.0;
        Umin[2]   = RealFirst();
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu3);
        Umax[1]   = 2 * M_PI;
        Umax[2]   = RealLast();
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu3);
        Ufirst[1] = Param1;
        Ufirst[2] = Param2;
        Ufirst[3] = Param3;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 1.0e-15;
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu3, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Func.Value(Ufirst, Umin);
            Root.Root(Ufirst);
            Point2D centre1 = C1.Location();
            double   R1 = C1.Radius();
            Point2D point1 = new(centre1.XY() + R1 * new Xy(Math.Cos(Ufirst[1]), Math.Sin(Ufirst[1])));
            Point2D centre2 = L2.Location();
            Point2D point2 = new(centre2.XY() + Ufirst[2] * L2.Direction().XY());
            Point2D point3 = new();
            Vector2D Tan3 = new();
            Geom2dGcc_CurveTool.D1(Cu3, Ufirst[3], out point3, out Tan3);
            GccAna_Circ2d3Tan circ = new(point1, point2, point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                Vector2D Tan1 = new(-Math.Sin(Ufirst[1]), Math.Cos(Ufirst[1]));
                Vector2D Tan2 = new(L2.Direction().XY());
                double   normetan1 = Tan1.Magnitude();
                double   normetan2 = Tan2.Magnitude();
                double   normetan3 = Tan3.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution() && normetan1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1 * normetan1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution() && normetan3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3 * normetan3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double dist = centre1.Distance(centre);
                    double Rsol = cirsol.Radius();
                    if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R1 && dist <= Rsol)
                            || (Qualified1.IsOutside() && dist >= Rsol)
                            || (Qualified1.IsEnclosed() && Rsol <= R1 && dist <= Rsol))
                    {
                        double pscal = centre.XY().Dot(new Xy(-L2.Direction().Y(), L2.Direction().X())); // OCCT: the side of L2 from the origin, not from a point of L2
                        if (Qualified2.IsUnqualified() || (Qualified2.IsOutside() && pscal <= 0.0)
                                || (Qualified2.IsEnclosed() && pscal >= 0.0))
                        {
                            double Angle1 = Vec3.Angle(Tan3);
                            if (Qualified3.IsUnqualified() || (Qualified3.IsEnclosing() && Angle1 <= 0.0)
                                    || (Qualified3.IsOutside() && Angle1 >= 0)
                                    || (Qualified3.IsEnclosed() && Angle1 <= 0.0))
                            {
                                qualifier1 = Qualified1.Qualifier();
                                qualifier2 = Qualified2.Qualifier();
                                qualifier3 = Qualified3.Qualifier();
                                pararg1    = Ufirst[1];
                                par1sol    = 0.0;
                                pnttg1sol  = point1;
                                pararg2    = Ufirst[2];
                                pnttg2sol  = point2;
                                par2sol    = 0.0;
                                pararg3    = Ufirst[3];
                                pnttg3sol  = point3;
                                par3sol    = 0.0;
                                WellDone   = true;
                            }
                        }
                    }
                }
            }
        }
    }

    public Geom2dGcc_Circ2d3TanIter(in GccEnt_QualifiedCirc Qualified1, in Geom2dGcc_QCurve Qualified2, in Point2D Point3, double Param1, double Param2, double Tolerance)
    {
        TheSame1 = false;
        TheSame3 = false;
        par1sol  = 0.0;
        par2sol  = 0.0;
        par3sol  = 0.0;
        pararg1  = 0.0;
        pararg2  = 0.0;
        pararg3  = 0.0;

        double Tol = Math.Abs(Tolerance);
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        qualifier3 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D                   C1  = Qualified1.Qualified();
        CurveAdaptor2D         Cu2 = Qualified2.Qualified();
        Circle2D                   C3 = new(new Axis2D(Point3, new Direction2D(Direction2D.D.X)), 0.0);
        Geom2dGcc_FunctionTanCuCuCu Func = new(C1, C3, Cu2);
        math_Vector                 Umin = new(1, 3);
        math_Vector                 Umax = new(1, 3);
        math_Vector                 Ufirst = new(1, 3);
        math_Vector                 tol = new(1, 3);
        Umin[1]   = 0.0;
        Umin[3]   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umin[2]   = 0.0;
        Umax[1]   = 2 * M_PI;
        Umax[3]   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Umax[2]   = 2 * M_PI;
        Ufirst[1] = Param1;
        Ufirst[2] = M_PI;
        Ufirst[3] = Param2;
        tol[1]    = 2.0e-15 * M_PI;
        tol[2]    = 2.0e-15 * M_PI;
        tol[3]    = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolerance));
        FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone)
        {
            Root.Root(Ufirst);
            Func.Value(Ufirst, Umin);
            Point2D centre1 = C1.Location();
            double   R1 = C1.Radius();
            Point2D point1 = new(centre1.XY() + R1 * new Xy(Math.Cos(Ufirst[1]), Math.Sin(Ufirst[1])));
            Point2D point2 = new();
            //     gp_Vec2d Tan2,Nor2;
            Vector2D Tan2 = new();
            Geom2dGcc_CurveTool.D1(Cu2, Ufirst[2], out point2, out Tan2); // OCCT: Cu2's parameter is Ufirst(3); Ufirst(2) is the one on the point's circle
            GccAna_Circ2d3Tan circ = new(point1, point2, Point3, Tol);
            if (circ.IsDone())
            {
                cirsol = circ.ThisSolution(1);
                Point2D centre = cirsol.Location();
                Vector2D Tan1 = new(-Math.Sin(Ufirst[1]), Math.Cos(Ufirst[1]));
                Vector2D Tan3 = new(-Math.Sin(Ufirst[3]), Math.Cos(Ufirst[3])); // OCCT: the point's circle parameter is Ufirst(2) (likewise above)
                double   normetan2 = Tan2.Magnitude();
                Vector2D Vec1 = new(point1, centre);
                Vector2D Vec2 = new(point2, centre);
                Vector2D Vec3 = new(Point3, centre);
                double   normevec1 = Vec1.Magnitude();
                double   normevec2 = Vec2.Magnitude();
                double   normevec3 = Vec3.Magnitude();
                double   dot1, dot2, dot3;
                if (normevec1 >= Constants.Resolution())
                {
                    dot1 = Vec1.Dot(Tan1) / (normevec1);
                }
                else
                {
                    dot1 = 0.0;
                }
                if (normevec2 >= Constants.Resolution() && normetan2 >= Constants.Resolution())
                {
                    dot2 = Vec2.Dot(Tan2) / (normevec2 * normetan2);
                }
                else
                {
                    dot2 = 0.0;
                }
                if (normevec3 >= Constants.Resolution())
                {
                    dot3 = Vec3.Dot(Tan3) / (normevec3);
                }
                else
                {
                    dot3 = 0.0;
                }
                Tol = 1.0e-12;
                if (dot1 <= Tol && dot2 <= Tol && dot3 <= Tol)
                {
                    double dist = centre1.Distance(centre);
                    double Rsol = cirsol.Radius();
                    if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Rsol >= R1 && dist <= Rsol)
                            || (Qualified1.IsOutside() && dist >= Rsol)
                            || (Qualified1.IsEnclosed() && Rsol <= R1 && dist <= Rsol))
                    {
                        double Angle1 = Vec2.Angle(Tan2);
                        if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && Angle1 <= 0.0)
                                || (Qualified2.IsOutside() && Angle1 >= 0)
                                || (Qualified2.IsEnclosed() && Angle1 <= 0.0))
                        {
                            qualifier1 = Qualified1.Qualifier();
                            qualifier2 = Qualified2.Qualifier();
                            qualifier3 = GccEnt_noqualifier;
                            pararg1    = Ufirst[1];
                            par1sol    = 0.0;
                            pnttg1sol  = point1;
                            pararg2    = Ufirst[2];
                            pnttg2sol  = point2;
                            par2sol    = 0.0;
                            pararg3    = 0.0;
                            pnttg3sol  = Point3;
                            par3sol    = 0.0;
                            WellDone   = true;
                        }
                    }
                }
            }
        }
    }

    /// <summary>True if the construction found a solution.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The solution (not checked: an unset circle if the construction failed).</summary>
    public Circle2D ThisSolution()
    {
        return cirsol;
    }

    /// <summary>The qualifiers of the arguments for the solution.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void WhichQualifier(out GccEnt_Position Qualif1, out GccEnt_Position Qualif2, out GccEnt_Position Qualif3)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            Qualif1 = qualifier1;
            Qualif2 = qualifier2;
            Qualif3 = qualifier3;
        }
    }

    /// <summary>The tangency point on the first argument: its parameter on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    public void Tangency1(out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            if (!TheSame1)
            {
                ParSol = par1sol;
                ParArg = pararg1;
                PntSol = pnttg1sol;
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The tangency point on the second argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void Tangency2(out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            ParSol = par2sol;
            ParArg = pararg2;
            PntSol = pnttg2sol;
        }
    }

    /// <summary>The tangency point on the third argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public void Tangency3(out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            ParSol = par3sol;
            ParArg = pararg3;
            PntSol = pnttg3sol;
        }
    }

    /// <summary>True if the solution is the first argument (never, as OCCT sets it).</summary>
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

    /// <summary>True if the solution is the second argument (OCCT answers for the third).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame2()
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }

        if (!TheSame3) // OCCT: tests TheSame3 (always false, so no effect)
        {
            return false;
        }

        return true;
    }

    /// <summary>
    /// True if the solution is the third argument. OCCT answers true whenever the construction is done, so
    /// Geom2dGcc_Circ2d3Tan's Tangency3 throws for every iterative solution.
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public bool IsTheSame3()
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }

        return true; // OCCT: always true (occt-issues.md)
    }
}
