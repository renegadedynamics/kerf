// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Static access to a 2D curve adaptor (an arc of restriction), as the blend walkers call it.</summary>
public static class BRepBlend_HCurve2dTool
{
    public static double FirstParameter(Adaptor2d_Curve2d C) => C.FirstParameter();

    public static double LastParameter(Adaptor2d_Curve2d C) => C.LastParameter();

    public static GeomAbs_Shape Continuity(Adaptor2d_Curve2d C) => C.Continuity();

    public static int NbIntervals(Adaptor2d_Curve2d C, GeomAbs_Shape S) => C.NbIntervals(S);

    public static void Intervals(Adaptor2d_Curve2d C, NCollection_Array1<double> T, GeomAbs_Shape S) =>
        C.Intervals(T, S);

    public static bool IsClosed(Adaptor2d_Curve2d C) => C.IsClosed();

    public static bool IsPeriodic(Adaptor2d_Curve2d C) => C.IsPeriodic();

    public static double Period(Adaptor2d_Curve2d C) => C.Period();

    public static gp_Pnt2d Value(Adaptor2d_Curve2d C, double U) => C.Value(U);

    public static void D0(Adaptor2d_Curve2d C, double U, out gp_Pnt2d P) => C.D0(U, out P);

    public static void D1(Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d V) => C.D1(U, out P, out V);

    public static void D2(Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2) =>
        C.D2(U, out P, out V1, out V2);

    public static void D3(Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2,
                          out gp_Vec2d V3) => C.D3(U, out P, out V1, out V2, out V3);

    public static gp_Vec2d DN(Adaptor2d_Curve2d C, double U, int N) => C.DN(U, N);

    public static double Resolution(Adaptor2d_Curve2d C, double R3d) => C.Resolution(R3d);

    public static GeomAbs_CurveType GetType(Adaptor2d_Curve2d C) => C.GetType();

    public static gp_Lin2d Line(Adaptor2d_Curve2d C) => C.Line();

    public static gp_Circ2d Circle(Adaptor2d_Curve2d C) => C.Circle();

    public static gp_Elips2d Ellipse(Adaptor2d_Curve2d C) => C.Ellipse();

    public static gp_Hypr2d Hyperbola(Adaptor2d_Curve2d C) => C.Hyperbola();

    public static gp_Parab2d Parabola(Adaptor2d_Curve2d C) => C.Parabola();

    public static Geom2d_BezierCurve Bezier(Adaptor2d_Curve2d C) => C.Bezier();

    public static Geom2d_BSplineCurve BSpline(Adaptor2d_Curve2d C) => C.BSpline();

    /// <summary>The number of samples on the curve between <paramref name="U0"/> and <paramref name="U1"/> (2 to 50).</summary>
    public static int NbSamples(Adaptor2d_Curve2d C, double U0, double U1)
    {
        GeomAbs_CurveType typC = C.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == GeomAbs_Line)
        {
            nbs = 2;
        }
        else if (typC == GeomAbs_BezierCurve)
        {
            nbs = 3 + C.Bezier().NbPoles();
        }
        else if (typC == GeomAbs_BSplineCurve)
        {
            Geom2d_BSplineCurve BSC = C.BSpline();
            nbs = BSC.NbKnots();
            nbs *= BSC.Degree();
            nbs *= BSC.LastParameter() - BSC.FirstParameter();
            nbs /= U1 - U0;
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        if (nbs > 50)
        {
            nbs = 50;
        }
        return ((int)nbs);
    }
}
