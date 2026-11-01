// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Static access to a curve adaptor, as the blend walkers call it.</summary>
public static class BRepBlend_HCurveTool
{
    public static double FirstParameter(Adaptor3d_Curve C) => C.FirstParameter();

    public static double LastParameter(Adaptor3d_Curve C) => C.LastParameter();

    public static GeomAbs_Shape Continuity(Adaptor3d_Curve C) => C.Continuity();

    public static int NbIntervals(Adaptor3d_Curve C, GeomAbs_Shape S) => C.NbIntervals(S);

    public static void Intervals(Adaptor3d_Curve C, NCollection_Array1<double> T, GeomAbs_Shape S) =>
        C.Intervals(T, S);

    public static bool IsClosed(Adaptor3d_Curve C) => C.IsClosed();

    public static bool IsPeriodic(Adaptor3d_Curve C) => C.IsPeriodic();

    public static double Period(Adaptor3d_Curve C) => C.Period();

    public static gp_Pnt Value(Adaptor3d_Curve C, double U) => C.Value(U);

    public static void D0(Adaptor3d_Curve C, double U, out gp_Pnt P) => C.D0(U, out P);

    public static void D1(Adaptor3d_Curve C, double U, out gp_Pnt P, out gp_Vec V) => C.D1(U, out P, out V);

    public static void D2(Adaptor3d_Curve C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2) =>
        C.D2(U, out P, out V1, out V2);

    public static void D3(Adaptor3d_Curve C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2,
                          out gp_Vec V3) => C.D3(U, out P, out V1, out V2, out V3);

    public static gp_Vec DN(Adaptor3d_Curve C, double U, int N) => C.DN(U, N);

    public static double Resolution(Adaptor3d_Curve C, double R3d) => C.Resolution(R3d);

    public static GeomAbs_CurveType GetType(Adaptor3d_Curve C) => C.GetType();

    public static gp_Lin Line(Adaptor3d_Curve C) => C.Line();

    public static gp_Circ Circle(Adaptor3d_Curve C) => C.Circle();

    public static gp_Elips Ellipse(Adaptor3d_Curve C) => C.Ellipse();

    public static gp_Hypr Hyperbola(Adaptor3d_Curve C) => C.Hyperbola();

    public static gp_Parab Parabola(Adaptor3d_Curve C) => C.Parabola();

    public static Geom_BezierCurve Bezier(Adaptor3d_Curve C) => C.Bezier();

    public static Geom_BSplineCurve BSpline(Adaptor3d_Curve C) => C.BSpline();

    /// <summary>The number of samples on the curve between <paramref name="U0"/> and <paramref name="U1"/> (2 to 50).</summary>
    public static int NbSamples(Adaptor3d_Curve C, double U0, double U1)
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
            Geom_BSplineCurve BSC = C.BSpline();
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
