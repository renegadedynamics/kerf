// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Static access to a curve adaptor, as the blend walkers call it.</summary>
public static class BRepBlend_HCurveTool
{
    public static double FirstParameter(Adaptor3d_Curve C) => C.FirstParameter();

    public static double LastParameter(Adaptor3d_Curve C) => C.LastParameter();

    public static Continuity Continuity(Adaptor3d_Curve C) => C.Continuity();

    public static int NbIntervals(Adaptor3d_Curve C, Continuity S) => C.NbIntervals(S);

    public static void Intervals(Adaptor3d_Curve C, NCollection_Array1<double> T, Continuity S) =>
        C.Intervals(T, S);

    public static bool IsClosed(Adaptor3d_Curve C) => C.IsClosed();

    public static bool IsPeriodic(Adaptor3d_Curve C) => C.IsPeriodic();

    public static double Period(Adaptor3d_Curve C) => C.Period();

    public static Point3D Value(Adaptor3d_Curve C, double U) => C.Value(U);

    public static void D0(Adaptor3d_Curve C, double U, out Point3D P) => C.D0(U, out P);

    public static void D1(Adaptor3d_Curve C, double U, out Point3D P, out Vector3D V) => C.D1(U, out P, out V);

    public static void D2(Adaptor3d_Curve C, double U, out Point3D P, out Vector3D V1, out Vector3D V2) =>
        C.D2(U, out P, out V1, out V2);

    public static void D3(Adaptor3d_Curve C, double U, out Point3D P, out Vector3D V1, out Vector3D V2,
                          out Vector3D V3) => C.D3(U, out P, out V1, out V2, out V3);

    public static Vector3D DN(Adaptor3d_Curve C, double U, int N) => C.DN(U, N);

    public static double Resolution(Adaptor3d_Curve C, double R3d) => C.Resolution(R3d);

    public static CurveKind GetType(Adaptor3d_Curve C) => C.GetType();

    public static Line3D Line(Adaptor3d_Curve C) => C.Line();

    public static Circle3D Circle(Adaptor3d_Curve C) => C.Circle();

    public static Ellipse3D Ellipse(Adaptor3d_Curve C) => C.Ellipse();

    public static Hyperbola3D Hyperbola(Adaptor3d_Curve C) => C.Hyperbola();

    public static Parabola3D Parabola(Adaptor3d_Curve C) => C.Parabola();

    public static Geom_BezierCurve Bezier(Adaptor3d_Curve C) => C.Bezier();

    public static Geom_BSplineCurve BSpline(Adaptor3d_Curve C) => C.BSpline();

    /// <summary>The number of samples on the curve between <paramref name="U0"/> and <paramref name="U1"/> (2 to 50).</summary>
    public static int NbSamples(Adaptor3d_Curve C, double U0, double U1)
    {
        CurveKind typC = C.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + C.Bezier().NbPoles();
        }
        else if (typC == BSplineCurve)
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
