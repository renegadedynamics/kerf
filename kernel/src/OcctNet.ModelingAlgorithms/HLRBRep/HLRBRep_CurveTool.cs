// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CurveTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CurveTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CurveTool.cxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TypeDef.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>The tool over the projected edges (HLRBRep_Curve) that HLRBRep's 2D curve intersections use.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of IntCurve's generic classes (decision 322). OCCT's curve
/// type is <c>HLRBRep_CurvePtr</c>, a void pointer cast to <c>HLRBRep_Curve*</c>; the port's type argument is the
/// class <see cref="HLRBRep_Curve"/> itself (decision 1091). OCCT's <c>Bezier</c> and <c>BSpline</c> return null
/// handles ("not implemented") and are left out with nothing calling them. OCCT's function-scope static
/// <c>nbsOther</c>, never written, is a constant.
/// </remarks>
public readonly struct HLRBRep_CurveTool : IIntCurve_CurveTool<HLRBRep_Curve>
{
    // OCCT's IS_C2_COMPOSITE is 0: the curves are split at their C1 discontinuities.

    public static double FirstParameter(in HLRBRep_Curve C) => (C.FirstParameter());

    public static double LastParameter(in HLRBRep_Curve C) => (C.LastParameter());

    public static Continuity Continuity(in HLRBRep_Curve C) => (C.Continuity());

    /// <summary>The number of intervals of C1 continuity.</summary>
    public static int NbIntervals(in HLRBRep_Curve C) => (C.NbIntervals(C1));

    /// <summary>The bounds of the intervals of C1 continuity.</summary>
    public static void Intervals(in HLRBRep_Curve C, NCollection_Array1<double> Tab)
    {
        C.Intervals(Tab, C1);
    }

    public static void GetInterval(in HLRBRep_Curve C, int i, NCollection_Array1<double> Tab, out double a,
                                   out double b)
    {
        a = Tab.Value(i);
        b = Tab.Value(i + 1);
    }

    public static bool IsClosed(in HLRBRep_Curve C) => (C.IsClosed());

    public static bool IsPeriodic(in HLRBRep_Curve C) => (C.IsPeriodic());

    public static double Period(in HLRBRep_Curve C) => (C.Period());

    public static Point2D Value(in HLRBRep_Curve C, double U) => (C.Value(U));

    public static void D0(in HLRBRep_Curve C, double U, out Point2D P)
    {
        C.D0(U, out P);
    }

    public static void D1(in HLRBRep_Curve C, double U, out Point2D P, out Vector2D T)
    {
        C.D1(U, out P, out T);
    }

    public static void D2(in HLRBRep_Curve C, double U, out Point2D P, out Vector2D T, out Vector2D N)
    {
        C.D2(U, out P, out T, out N);
    }

    /// <summary>OCCT's HLRBRep_Curve::D3, which writes nothing.</summary>
    public static void D3(in HLRBRep_Curve C, double U, ref Point2D P, ref Vector2D V1, ref Vector2D V2,
                          ref Vector2D V3)
    {
        C.D3(U, ref P, ref V1, ref V2, ref V3);
    }

    public static Vector2D DN(in HLRBRep_Curve C, double U, int N) => (C.DN(U, N));

    public static double Resolution(in HLRBRep_Curve C, double R3d) => (C.Resolution(R3d));

    public static CurveKind GetType(in HLRBRep_Curve C) => (C.GetType());

    public static CurveKind TheType(in HLRBRep_Curve C) => (C.GetType());

    public static Line2D Line(in HLRBRep_Curve C) => (C.Line());

    public static Circle2D Circle(in HLRBRep_Curve C) => (C.Circle());

    public static Ellipse2D Ellipse(in HLRBRep_Curve C) => (C.Ellipse());

    public static Parabola2D Parabola(in HLRBRep_Curve C) => (C.Parabola());

    public static Hyperbola2D Hyperbola(in HLRBRep_Curve C) => (C.Hyperbola());

    public static double EpsX(in HLRBRep_Curve C) => (1e-10);

    public static int Degree(in HLRBRep_Curve C) => (C.Degree());

    /// <summary>The number of samples of the curve on its whole range, between 2 and 50.</summary>
    public static int NbSamples(in HLRBRep_Curve C)
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
            nbs = 3 + C.NbPoles();
        }
        else if (typC == BSplineCurve)
        {
            nbs = C.NbKnots();
            nbs *= C.Degree();
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

    /// <summary>The number of samples of the curve on [u1, u2], between 2 and 50.</summary>
    public static int NbSamples(in HLRBRep_Curve C, double u1, double u2)
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
            nbs = 3 + C.NbPoles();
        }
        else if (typC == BSplineCurve)
        {
            Curve3D? aCurve = C.Curve().Curve().Curve();
            CurveAdaptor GAcurve = new(aCurve!, u1, u2);
            nbs = GAcurve.NbIntervals(CN) + 1;
            nbs *= C.Degree();
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
