// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_HCurve2dTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_HCurve2dTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_HCurve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Static access to a 2D curve adaptor (an arc of restriction), as the contour algorithms call it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members (decision 338), so that it can be the
/// arc tool of IntStart's generics: it implements
/// <see cref="IIntImp_CurveTool{TheCurve}"/> over <see cref="Adaptor2d_Curve2d"/> (decision 344).
/// </remarks>
public readonly struct Contap_HCurve2dTool : IIntImp_CurveTool<Adaptor2d_Curve2d>
{
    public static double FirstParameter(in Adaptor2d_Curve2d C) => (C.FirstParameter());

    public static double LastParameter(in Adaptor2d_Curve2d C) => (C.LastParameter());

    public static GeomAbs_Shape Continuity(in Adaptor2d_Curve2d C) => (C.Continuity());

    /// <summary>The number of intervals for continuity <paramref name="S"/>.</summary>
    public static int NbIntervals(in Adaptor2d_Curve2d C, GeomAbs_Shape S) => (C.NbIntervals(S));

    /// <summary>Stores in <paramref name="T"/> the parameters bounding the intervals of continuity <paramref name="S"/>.</summary>
    public static void Intervals(in Adaptor2d_Curve2d C, NCollection_Array1<double> T, GeomAbs_Shape S) =>
        C.Intervals(T, S);

    public static bool IsClosed(in Adaptor2d_Curve2d C) => (C.IsClosed());

    public static bool IsPeriodic(in Adaptor2d_Curve2d C) => (C.IsPeriodic());

    public static double Period(in Adaptor2d_Curve2d C) => (C.Period());

    public static gp_Pnt2d Value(in Adaptor2d_Curve2d C, double U) => (C.Value(U));

    public static void D0(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P) => C.D0(U, out P);

    public static void D1(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d V) => C.D1(U, out P, out V);

    public static void D2(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2) =>
        C.D2(U, out P, out V1, out V2);

    public static void D3(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2,
                          out gp_Vec2d V3) => C.D3(U, out P, out V1, out V2, out V3);

    public static gp_Vec2d DN(in Adaptor2d_Curve2d C, double U, int N) => (C.DN(U, N));

    /// <summary>The parametric resolution corresponding to the real space resolution <paramref name="R3d"/>.</summary>
    public static double Resolution(in Adaptor2d_Curve2d C, double R3d) => (C.Resolution(R3d));

    public static GeomAbs_CurveType GetType(in Adaptor2d_Curve2d C) => (C.GetType());

    public static gp_Lin2d Line(in Adaptor2d_Curve2d C) => (C.Line());

    public static gp_Circ2d Circle(in Adaptor2d_Curve2d C) => (C.Circle());

    public static gp_Elips2d Ellipse(in Adaptor2d_Curve2d C) => (C.Ellipse());

    public static gp_Hypr2d Hyperbola(in Adaptor2d_Curve2d C) => (C.Hyperbola());

    public static gp_Parab2d Parabola(in Adaptor2d_Curve2d C) => (C.Parabola());

    public static Geom2d_BezierCurve Bezier(in Adaptor2d_Curve2d C) => (C.Bezier());

    public static Geom2d_BSplineCurve BSpline(in Adaptor2d_Curve2d C) => (C.BSpline());

    /// <summary>The number of samples on the curve between <paramref name="U0"/> and <paramref name="U1"/> (2 to 50).</summary>
    public static int NbSamples(in Adaptor2d_Curve2d C, double U0, double U1)
    {
        double nbs = 10.0;
        switch (C.GetType())
        {
            case GeomAbs_Line:
                nbs = 2.0;
                break;
            case GeomAbs_BezierCurve:
                nbs = 3.0 + C.NbPoles();
                break;
            case GeomAbs_BSplineCurve:
                nbs = C.NbKnots();
                nbs *= C.Degree();
                // OCCT: the inverse of IntPatch_HCurve2dTool's corrected ratio (U1 - U0) / (Last - First)
                nbs *= C.LastParameter() - C.FirstParameter();
                nbs /= U1 - U0;
                if (nbs < 2.0)
                {
                    nbs = 2.0;
                }
                break;
            default:
                break;
        }
        if (nbs > 50.0)
        {
            nbs = 50.0;
        }
        return ((int)nbs);
    }
}
