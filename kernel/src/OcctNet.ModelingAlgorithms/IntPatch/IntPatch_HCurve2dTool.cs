// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HCurve2dTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HCurve2dTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HCurve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Static access to a 2D curve adaptor (an arc of restriction), as the intersection algorithms call it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members (decision 338), so that it can be the curve
/// tool of IntImp_IntCS (<see cref="IntPatch_CurvIntSurf"/>) and of IntStart's generics: it implements
/// <see cref="IIntImp_CurveTool{TheCurve}"/> over <see cref="Curve2DAdaptorBase"/> (decision 344).
/// </remarks>
public readonly struct IntPatch_HCurve2dTool : IIntImp_CurveTool<Curve2DAdaptorBase>
{
    public static double FirstParameter(in Curve2DAdaptorBase C) => (C.FirstParameter());

    public static double LastParameter(in Curve2DAdaptorBase C) => (C.LastParameter());

    public static Continuity Continuity(in Curve2DAdaptorBase C) => (C.Continuity());

    /// <summary>The number of intervals for continuity <paramref name="S"/>.</summary>
    public static int NbIntervals(in Curve2DAdaptorBase C, Continuity S) => (C.NbIntervals(S));

    /// <summary>Stores in <paramref name="T"/> the parameters bounding the intervals of continuity <paramref name="S"/>.</summary>
    public static void Intervals(in Curve2DAdaptorBase C, NCollection_Array1<double> T, Continuity S) =>
        C.Intervals(T, S);

    public static bool IsClosed(in Curve2DAdaptorBase C) => (C.IsClosed);

    public static bool IsPeriodic(in Curve2DAdaptorBase C) => (C.IsPeriodic);

    public static double Period(in Curve2DAdaptorBase C) => (C.Period());

    public static Point2D Value(in Curve2DAdaptorBase C, double U) => (C.Value(U));

    public static void D0(in Curve2DAdaptorBase C, double U, out Point2D P) => C.D0(U, out P);

    public static void D1(in Curve2DAdaptorBase C, double U, out Point2D P, out Vector2D V) => C.D1(U, out P, out V);

    public static void D2(in Curve2DAdaptorBase C, double U, out Point2D P, out Vector2D V1, out Vector2D V2) =>
        C.D2(U, out P, out V1, out V2);

    public static void D3(in Curve2DAdaptorBase C, double U, out Point2D P, out Vector2D V1, out Vector2D V2,
                          out Vector2D V3) => C.D3(U, out P, out V1, out V2, out V3);

    public static Vector2D DN(in Curve2DAdaptorBase C, double U, int N) => (C.DN(U, N));

    /// <summary>The parametric resolution corresponding to the real space resolution <paramref name="R3d"/>.</summary>
    public static double Resolution(in Curve2DAdaptorBase C, double R3d) => (C.Resolution(R3d));

    public static CurveKind GetType(in Curve2DAdaptorBase C) => (C.GetType());

    public static Line2D Line(in Curve2DAdaptorBase C) => (C.Line());

    public static Circle2D Circle(in Curve2DAdaptorBase C) => (C.Circle());

    public static Ellipse2D Ellipse(in Curve2DAdaptorBase C) => (C.Ellipse());

    public static Hyperbola2D Hyperbola(in Curve2DAdaptorBase C) => (C.Hyperbola());

    public static Parabola2D Parabola(in Curve2DAdaptorBase C) => (C.Parabola());

    public static BezierCurve2D Bezier(in Curve2DAdaptorBase C) => (C.Bezier());

    public static BSplineCurve2D BSpline(in Curve2DAdaptorBase C) => (C.BSpline());

    /// <summary>The number of samples on the curve between <paramref name="U0"/> and <paramref name="U1"/> (2 to 50).</summary>
    public static int NbSamples(in Curve2DAdaptorBase C, double U0, double U1)
    {
        double nbs;
        switch (C.GetType())
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return 2;
            case BezierCurve:
            {
                nbs = (3 + C.PoleCount);
            }
            break;
            case BSplineCurve:
            {
                nbs = C.KnotCount;
                nbs *= C.Degree();
                // szv:nbs *= C->LastParameter() - C->FirstParameter();
                // szv:nbs /= U1-U0;
                nbs *= U1 - U0;
                nbs /= C.LastParameter() - C.FirstParameter();
                if (nbs < 2.0)
                {
                    nbs = 2.0;
                }
            }
            break;
            default:
                return 25;
        }
        if (nbs > 50.0)
        {
            return 50;
        }
        return ((int)nbs);
    }
}
