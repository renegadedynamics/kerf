// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HCurve2dTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HCurve2dTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HCurve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>Static access to a 2D curve adaptor (an arc of restriction), as the intersection algorithms call it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members (decision 338), so that it can be the curve
/// tool of IntImp_IntCS (<see cref="CurvIntSurf"/>) and of IntStart's generics: it implements
/// <see cref="IImplicitCurveTool{TheCurve}"/> over <see cref="Curve2DAdaptorBase"/> (decision 344).
/// </remarks>
internal readonly struct PatchHCurve2dTool : IImplicitCurveTool<Curve2DAdaptorBase>
{
    public static double FirstParameter(in Curve2DAdaptorBase c) => (c.FirstParameter());

    public static double LastParameter(in Curve2DAdaptorBase c) => (c.LastParameter());

    public static Continuity Continuity(in Curve2DAdaptorBase c) => (c.Continuity());

    /// <summary>The number of intervals for continuity <paramref name="s"/>.</summary>
    public static int NbIntervals(in Curve2DAdaptorBase c, Continuity s) => (c.NbIntervals(s));

    /// <summary>Stores in <paramref name="t"/> the parameters bounding the intervals of continuity <paramref name="s"/>.</summary>
    public static void Intervals(in Curve2DAdaptorBase c, NCollection_Array1<double> t, Continuity s) =>
        c.Intervals(t, s);

    public static bool IsClosed(in Curve2DAdaptorBase c) => (c.IsClosed);

    public static bool IsPeriodic(in Curve2DAdaptorBase c) => (c.IsPeriodic);

    public static double Period(in Curve2DAdaptorBase c) => (c.Period());

    public static Point2D Value(in Curve2DAdaptorBase c, double u) => (c.Value(u));

    public static void D0(in Curve2DAdaptorBase c, double u, out Point2D p) => c.D0(u, out p);

    public static void D1(in Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v) => c.D1(u, out p, out v);

    public static void D2(in Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v1, out Vector2D v2) =>
        c.D2(u, out p, out v1, out v2);

    public static void D3(in Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v1, out Vector2D v2,
                          out Vector2D v3) => c.D3(u, out p, out v1, out v2, out v3);

    public static Vector2D DN(in Curve2DAdaptorBase c, double u, int n) => (c.DN(u, n));

    /// <summary>The parametric resolution corresponding to the real space resolution <paramref name="r3d"/>.</summary>
    public static double Resolution(in Curve2DAdaptorBase c, double r3d) => (c.Resolution(r3d));

    public static CurveKind GetType(in Curve2DAdaptorBase c) => (c.GetType());

    public static Line2D Line(in Curve2DAdaptorBase c) => (c.Line());

    public static Circle2D Circle(in Curve2DAdaptorBase c) => (c.Circle());

    public static Ellipse2D Ellipse(in Curve2DAdaptorBase c) => (c.Ellipse());

    public static Hyperbola2D Hyperbola(in Curve2DAdaptorBase c) => (c.Hyperbola());

    public static Parabola2D Parabola(in Curve2DAdaptorBase c) => (c.Parabola());

    public static BezierCurve2D Bezier(in Curve2DAdaptorBase c) => (c.Bezier());

    public static BSplineCurve2D BSpline(in Curve2DAdaptorBase c) => (c.BSpline());

    /// <summary>The number of samples on the curve between <paramref name="u0"/> and <paramref name="u1"/> (2 to 50).</summary>
    public static int NbSamples(in Curve2DAdaptorBase c, double u0, double u1)
    {
        double nbs;
        switch (c.GetType())
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return 2;
            case BezierCurve:
            {
                nbs = (3 + c.PoleCount);
            }
            break;
            case BSplineCurve:
            {
                nbs = c.KnotCount;
                nbs *= c.Degree();
                // szv:nbs *= C->LastParameter() - C->FirstParameter();
                // szv:nbs /= U1-U0;
                nbs *= u1 - u0;
                nbs /= c.LastParameter() - c.FirstParameter();
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
