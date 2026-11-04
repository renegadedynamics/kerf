// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The static tool over a 2D curve that IntCurve's generic classes use (OCCT's TheCurveTool macro:
/// Geom2dInt_Geom2dCurveTool over Adaptor2d_Curve2d, HLRBRep_CurveTool over HLRBRep's curves).
/// </summary>
/// <remarks>Not an OCCT type (decision 322).</remarks>
/// <typeparam name="TheCurve">The curve.</typeparam>
internal interface ICurveCurveTool<TheCurve> : IParTool<TheCurve>
{
    static abstract CurveKind GetType(in TheCurve c);

    static abstract Line2D Line(in TheCurve c);

    static abstract Circle2D Circle(in TheCurve c);

    static abstract Ellipse2D Ellipse(in TheCurve c);

    static abstract Parabola2D Parabola(in TheCurve c);

    static abstract Hyperbola2D Hyperbola(in TheCurve c);

    static abstract void D0(in TheCurve c, double u, out Point2D p);

    static abstract void D1(in TheCurve c, double u, out Point2D p, out Vector2D t);

    static abstract double FirstParameter(in TheCurve c);

    static abstract double LastParameter(in TheCurve c);

    /// <summary>The number of samples of the curve on its whole range.</summary>
    static abstract int NbSamples(in TheCurve c);

    /// <summary>The number of intervals of continuity the intersection splits the curve into.</summary>
    static abstract int NbIntervals(in TheCurve c);

    /// <summary>The bounds of the intervals of <see cref="NbIntervals"/>, in an array of NbIntervals + 1 values.</summary>
    static abstract void Intervals(in TheCurve c, NCollection_Array1<double> tab);

    /// <summary>The bounds of the interval <paramref name="i"/> in <paramref name="tab"/>.</summary>
    static abstract void GetInterval(in TheCurve c, int i, NCollection_Array1<double> tab, out double a, out double b);
}
