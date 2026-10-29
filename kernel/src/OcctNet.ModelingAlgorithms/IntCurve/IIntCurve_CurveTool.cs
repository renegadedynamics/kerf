// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The static tool over a 2D curve that IntCurve's generic classes use (OCCT's TheCurveTool macro:
/// Geom2dInt_Geom2dCurveTool over Adaptor2d_Curve2d, HLRBRep_CurveTool over HLRBRep's curves).
/// </summary>
/// <remarks>Not an OCCT type (decision 322).</remarks>
/// <typeparam name="TheCurve">The curve.</typeparam>
public interface IIntCurve_CurveTool<TheCurve> : IIntImpParGen_ParTool<TheCurve>
{
    static abstract GeomAbs_CurveType GetType(in TheCurve C);

    static abstract gp_Lin2d Line(in TheCurve C);

    static abstract gp_Circ2d Circle(in TheCurve C);

    static abstract gp_Elips2d Ellipse(in TheCurve C);

    static abstract gp_Parab2d Parabola(in TheCurve C);

    static abstract gp_Hypr2d Hyperbola(in TheCurve C);

    static abstract void D0(in TheCurve C, double U, out gp_Pnt2d P);

    static abstract void D1(in TheCurve C, double U, out gp_Pnt2d P, out gp_Vec2d T);

    static abstract double FirstParameter(in TheCurve C);

    static abstract double LastParameter(in TheCurve C);

    /// <summary>The number of samples of the curve on its whole range.</summary>
    static abstract int NbSamples(in TheCurve C);

    /// <summary>The number of intervals of continuity the intersection splits the curve into.</summary>
    static abstract int NbIntervals(in TheCurve C);

    /// <summary>The bounds of the intervals of <see cref="NbIntervals"/>, in an array of NbIntervals + 1 values.</summary>
    static abstract void Intervals(in TheCurve C, NCollection_Array1<double> Tab);

    /// <summary>The bounds of the interval <paramref name="i"/> in <paramref name="Tab"/>.</summary>
    static abstract void GetInterval(in TheCurve C, int i, NCollection_Array1<double> Tab, out double a, out double b);
}
