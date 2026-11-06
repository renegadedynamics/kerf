// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheQuadCurvExactHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheQuadCurvExactHInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The intersection of a curve adaptor with a quadric surface adaptor (plane, cylinder, cone, sphere): the roots of
/// the quadric's implicit function along the curve, and the intervals where the curve lies on the surface.
/// </summary>
/// <remarks>Not done (<see cref="IsDone"/> false) when the surface is not a quadric or a root search failed.</remarks>
internal sealed class TheQuadCurvExactHInter
    : ITheQuadCurvExactInter<TheQuadCurvExactHInter, SurfaceAdaptorBase, CurveAdaptorBase>
{
    private readonly int _nbpnts;
    private readonly NCollection_Sequence<double> _pnts = new();
    private readonly int _nbintv;
    private readonly NCollection_Sequence<double> _intv = new();

    public TheQuadCurvExactHInter(SurfaceAdaptorBase s, CurveAdaptorBase c)
    {
        QuadricCurveExactInterUtils.PerformIntersection<
            SurfaceAdaptorBase, TheHSurfaceTool, CurveAdaptorBase, TheHCurveTool,
            TheQuadCurvFuncOfTheQuadCurvExactHInter>(s, c, _pnts, _intv, out _nbpnts, out _nbintv);
    }

    static TheQuadCurvExactHInter ITheQuadCurvExactInter<TheQuadCurvExactHInter, SurfaceAdaptorBase, CurveAdaptorBase>.Create(
        in SurfaceAdaptorBase s, in CurveAdaptorBase c) => new(s, c);

    public bool IsDone() => (_nbpnts != -1);

    public int NbRoots() => _nbpnts;

    public int IntervalCount => _nbintv;

    /// <summary>The root of range <paramref name="index"/> (a parameter on the curve).</summary>
    public double Root(int index) => _pnts[index];

    /// <summary>The bounds of the interval of range <paramref name="index"/> where the curve lies on the surface.</summary>
    public void Intervals(int index, out double a, out double b)
    {
        int index2 = index + index - 1;
        a = _intv[index2];
        b = _intv[index2 + 1];
    }
}
