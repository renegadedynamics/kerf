// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheQuadCurvExactInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheQuadCurvExactInterCSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The intersection of a curve adaptor with a quadric surface adaptor (plane, cylinder, cone, sphere): the roots of
/// the quadric's implicit function along the curve, and the intervals where the curve lies on the surface.
/// </summary>
/// <remarks>Not done (<see cref="IsDone"/> false) when the surface is not a quadric or a root search failed.</remarks>
public sealed class HLRBRep_TheQuadCurvExactInterCSurf
    : IIntCurveSurface_TheQuadCurvExactInter<HLRBRep_TheQuadCurvExactInterCSurf, HLRBRep_Surface, gp_Lin>
{
    private readonly int nbpnts;
    private readonly NCollection_Sequence<double> pnts = new();
    private readonly int nbintv;
    private readonly NCollection_Sequence<double> intv = new();

    public HLRBRep_TheQuadCurvExactInterCSurf(HLRBRep_Surface S, gp_Lin C)
    {
        IntCurveSurface_QuadricCurveExactInterUtils.PerformIntersection<
            HLRBRep_Surface, HLRBRep_SurfaceTool, gp_Lin, HLRBRep_LineTool,
            HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf>(S, C, pnts, intv, out nbpnts, out nbintv);
    }

    static HLRBRep_TheQuadCurvExactInterCSurf IIntCurveSurface_TheQuadCurvExactInter<HLRBRep_TheQuadCurvExactInterCSurf, HLRBRep_Surface, gp_Lin>.Create(
        in HLRBRep_Surface S, in gp_Lin C) => new(S, C);

    public bool IsDone() => (nbpnts != -1);

    public int NbRoots() => nbpnts;

    public int NbIntervals() => nbintv;

    /// <summary>The root of range <paramref name="Index"/> (a parameter on the curve).</summary>
    public double Root(int Index) => pnts[Index];

    /// <summary>The bounds of the interval of range <paramref name="Index"/> where the curve lies on the surface.</summary>
    public void Intervals(int Index, out double a, out double b)
    {
        int Index2 = Index + Index - 1;
        a = intv[Index2];
        b = intv[Index2 + 1];
    }
}
