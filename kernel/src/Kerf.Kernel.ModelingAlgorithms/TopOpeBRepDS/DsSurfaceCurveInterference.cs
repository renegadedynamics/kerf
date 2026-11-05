// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceCurveInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceCurveInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>An interference with a 2D curve: a curve on a face or surface, with its pcurve there.</summary>
internal sealed class DsSurfaceCurveInterference : DsInterference
{
    private Curve2D? _pCurve;

    /// <summary>An empty interference.</summary>
    public DsSurfaceCurveInterference()
    {
    }

    /// <summary>
    /// The interference of the geometry <paramref name="g"/> on the support <paramref name="s"/>, with the pcurve
    /// <paramref name="pc"/> on the support.
    /// </summary>
    public DsSurfaceCurveInterference(in DsTransition t,
                                                 DsKind st,
                                                 int s,
                                                 DsKind gt,
                                                 int g,
                                                 Curve2D? pc)
        : base(t, st, s, gt, g)
    {
        _pCurve = pc;
    }

    /// <summary>A copy of the transition, support and geometry of <paramref name="i"/>, with no pcurve.</summary>
    public DsSurfaceCurveInterference(DsInterference i)
        : base(i)
    {
    }

    /// <summary>The pcurve on the support.</summary>
    public Curve2D? PCurve() => _pCurve;

    internal void PCurve(Curve2D? pc)
    {
        _pCurve = pc;
    }
}
