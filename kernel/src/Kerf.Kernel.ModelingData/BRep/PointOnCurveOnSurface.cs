// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurveOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of a vertex by a parameter on a curve on a surface.</summary>
internal class PointOnCurveOnSurface : PointsOnSurface
{
    private Curve2D? _pCurve;

    /// <summary>The parameter <paramref name="p"/> on the pcurve <paramref name="c"/> on <paramref name="s"/> (frozen here).</summary>
    public PointOnCurveOnSurface(double p, Curve2D? c, Surface? s, in Placement l) : base(p, s, l)
    {
        c?.Freeze(); // Kerf: attached geometry is frozen
        _pCurve = c;
    }

    /// <summary>True.</summary>
    public override bool IsPointOnCurveOnSurface() => true;

    /// <summary>A point on the 2D curve <paramref name="pc"/> on the surface <paramref name="s"/>.</summary>
    public override bool IsPointOnCurveOnSurface(Curve2D? pc, Surface? s, in Placement l)
        => ReferenceEquals(_pCurve, pc) && ReferenceEquals(Surface(), s) && (Location().IsEqual(l));

    public override Curve2D? PCurve() => _pCurve;

    internal override void PCurve(Curve2D? c)
    {
        CheckMutable();
        c?.Freeze(); // Kerf: attached geometry is frozen
        _pCurve = c;
    }
}
