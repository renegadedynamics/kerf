// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurve.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of a vertex by a parameter on a 3D curve.</summary>
internal class PointOnCurve : PointRepresentation
{
    private Curve3D? _curve;

    /// <summary>The parameter <paramref name="p"/> on <paramref name="c"/> (frozen here).</summary>
    public PointOnCurve(double p, Curve3D? c, in Placement l) : base(p, l)
    {
        c?.Freeze(); // Kerf: attached geometry is frozen
        _curve = c;
    }

    /// <summary>True.</summary>
    public override bool IsPointOnCurve() => true;

    /// <summary>A point on the curve <paramref name="c"/>.</summary>
    public override bool IsPointOnCurve(Curve3D? c, in Placement l)
        => ReferenceEquals(_curve, c) && (Location().IsEqual(l));

    public override Curve3D? Curve() => _curve;

    internal override void Curve(Curve3D? c)
    {
        CheckMutable();
        c?.Freeze(); // Kerf: attached geometry is frozen
        _curve = c;
    }
}
