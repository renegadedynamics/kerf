// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of a vertex by parameters (U, V) on a surface.</summary>
internal class PointOnSurface : PointsOnSurface
{
    private double _parameter2;

    /// <summary>The parameters (<paramref name="p1"/>, <paramref name="p2"/>) on <paramref name="s"/> (frozen here).</summary>
    public PointOnSurface(double p1, double p2, Surface? s, in Placement l) : base(p1, s, l)
        => _parameter2 = p2;

    /// <summary>True.</summary>
    public override bool IsPointOnSurface() => true;

    /// <summary>A point on the surface <paramref name="s"/>.</summary>
    public override bool IsPointOnSurface(Surface? s, in Placement l)
        => ReferenceEquals(Surface(), s) && (Location().IsEqual(l));

    public override double Parameter2() => _parameter2;

    internal override void Parameter2(double p)
    {
        CheckMutable();
        _parameter2 = p;
    }
}
