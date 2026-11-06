// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by two 2D polygons in the parametric space of a closed surface.</summary>
internal class PolygonOnClosedSurface : PolygonOnSurface
{
    private Polygon2D? _polygon2;

    /// <summary>The polygons <paramref name="p1"/> and <paramref name="p2"/> on <paramref name="s"/> (all frozen here).</summary>
    public PolygonOnClosedSurface(Polygon2D? p1, Polygon2D? p2, Surface? s, in Placement l)
        : base(p1, s, l)
    {
        p2?.Freeze(); // Kerf: attached meshes are frozen
        _polygon2 = p2;
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnClosedSurface => true;

    public override Polygon2D? Polygon2() => _polygon2;

    internal override void Polygon2(Polygon2D? p)
    {
        CheckMutable();
        p?.Freeze(); // Kerf: attached meshes are frozen
        _polygon2 = p;
    }

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        PolygonOnClosedSurface p = new PolygonOnClosedSurface(Polygon(), _polygon2, Surface(), Location());
        return p;
    }
}
