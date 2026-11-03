// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by a 2D polygon in the parametric space of a surface.</summary>
internal class PolygonOnSurface : CurveRepresentation
{
    private Polygon2D? _polygon2D;
    private readonly Surface? _surface;

    /// <summary>The polygon <paramref name="p"/> on <paramref name="s"/> (both frozen here).</summary>
    public PolygonOnSurface(Polygon2D? p, Surface? s, in Placement l) : base(l)
    {
        p?.Freeze(); // OCCT.NET: attached meshes and geometry are frozen
        s?.Freeze();
        _polygon2D = p;
        _surface = s;
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnSurface() => true;

    /// <summary>A polygon in the parametric space of <paramref name="s"/> with location <paramref name="l"/>.</summary>
    public override bool IsPolygonOnSurface(Surface? s, in Placement l)
        => ReferenceEquals(s, _surface) && (l.IsEqual(_location));

    public override Surface? Surface() => _surface;

    public override Polygon2D? Polygon() => _polygon2D;

    internal override void Polygon(Polygon2D? p)
    {
        CheckMutable();
        p?.Freeze(); // OCCT.NET: attached meshes are frozen
        _polygon2D = p;
    }

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        PolygonOnSurface p = new PolygonOnSurface(_polygon2D, _surface, Location());
        return p;
    }
}
