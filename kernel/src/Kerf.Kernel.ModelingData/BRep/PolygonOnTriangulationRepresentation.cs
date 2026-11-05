// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnTriangulation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by an array of nodes of a triangulation.</summary>
internal class PolygonOnTriangulationRepresentation : CurveRepresentation
{
    private PolygonOnTriangulation? _polygon;
    private readonly Triangulation? _triangulation;

    /// <summary>The polygon <paramref name="p"/> on <paramref name="t"/> (both frozen here).</summary>
    public PolygonOnTriangulationRepresentation(PolygonOnTriangulation? p, Triangulation? t, in Placement l)
        : this(p, t, l, true)
    {
    }

    /// <summary>
    /// The polygon <paramref name="p"/> on <paramref name="t"/>, both frozen if <paramref name="toFreeze"/> (the
    /// mesher's non-freezing attach, decision 488).
    /// </summary>
    internal PolygonOnTriangulationRepresentation(PolygonOnTriangulation? p, Triangulation? t, in Placement l,
                                         bool toFreeze)
        : base(l)
    {
        if (toFreeze)
        {
            p?.Freeze(); // Kerf: attached meshes are frozen
            t?.Freeze();
        }
        _polygon = p;
        _triangulation = t;
    }

    internal override void FreezeContents()
    {
        _polygon?.Freeze();
        _triangulation?.Freeze();
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnTriangulation() => true;

    /// <summary>A polygon in the definition of <paramref name="t"/> with location <paramref name="l"/>.</summary>
    public override bool IsPolygonOnTriangulation(Triangulation? t, in Placement l)
        => ReferenceEquals(t, _triangulation) && (l.IsEqual(_location));

    internal override void PolygonOnTriangulation(PolygonOnTriangulation? p)
    {
        CheckMutable();
        p?.Freeze(); // Kerf: attached meshes are frozen
        _polygon = p;
    }

    public override PolygonOnTriangulation? PolygonOnTriangulation() => _polygon;

    public override Triangulation? Triangulation() => _triangulation;

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        PolygonOnTriangulationRepresentation p = new PolygonOnTriangulationRepresentation(_polygon, _triangulation, Location());
        return p;
    }
}
