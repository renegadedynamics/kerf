// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedTriangulation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by two arrays of nodes of a triangulation (a seam).</summary>
internal class PolygonOnClosedTriangulation : PolygonOnTriangulationRepresentation
{
    private PolygonOnTriangulation? _polygon2;

    /// <summary>The polygons <paramref name="p1"/> and <paramref name="p2"/> on <paramref name="t"/> (all frozen here).</summary>
    public PolygonOnClosedTriangulation(PolygonOnTriangulation? p1,
                                             PolygonOnTriangulation? p2,
                                             Triangulation? t,
                                             in Placement l)
        : this(p1, p2, t, l, true)
    {
    }

    /// <summary>
    /// The polygons <paramref name="p1"/> and <paramref name="p2"/> on <paramref name="t"/>, all frozen if
    /// <paramref name="toFreeze"/> (the mesher's non-freezing attach, decision 488).
    /// </summary>
    internal PolygonOnClosedTriangulation(PolygonOnTriangulation? p1,
                                               PolygonOnTriangulation? p2,
                                               Triangulation? t,
                                               in Placement l,
                                               bool toFreeze)
        : base(p1, t, l, toFreeze)
    {
        if (toFreeze)
        {
            p2?.Freeze(); // Kerf: attached meshes are frozen
        }
        _polygon2 = p2;
    }

    internal override void FreezeContents()
    {
        base.FreezeContents();
        _polygon2?.Freeze();
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnClosedTriangulation => true;

    internal override void PolygonOnTriangulation2(PolygonOnTriangulation? p2)
    {
        CheckMutable();
        p2?.Freeze(); // Kerf: attached meshes are frozen
        _polygon2 = p2;
    }

    public override PolygonOnTriangulation? PolygonOnTriangulation2() => _polygon2;

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        PolygonOnClosedTriangulation p =
            new PolygonOnClosedTriangulation(PolygonOnTriangulation(), _polygon2, Triangulation(), Location());
        return p;
    }
}
