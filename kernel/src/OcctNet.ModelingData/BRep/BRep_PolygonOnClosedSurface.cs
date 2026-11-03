// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Topology;

namespace Occt;

/// <summary>The representation of an edge by two 2D polygons in the parametric space of a closed surface.</summary>
public class BRep_PolygonOnClosedSurface : BRep_PolygonOnSurface
{
    private Polygon2D? myPolygon2;

    /// <summary>The polygons <paramref name="P1"/> and <paramref name="P2"/> on <paramref name="S"/> (all frozen here).</summary>
    public BRep_PolygonOnClosedSurface(Polygon2D? P1, Polygon2D? P2, Geom_Surface? S, in Placement L)
        : base(P1, S, L)
    {
        P2?.Freeze(); // OCCT.NET: attached meshes are frozen
        myPolygon2 = P2;
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnClosedSurface() => true;

    public override Polygon2D? Polygon2() => myPolygon2;

    internal override void Polygon2(Polygon2D? P)
    {
        CheckMutable();
        P?.Freeze(); // OCCT.NET: attached meshes are frozen
        myPolygon2 = P;
    }

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_PolygonOnClosedSurface P = new BRep_PolygonOnClosedSurface(Polygon(), myPolygon2, Surface(), Location());
        return P;
    }
}
