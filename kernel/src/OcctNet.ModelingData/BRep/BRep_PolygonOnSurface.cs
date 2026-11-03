// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Topology;

namespace Occt;

/// <summary>The representation of an edge by a 2D polygon in the parametric space of a surface.</summary>
public class BRep_PolygonOnSurface : BRep_CurveRepresentation
{
    private Polygon2D? myPolygon2D;
    private readonly Geom_Surface? mySurface;

    /// <summary>The polygon <paramref name="P"/> on <paramref name="S"/> (both frozen here).</summary>
    public BRep_PolygonOnSurface(Polygon2D? P, Geom_Surface? S, in Placement L) : base(L)
    {
        P?.Freeze(); // OCCT.NET: attached meshes and geometry are frozen
        S?.Freeze();
        myPolygon2D = P;
        mySurface = S;
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnSurface() => true;

    /// <summary>A polygon in the parametric space of <paramref name="S"/> with location <paramref name="L"/>.</summary>
    public override bool IsPolygonOnSurface(Geom_Surface? S, in Placement L)
        => ReferenceEquals(S, mySurface) && (L == myLocation);

    public override Geom_Surface? Surface() => mySurface;

    public override Polygon2D? Polygon() => myPolygon2D;

    internal override void Polygon(Polygon2D? P)
    {
        CheckMutable();
        P?.Freeze(); // OCCT.NET: attached meshes are frozen
        myPolygon2D = P;
    }

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_PolygonOnSurface P = new BRep_PolygonOnSurface(myPolygon2D, mySurface, Location());
        return P;
    }
}
