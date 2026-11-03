// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnTriangulation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Topology;

namespace Occt;

/// <summary>The representation of an edge by an array of nodes of a triangulation.</summary>
public class BRep_PolygonOnTriangulation : BRep_CurveRepresentation
{
    private PolygonOnTriangulation? myPolygon;
    private readonly Triangulation? myTriangulation;

    /// <summary>The polygon <paramref name="P"/> on <paramref name="T"/> (both frozen here).</summary>
    public BRep_PolygonOnTriangulation(PolygonOnTriangulation? P, Triangulation? T, in Placement L)
        : this(P, T, L, true)
    {
    }

    /// <summary>
    /// The polygon <paramref name="P"/> on <paramref name="T"/>, both frozen if <paramref name="theToFreeze"/> (the
    /// mesher's non-freezing attach, decision 488).
    /// </summary>
    internal BRep_PolygonOnTriangulation(PolygonOnTriangulation? P, Triangulation? T, in Placement L,
                                         bool theToFreeze)
        : base(L)
    {
        if (theToFreeze)
        {
            P?.Freeze(); // OCCT.NET: attached meshes are frozen
            T?.Freeze();
        }
        myPolygon = P;
        myTriangulation = T;
    }

    internal override void FreezeContents()
    {
        myPolygon?.Freeze();
        myTriangulation?.Freeze();
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnTriangulation() => true;

    /// <summary>A polygon in the definition of <paramref name="T"/> with location <paramref name="L"/>.</summary>
    public override bool IsPolygonOnTriangulation(Triangulation? T, in Placement L)
        => ReferenceEquals(T, myTriangulation) && (L == myLocation);

    internal override void PolygonOnTriangulation(PolygonOnTriangulation? P)
    {
        CheckMutable();
        P?.Freeze(); // OCCT.NET: attached meshes are frozen
        myPolygon = P;
    }

    public override PolygonOnTriangulation? PolygonOnTriangulation() => myPolygon;

    public override Triangulation? Triangulation() => myTriangulation;

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_PolygonOnTriangulation P = new BRep_PolygonOnTriangulation(myPolygon, myTriangulation, Location());
        return P;
    }
}
