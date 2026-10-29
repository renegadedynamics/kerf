// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedTriangulation.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PolygonOnClosedTriangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The representation of an edge by two arrays of nodes of a triangulation (a seam).</summary>
public class BRep_PolygonOnClosedTriangulation : BRep_PolygonOnTriangulation
{
    private Poly_PolygonOnTriangulation? myPolygon2;

    /// <summary>The polygons <paramref name="P1"/> and <paramref name="P2"/> on <paramref name="T"/> (all frozen here).</summary>
    public BRep_PolygonOnClosedTriangulation(Poly_PolygonOnTriangulation? P1,
                                             Poly_PolygonOnTriangulation? P2,
                                             Poly_Triangulation? T,
                                             in TopLoc_Location L)
        : this(P1, P2, T, L, true)
    {
    }

    /// <summary>
    /// The polygons <paramref name="P1"/> and <paramref name="P2"/> on <paramref name="T"/>, all frozen if
    /// <paramref name="theToFreeze"/> (the mesher's non-freezing attach, decision 488).
    /// </summary>
    internal BRep_PolygonOnClosedTriangulation(Poly_PolygonOnTriangulation? P1,
                                               Poly_PolygonOnTriangulation? P2,
                                               Poly_Triangulation? T,
                                               in TopLoc_Location L,
                                               bool theToFreeze)
        : base(P1, T, L, theToFreeze)
    {
        if (theToFreeze)
        {
            P2?.Freeze(); // OCCT.NET: attached meshes are frozen
        }
        myPolygon2 = P2;
    }

    internal override void FreezeContents()
    {
        base.FreezeContents();
        myPolygon2?.Freeze();
    }

    /// <summary>True.</summary>
    public override bool IsPolygonOnClosedTriangulation() => true;

    internal override void PolygonOnTriangulation2(Poly_PolygonOnTriangulation? P2)
    {
        CheckMutable();
        P2?.Freeze(); // OCCT.NET: attached meshes are frozen
        myPolygon2 = P2;
    }

    public override Poly_PolygonOnTriangulation? PolygonOnTriangulation2() => myPolygon2;

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_PolygonOnClosedTriangulation P =
            new BRep_PolygonOnClosedTriangulation(PolygonOnTriangulation(), myPolygon2, Triangulation(), Location());
        return P;
    }
}
