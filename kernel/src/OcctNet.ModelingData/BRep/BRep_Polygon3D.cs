// Port of: src/ModelingData/TKBRep/BRep/BRep_Polygon3D.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Polygon3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The representation of an edge by a 3D polygon.</summary>
public class BRep_Polygon3D : BRep_CurveRepresentation
{
    private Poly_Polygon3D? myPolygon3D;

    /// <summary>The polygon <paramref name="P"/> (frozen here) with location <paramref name="L"/>.</summary>
    public BRep_Polygon3D(Poly_Polygon3D? P, in TopLoc_Location L) : base(L)
    {
        P?.Freeze(); // OCCT.NET: attached meshes are frozen
        myPolygon3D = P;
    }

    /// <summary>True.</summary>
    public override bool IsPolygon3D() => true;

    public override Poly_Polygon3D? Polygon3D() => myPolygon3D;

    internal override void Polygon3D(Poly_Polygon3D? P)
    {
        CheckMutable();
        P?.Freeze(); // OCCT.NET: attached meshes are frozen
        myPolygon3D = P;
    }

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_Polygon3D P = new BRep_Polygon3D(myPolygon3D, Location());
        return P;
    }
}
