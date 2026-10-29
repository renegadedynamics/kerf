// Port of: src/ModelingData/TKBRep/BRep/BRep_Curve3D.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Curve3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The representation of an edge by a 3D curve.</summary>
public class BRep_Curve3D : BRep_GCurve
{
    private Geom_Curve? myCurve;

    /// <summary>The curve <paramref name="C"/> (frozen here) with location <paramref name="L"/>, over its whole range.</summary>
    public BRep_Curve3D(Geom_Curve? C, in TopLoc_Location L)
        : base(L, C is null ? RealFirst() : C.FirstParameter(), C is null ? RealLast() : C.LastParameter())
    {
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myCurve = C;
    }

    /// <summary>The point at parameter <paramref name="U"/>.</summary>
    public override void D0(double U, out gp_Pnt P)
    {
        // should be D0 NYI
        P = myCurve!.Value(U);
    }

    /// <summary>True.</summary>
    public override bool IsCurve3D() => true;

    public override Geom_Curve? Curve3D() => myCurve;

    internal override void Curve3D(Geom_Curve? C)
    {
        CheckMutable();
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myCurve = C;
    }

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_Curve3D C = new BRep_Curve3D(myCurve, Location());
        C.SetRange(First(), Last());
        return C;
    }
}
