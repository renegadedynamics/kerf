// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnClosedSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnClosedSurface.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnClosedSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The representation of an edge by two pcurves on a closed surface (a seam).</summary>
public class BRep_CurveOnClosedSurface : BRep_CurveOnSurface
{
    private Geom2d_Curve? myPCurve2;
    private GeomAbs_Shape myContinuity;
    private gp_Pnt2d myUV21;
    private gp_Pnt2d myUV22;

    /// <summary>The pcurves <paramref name="PC1"/> and <paramref name="PC2"/> on <paramref name="S"/> (all frozen here).</summary>
    public BRep_CurveOnClosedSurface(Geom2d_Curve PC1,
                                     Geom2d_Curve? PC2,
                                     Geom_Surface? S,
                                     in TopLoc_Location L,
                                     GeomAbs_Shape C)
        : base(PC1, S, L)
    {
        PC2?.Freeze(); // OCCT.NET: attached geometry is frozen
        myPCurve2 = PC2;
        myContinuity = C;
        myUV21 = new gp_Pnt2d();
        myUV22 = new gp_Pnt2d();
    }

    internal void SetUVPoints2(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        CheckMutable();
        myUV21 = P1;
        myUV22 = P2;
    }

    /// <summary>The UV points of the second pcurve at the ends of the range.</summary>
    public void UVPoints2(out gp_Pnt2d P1, out gp_Pnt2d P2)
    {
        P1 = myUV21;
        P2 = myUV22;
    }

    /// <summary>True.</summary>
    public override bool IsCurveOnClosedSurface() => true;

    /// <summary>True.</summary>
    public override bool IsRegularity() => true;

    /// <summary>A curve on two surfaces (continuity).</summary>
    public override bool IsRegularity(Geom_Surface? S1, Geom_Surface? S2, in TopLoc_Location L1, in TopLoc_Location L2)
        => ReferenceEquals(Surface(), S1) && ReferenceEquals(Surface(), S2) && (Location() == L1) && (Location() == L2);

    public override Geom2d_Curve? PCurve2() => myPCurve2;

    /// <summary>Surface().</summary>
    public override Geom_Surface? Surface2() => Surface();

    /// <summary>Location().</summary>
    public override TopLoc_Location Location2() => Location();

    public override GeomAbs_Shape Continuity() => myContinuity;

    internal override void Continuity(GeomAbs_Shape C)
    {
        CheckMutable();
        myContinuity = C;
    }

    internal override void PCurve2(Geom2d_Curve? C)
    {
        CheckMutable();
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myPCurve2 = C;
    }

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_CurveOnClosedSurface C =
            new BRep_CurveOnClosedSurface(PCurve()!, PCurve2(), Surface(), Location(), myContinuity);

        C.SetRange(First(), Last());
        C.SetUVPoints(myUV1, myUV2);
        C.SetUVPoints2(myUV21, myUV22);

        return C;
    }

    /// <summary>Recomputes the UV points of both pcurves after the range changed.</summary>
    internal override void Update()
    {
        CheckMutable();
        if (!Precision.IsNegativeInfinite(First()))
        {
            myPCurve2!.D0(First(), out myUV21);
        }

        if (!Precision.IsPositiveInfinite(Last()))
        {
            myPCurve2!.D0(Last(), out myUV22);
        }

        base.Update();
    }
}
