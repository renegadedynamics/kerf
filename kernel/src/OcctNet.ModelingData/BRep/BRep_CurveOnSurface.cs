// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnSurface.lxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The representation of an edge by a curve in the parametric space of a surface.</summary>
public class BRep_CurveOnSurface : BRep_GCurve
{
    protected gp_Pnt2d myUV1;
    protected gp_Pnt2d myUV2;
    private Geom2d_Curve? myPCurve;
    private readonly Geom_Surface? mySurface;

    /// <summary>The pcurve <paramref name="PC"/> on <paramref name="S"/> with location <paramref name="L"/> (both frozen here).</summary>
    public BRep_CurveOnSurface(Geom2d_Curve PC, Geom_Surface? S, in TopLoc_Location L)
        : base(L, PC.FirstParameter(), PC.LastParameter())
    {
        PC.Freeze(); // OCCT.NET: attached geometry is frozen
        S?.Freeze();
        myUV1 = new gp_Pnt2d();
        myUV2 = new gp_Pnt2d();
        myPCurve = PC;
        mySurface = S;
    }

    internal void SetUVPoints(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        CheckMutable();
        myUV1 = P1;
        myUV2 = P2;
    }

    /// <summary>The UV points at the ends of the range.</summary>
    public void UVPoints(out gp_Pnt2d P1, out gp_Pnt2d P2)
    {
        P1 = myUV1;
        P2 = myUV2;
    }

    /// <summary>The point at parameter <paramref name="U"/>.</summary>
    public override void D0(double U, out gp_Pnt P)
    {
        // should be D0 NYI
        gp_Pnt2d P2d = myPCurve!.Value(U);
        P = mySurface!.Value(P2d.X(), P2d.Y());
        P.Transform(myLocation.Transformation());
    }

    /// <summary>True.</summary>
    public override bool IsCurveOnSurface() => true;

    /// <summary>A curve in the parametric space of <paramref name="S"/> with location <paramref name="L"/>.</summary>
    public override bool IsCurveOnSurface(Geom_Surface? S, in TopLoc_Location L)
        => ReferenceEquals(S, mySurface) && (L == myLocation);

    public override Geom_Surface? Surface() => mySurface;

    public override Geom2d_Curve? PCurve() => myPCurve;

    internal override void PCurve(Geom2d_Curve? C)
    {
        CheckMutable();
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myPCurve = C;
    }

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_CurveOnSurface C = new BRep_CurveOnSurface(myPCurve!, mySurface, Location());

        C.SetRange(First(), Last());
        C.SetUVPoints(myUV1, myUV2);

        return C;
    }

    /// <summary>Recomputes the UV points after the range changed.</summary>
    internal override void Update()
    {
        CheckMutable();
        double f = First();
        double l = Last();
        bool isneg = Precision.IsNegativeInfinite(f);
        bool ispos = Precision.IsPositiveInfinite(l);
        if (!isneg)
        {
            myPCurve!.D0(f, out myUV1);
        }

        if (!ispos)
        {
            myPCurve!.D0(l, out myUV2);
        }
    }
}
