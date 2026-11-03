// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurveOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurveOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>The representation of a vertex by a parameter on a curve on a surface.</summary>
public class BRep_PointOnCurveOnSurface : BRep_PointsOnSurface
{
    private Geom2d_Curve? myPCurve;

    /// <summary>The parameter <paramref name="P"/> on the pcurve <paramref name="C"/> on <paramref name="S"/> (frozen here).</summary>
    public BRep_PointOnCurveOnSurface(double P, Geom2d_Curve? C, Geom_Surface? S, in Placement L) : base(P, S, L)
    {
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myPCurve = C;
    }

    /// <summary>True.</summary>
    public override bool IsPointOnCurveOnSurface() => true;

    /// <summary>A point on the 2D curve <paramref name="PC"/> on the surface <paramref name="S"/>.</summary>
    public override bool IsPointOnCurveOnSurface(Geom2d_Curve? PC, Geom_Surface? S, in Placement L)
        => ReferenceEquals(myPCurve, PC) && ReferenceEquals(Surface(), S) && (Location() == L);

    public override Geom2d_Curve? PCurve() => myPCurve;

    internal override void PCurve(Geom2d_Curve? C)
    {
        CheckMutable();
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myPCurve = C;
    }
}
