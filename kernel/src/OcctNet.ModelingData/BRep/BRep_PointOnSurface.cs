// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The representation of a vertex by parameters (U, V) on a surface.</summary>
public class BRep_PointOnSurface : BRep_PointsOnSurface
{
    private double myParameter2;

    /// <summary>The parameters (<paramref name="P1"/>, <paramref name="P2"/>) on <paramref name="S"/> (frozen here).</summary>
    public BRep_PointOnSurface(double P1, double P2, Geom_Surface? S, in TopLoc_Location L) : base(P1, S, L)
        => myParameter2 = P2;

    /// <summary>True.</summary>
    public override bool IsPointOnSurface() => true;

    /// <summary>A point on the surface <paramref name="S"/>.</summary>
    public override bool IsPointOnSurface(Geom_Surface? S, in TopLoc_Location L)
        => ReferenceEquals(Surface(), S) && (Location() == L);

    public override double Parameter2() => myParameter2;

    internal override void Parameter2(double P)
    {
        CheckMutable();
        myParameter2 = P;
    }
}
