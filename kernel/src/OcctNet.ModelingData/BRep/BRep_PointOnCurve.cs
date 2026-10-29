// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurve.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointOnCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The representation of a vertex by a parameter on a 3D curve.</summary>
public class BRep_PointOnCurve : BRep_PointRepresentation
{
    private Geom_Curve? myCurve;

    /// <summary>The parameter <paramref name="P"/> on <paramref name="C"/> (frozen here).</summary>
    public BRep_PointOnCurve(double P, Geom_Curve? C, in TopLoc_Location L) : base(P, L)
    {
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myCurve = C;
    }

    /// <summary>True.</summary>
    public override bool IsPointOnCurve() => true;

    /// <summary>A point on the curve <paramref name="C"/>.</summary>
    public override bool IsPointOnCurve(Geom_Curve? C, in TopLoc_Location L)
        => ReferenceEquals(myCurve, C) && (Location() == L);

    public override Geom_Curve? Curve() => myCurve;

    internal override void Curve(Geom_Curve? C)
    {
        CheckMutable();
        C?.Freeze(); // OCCT.NET: attached geometry is frozen
        myCurve = C;
    }
}
