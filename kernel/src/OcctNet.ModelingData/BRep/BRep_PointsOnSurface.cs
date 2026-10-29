// Port of: src/ModelingData/TKBRep/BRep/BRep_PointsOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointsOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The root of the representations of a vertex by parameters on a surface.</summary>
public abstract class BRep_PointsOnSurface : BRep_PointRepresentation
{
    private Geom_Surface? mySurface;

    protected BRep_PointsOnSurface(double P, Geom_Surface? S, in TopLoc_Location L) : base(P, L)
    {
        S?.Freeze(); // OCCT.NET: attached geometry is frozen
        mySurface = S;
    }

    public override Geom_Surface? Surface() => mySurface;

    internal override void Surface(Geom_Surface? S)
    {
        CheckMutable();
        S?.Freeze(); // OCCT.NET: attached geometry is frozen
        mySurface = S;
    }
}
