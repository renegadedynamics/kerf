// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOn2Surfaces.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOn2Surfaces.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>The continuity of an edge between two surfaces.</summary>
public class BRep_CurveOn2Surfaces : BRep_CurveRepresentation
{
    private readonly Geom_Surface? mySurface;
    private readonly Geom_Surface? mySurface2;
    private readonly Placement myLocation2;
    private Continuity myContinuity;

    /// <summary>The continuity <paramref name="C"/> between <paramref name="S1"/> and <paramref name="S2"/> (frozen here).</summary>
    public BRep_CurveOn2Surfaces(Geom_Surface? S1,
                                 Geom_Surface? S2,
                                 in Placement L1,
                                 in Placement L2,
                                 Continuity C)
        : base(L1)
    {
        S1?.Freeze(); // OCCT.NET: attached geometry is frozen
        S2?.Freeze();
        mySurface = S1;
        mySurface2 = S2;
        myLocation2 = L2;
        myContinuity = C;
    }

    /// <summary>True.</summary>
    public override bool IsRegularity() => true;

    /// <summary>A curve on two surfaces (continuity), in either order.</summary>
    public override bool IsRegularity(Geom_Surface? S1, Geom_Surface? S2, in Placement L1, in Placement L2)
        => (ReferenceEquals(mySurface, S1) && ReferenceEquals(mySurface2, S2) && myLocation == L1 && myLocation2 == L2)
           || (ReferenceEquals(mySurface, S2) && ReferenceEquals(mySurface2, S1) && myLocation == L2 && myLocation2 == L1);

    /// <summary>Throws: a continuity has no points.</summary>
    /// <exception cref="NullObjectException">Always.</exception>
    public void D0(double U, out Point3D P) => throw new NullObjectException("BRep_CurveOn2Surfaces::D0");

    public override Geom_Surface? Surface() => mySurface;

    public override Geom_Surface? Surface2() => mySurface2;

    public override Placement Location2() => myLocation2;

    public override Continuity Continuity() => myContinuity;

    internal override void Continuity(Continuity C)
    {
        CheckMutable();
        myContinuity = C;
    }

    /// <summary>A copy of this representation.</summary>
    public override BRep_CurveRepresentation Copy()
    {
        BRep_CurveOn2Surfaces C =
            new BRep_CurveOn2Surfaces(Surface(), Surface2(), Location(), Location2(), myContinuity);
        return C;
    }
}
