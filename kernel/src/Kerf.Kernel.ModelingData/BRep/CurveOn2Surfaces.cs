// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOn2Surfaces.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_CurveOn2Surfaces.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The continuity of an edge between two surfaces.</summary>
internal class CurveOn2Surfaces : CurveRepresentation
{
    private readonly Surface? _surface;
    private readonly Surface? _surface2;
    private readonly Placement _location2;
    private Continuity _continuity;

    /// <summary>The continuity <paramref name="c"/> between <paramref name="s1"/> and <paramref name="s2"/> (frozen here).</summary>
    public CurveOn2Surfaces(Surface? s1,
                                 Surface? s2,
                                 in Placement l1,
                                 in Placement l2,
                                 Continuity c)
        : base(l1)
    {
        s1?.Freeze(); // Kerf: attached geometry is frozen
        s2?.Freeze();
        _surface = s1;
        _surface2 = s2;
        _location2 = l2;
        _continuity = c;
    }

    /// <summary>True.</summary>
    public override bool IsRegularity() => true;

    /// <summary>A curve on two surfaces (continuity), in either order.</summary>
    public override bool IsRegularity(Surface? s1, Surface? s2, in Placement l1, in Placement l2)
        => (ReferenceEquals(_surface, s1) && ReferenceEquals(_surface2, s2) && _location.IsEqual(l1) && _location2.IsEqual(l2))
           || (ReferenceEquals(_surface, s2) && ReferenceEquals(_surface2, s1) && _location.IsEqual(l2) && _location2.IsEqual(l1));

    /// <summary>Throws: a continuity has no points.</summary>
    /// <exception cref="NullObjectException">Always.</exception>
    public void D0(double u, out Point3D p) => throw new NullObjectException("BRep_CurveOn2Surfaces::D0");

    public override Surface? Surface() => _surface;

    public override Surface? Surface2() => _surface2;

    public override Placement Location2() => _location2;

    public override Continuity Continuity() => _continuity;

    internal override void Continuity(Continuity c)
    {
        CheckMutable();
        _continuity = c;
    }

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        CurveOn2Surfaces c =
            new CurveOn2Surfaces(Surface(), Surface2(), Location(), Location2(), _continuity);
        return c;
    }
}
