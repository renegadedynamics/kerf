// Port of: src/ModelingData/TKBRep/BRep/BRep_Curve3D.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Curve3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by a 3D curve.</summary>
public class Curve3DRepresentation : GCurve
{
    private Curve3D? _curve;

    /// <summary>The curve <paramref name="c"/> (frozen here) with location <paramref name="l"/>, over its whole range.</summary>
    public Curve3DRepresentation(Curve3D? c, in Placement l)
        : base(l, c is null ? RealFirst() : c.FirstParameter(), c is null ? RealLast() : c.LastParameter())
    {
        c?.Freeze(); // OCCT.NET: attached geometry is frozen
        _curve = c;
    }

    /// <summary>The point at parameter <paramref name="u"/>.</summary>
    public override void D0(double u, out Point3D p)
    {
        // should be D0 NYI
        p = _curve!.Value(u);
    }

    /// <summary>True.</summary>
    public override bool IsCurve3D => true;

    public override Curve3D? Curve3D() => _curve;

    internal override void Curve3D(Curve3D? c)
    {
        CheckMutable();
        c?.Freeze(); // OCCT.NET: attached geometry is frozen
        _curve = c;
    }

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        Curve3DRepresentation c = new Curve3DRepresentation(_curve, Location());
        c.SetRange(First(), Last());
        return c;
    }
}
