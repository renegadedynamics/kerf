// Port of: src/ModelingData/TKBRep/BRep/BRep_Polygon3D.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_Polygon3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The representation of an edge by a 3D polygon.</summary>
internal class Polygon3DRepresentation : CurveRepresentation
{
    private Polygon3D? _polygon3D;

    /// <summary>The polygon <paramref name="p"/> (frozen here) with location <paramref name="l"/>.</summary>
    public Polygon3DRepresentation(Polygon3D? p, in Placement l) : this(p, l, true)
    {
    }

    /// <summary>
    /// The polygon <paramref name="p"/> with location <paramref name="l"/>, frozen if <paramref name="toFreeze"/>
    /// (the mesher's non-freezing attach, decision 488).
    /// </summary>
    internal Polygon3DRepresentation(Polygon3D? p, in Placement l, bool toFreeze) : base(l)
    {
        if (toFreeze)
        {
            p?.Freeze(); // Kerf: attached meshes are frozen
        }
        _polygon3D = p;
    }

    /// <summary>True.</summary>
    public override bool IsPolygon3D => true;

    public override Polygon3D? Polygon3D() => _polygon3D;

    internal override void Polygon3D(Polygon3D? p) => Polygon3D(p, true);

    /// <summary>Sets the polygon, frozen if <paramref name="toFreeze"/> (decision 488).</summary>
    internal void Polygon3D(Polygon3D? p, bool toFreeze)
    {
        CheckMutable();
        if (toFreeze)
        {
            p?.Freeze(); // Kerf: attached meshes are frozen
        }
        _polygon3D = p;
    }

    internal override void FreezeContents() => _polygon3D?.Freeze();

    /// <summary>A copy of this representation.</summary>
    public override CurveRepresentation Copy()
    {
        Polygon3DRepresentation p = new Polygon3DRepresentation(_polygon3D, Location());
        return p;
    }
}
