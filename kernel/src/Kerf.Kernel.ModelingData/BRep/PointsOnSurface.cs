// Port of: src/ModelingData/TKBRep/BRep/BRep_PointsOnSurface.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_PointsOnSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Brep;

/// <summary>The root of the representations of a vertex by parameters on a surface.</summary>
internal abstract class PointsOnSurface : PointRepresentation
{
    private Surface? _surface;

    protected PointsOnSurface(double p, Surface? s, in Placement l) : base(p, l)
    {
        s?.Freeze(); // OCCT.NET: attached geometry is frozen
        _surface = s;
    }

    public override Surface? Surface() => _surface;

    internal override void Surface(Surface? s)
    {
        CheckMutable();
        s?.Freeze(); // OCCT.NET: attached geometry is frozen
        _surface = s;
    }
}
