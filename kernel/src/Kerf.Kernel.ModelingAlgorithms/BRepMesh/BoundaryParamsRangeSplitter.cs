// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_BoundaryParamsRangeSplitter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>The NURBS range splitter whose grid parameters are those of the border points.</summary>
internal class BoundaryParamsRangeSplitter : NurbsRangeSplitter
{
    /// <summary>A splitter.</summary>
    public BoundaryParamsRangeSplitter()
    {
    }

    /// <summary>Registers a border point and its parameters.</summary>
    public override void AddPoint(in Point2D point)
    {
        base.AddPoint(point);
        GetParametersU().Add(point.X);
        GetParametersV().Add(point.Y);
    }

    /// <summary>The parameters are the border points': nothing to initialise.</summary>
    protected override bool InitParameters() => true;
}
