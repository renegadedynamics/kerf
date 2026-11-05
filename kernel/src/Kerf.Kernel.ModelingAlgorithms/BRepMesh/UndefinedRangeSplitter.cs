// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_UndefinedRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_UndefinedRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Meshing;

/// <summary>The NURBS range splitter for surfaces of other types: one interval where the surface defines none.</summary>
internal class UndefinedRangeSplitter : NurbsRangeSplitter
{
    /// <summary>A splitter.</summary>
    public UndefinedRangeSplitter()
    {
    }

    /// <summary>The number of intervals computed from the available geometrical parameters: 1.</summary>
    protected override int GetUndefinedIntervalNb(SurfaceAdaptorBase surface,
                                                  bool isU,
                                                  Continuity continuity)
    {
        return 1;
    }
}
