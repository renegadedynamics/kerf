// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_UndefinedRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_UndefinedRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The NURBS range splitter for surfaces of other types: one interval where the surface defines none.</summary>
public class BRepMesh_UndefinedRangeSplitter : BRepMesh_NURBSRangeSplitter
{
    /// <summary>A splitter.</summary>
    public BRepMesh_UndefinedRangeSplitter()
    {
    }

    /// <summary>The number of intervals computed from the available geometrical parameters: 1.</summary>
    protected override int getUndefinedIntervalNb(Adaptor3d_Surface theSurface,
                                                  bool isU,
                                                  Continuity theContinuity)
    {
        return 1;
    }
}
