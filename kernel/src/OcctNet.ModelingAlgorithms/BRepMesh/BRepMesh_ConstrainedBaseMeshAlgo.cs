// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ConstrainedBaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ConstrainedBaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>The base of the face algorithms building the triangulation by a Delaunay approach.</summary>
public abstract class BRepMesh_ConstrainedBaseMeshAlgo : BRepMesh_BaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    protected BRepMesh_ConstrainedBaseMeshAlgo()
    {
    }

    /// <summary>The numbers of cells of the circles' acceleration grid along U and V; (-1, -1) by default.</summary>
    protected virtual (int first, int second) getCellsCount(int theVerticesNb)
    {
        _ = theVerticesNb;
        return (-1, -1);
    }

    /// <summary>
    /// Processes the generated mesh; does nothing by default. Called from <c>generateMesh</c> in the subclasses.
    /// </summary>
    protected virtual void postProcessMesh(BRepMesh_Delaun theMesher, ProgressRange theRange)
    {
        _ = theMesher;
        _ = theRange;
    }
}
