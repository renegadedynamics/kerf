// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretRoot.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Occt;

/// <summary>The common interface of the meshing algorithms made by the mesh factories.</summary>
/// <remarks>A working object: the shape is set, then <see cref="Perform"/> meshes it.</remarks>
public abstract class BRepMesh_DiscretRoot : Transient
{
    protected Shape myShape;
    protected bool myIsDone;

    /// <summary>An algorithm without a shape.</summary>
    protected BRepMesh_DiscretRoot()
    {
        myShape = new Shape();
        myIsDone = false;
    }

    /// <summary>Sets the shape to triangulate.</summary>
    public void SetShape(in Shape theShape) => myShape = theShape;

    /// <summary>The shape to triangulate (for <see cref="BRepMesh_IncrementalMesh"/>, the meshed shape once done).</summary>
    public Shape Shape() => myShape;

    /// <summary>True if the triangulation was performed and succeeded.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Computes the triangulation of the shape.</summary>
    public abstract void Perform(ProgressRange theRange = default);

    /// <summary>Sets the IsDone flag.</summary>
    protected void setDone() => myIsDone = true;

    /// <summary>Clears the IsDone flag.</summary>
    protected void setNotDone() => myIsDone = false;

    /// <summary>Initialization hook of the plugins.</summary>
    protected virtual void init()
    {
    }
}
