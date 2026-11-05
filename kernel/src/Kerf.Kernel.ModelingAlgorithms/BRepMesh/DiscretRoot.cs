// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretRoot.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DiscretRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>The common interface of the meshing algorithms made by the mesh factories.</summary>
/// <remarks>A working object: the shape is set, then <see cref="Perform"/> meshes it.</remarks>
internal abstract class DiscretRoot : Transient
{
    protected Shape _shape;
    protected bool _isDone;

    /// <summary>An algorithm without a shape.</summary>
    protected DiscretRoot()
    {
        _shape = new Shape();
        _isDone = false;
    }

    /// <summary>Sets the shape to triangulate.</summary>
    public void SetShape(in Shape shape) => _shape = shape;

    /// <summary>The shape to triangulate (for <see cref="IncrementalMesh"/>, the meshed shape once done).</summary>
    public Shape Shape() => _shape;

    /// <summary>True if the triangulation was performed and succeeded.</summary>
    public bool IsDone => _isDone;

    /// <summary>Computes the triangulation of the shape.</summary>
    public abstract void Perform(ProgressRange range = default);

    /// <summary>Sets the IsDone flag.</summary>
    protected void SetDone() => _isDone = true;

    /// <summary>Clears the IsDone flag.</summary>
    protected void SetNotDone() => _isDone = false;

    /// <summary>Initialization hook of the plugins.</summary>
    protected virtual void Init()
    {
    }
}
