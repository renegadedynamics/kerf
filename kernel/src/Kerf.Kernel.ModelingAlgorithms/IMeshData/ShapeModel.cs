// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Shape.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Shape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Kerf.Engine.Meshing.Model;

/// <summary>
/// The base of the discrete model's entities and the mesher's tools that keep a reference shape.
/// </summary>
/// <remarks>
/// The discrete model is a working object of the mesher (decision 21): its entities are changed in place by the
/// mesher's algorithms, as in OCCT. Handles of IMeshData (<c>IMeshData::IFaceHandle</c>, ...) are references to the
/// classes; its raw pointers (<c>IMeshData::IFacePtr</c>, <c>IEdgePtr</c>) are the same references, and maps keyed by
/// them compare by reference and iterate in insertion order (decision 490).
/// </remarks>
internal abstract class ShapeModel : Transient
{
    private Shape _shape;

    /// <summary>An entity with a null shape.</summary>
    protected ShapeModel()
    {
        _shape = new Shape();
    }

    /// <summary>An entity of the given shape.</summary>
    protected ShapeModel(in Shape shape)
    {
        _shape = shape;
    }

    /// <summary>Assigns the shape to the discrete shape.</summary>
    public void SetShape(in Shape shape) => _shape = shape;

    /// <summary>The shape assigned to the discrete shape.</summary>
    public Shape GetShape() => _shape;
}
