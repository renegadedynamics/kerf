// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Model.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Model.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Model;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing.Data;

/// <summary>The default discrete model of a shape.</summary>
/// <remarks>
/// OCCT's model owns a thread-safe incremental allocator for all its entities; the port's entities are ordinary
/// objects. <see cref="AddFace"/> and <see cref="AddEdge"/> aren't thread-safe, as in OCCT: the mesher adds them
/// before its parallel loops.
/// </remarks>
public class ModelData : ModelModel
{
    private double _maxSize;
    private readonly NCollection_DynamicArray<FaceModel> _dFaces;
    private readonly NCollection_DynamicArray<EdgeModel> _dEdges;

    /// <summary>An empty model of the shape.</summary>
    public ModelData(in Shape shape) : base(shape)
    {
        _maxSize = 0.0;
        _dFaces = new NCollection_DynamicArray<FaceModel>(256);
        _dEdges = new NCollection_DynamicArray<EdgeModel>(256);
    }

    /// <summary>The maximum size of the shape's bounding box.</summary>
    public override double GetMaxSize() => _maxSize;

    /// <summary>Sets the maximum size of the shape's bounding box.</summary>
    public void SetMaxSize(double value) => _maxSize = value;

    /// <inheritdoc/>
    public override int FacesNb() => _dFaces.Length();

    /// <inheritdoc/>
    public override FaceModel AddFace(in Face face)
    {
        FaceModel aFace = new FaceData(face);
        return _dFaces.Append(aFace);
    }

    /// <inheritdoc/>
    public override FaceModel GetFace(int index) => _dFaces.Value(index);

    /// <inheritdoc/>
    public override int EdgesNb() => _dEdges.Length();

    /// <inheritdoc/>
    public override EdgeModel AddEdge(in Edge edge)
    {
        EdgeModel aEdge = new EdgeData(edge);
        return _dEdges.Append(aEdge);
    }

    /// <inheritdoc/>
    public override EdgeModel GetEdge(int index) => _dEdges.Value(index);
}
